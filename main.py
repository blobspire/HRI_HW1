from fileinput import filename
import pybullet as p
import pybullet_data
import numpy as np
import os
import time
import json
from robot import Panda
from teleop import KeyboardController


# parameters
control_dt = 1. / 240.

# create simulation and place camera
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
# disable keyboard shortcuts so they do not interfere with keyboard control
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.resetDebugVisualizerCamera(cameraDistance=1.0, 
                                cameraYaw=40.0,
                                cameraPitch=-30.0, 
                                cameraTargetPosition=[0.5, 0.0, 0.2])

# load the objects
urdfRootPath = pybullet_data.getDataPath()
plane = p.loadURDF(os.path.join(urdfRootPath, "plane.urdf"), basePosition=[0, 0, -0.625])
table = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[0.5, 0, -0.625])
cube1 = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), basePosition=[0.6, -0.2, 0.05])
cube2 = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"),
                    basePosition=[0.7, 0.2, 0.05], baseOrientation=p.getQuaternionFromEuler([0, 0, 0.7]))

# load the robot
jointStartPositions = [0.0, 0.0, 0.0, -2*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.0, 0.0, 0.04, 0.04]
panda = Panda(basePosition=[0, 0, 0],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                jointStartPositions=jointStartPositions)

# teleoperation interface
teleop = KeyboardController()

# run simulation
# you can teleoperate the robot using the keyboard;
# see "teleop.py" for the mapping between keys and motions
state = panda.get_state()
target_position = state["ee-position"]
target_quaternion = state['ee-quaternion']

# User defined constraints
MIN_HEIGHT = 0.01
ERROR = 0.001 # Threshold for considering the robot to have reached the target position

gripper_open = True
filename = 'state.json'

# Remove the existing state.json file if it exists to start fresh
if os.path.exists(filename):
    os.remove(filename)

while True:
    # update the target pose
    action = teleop.get_action()
    target_position = target_position + action[0:3]

    # Ensure that the target position is not within the table
    if target_position[2] < MIN_HEIGHT:
        target_position[2] = MIN_HEIGHT

    target_quaternion = p.multiplyTransforms([0, 0, 0], p.getQuaternionFromEuler(action[3:6]),
                                                [0, 0, 0], target_quaternion)[1]
    # move to the target pose
    panda.move_to_pose(ee_position=target_position, ee_quaternion=target_quaternion)

    # open or close the gripper
    if action[6] == +1:
        panda.open_gripper()
        gripper_open = True
    elif action[6] == -1:
        panda.close_gripper()
        gripper_open = False

    # print when "." is pressed and save the robot state to a JSON file
    if action[7] == +1:
        print("button pressed")
        # Capture the current state of the robot and save it to a JSON file
        state = panda.get_state()
        state["gripper-open"] = gripper_open

        past_states = [] # Default to empty list in case file does not exist
        if os.path.exists(filename):
            # Read past states (to append)
            with open(filename, 'r') as f:
                past_states = json.load(f)

        # Now write the past states plus the new state
        with open(filename, 'w') as f:
            json.dump(past_states + [state] if len(past_states) > 0 else [state], f) # Don't append empty list if no past state

        # Sleep for 0.5s to prevent multiple prints/writes if the button is held down
        time.sleep(0.5)

    # Load the robot states from a JSON file when "," is pressed
    # The states are a list of dictionaries of states. Load each one sequentially with a 1s delay to see the robot move through the states.
    if action[7] == -1:
        print("loading state")
        # Load the state from the JSON file
        with open('state.json', 'r') as f:
            states = json.load(f)
        for state in states:
            # Set the robot to the loaded joint positions
            # joint_positions = state["joint-position"]
            # panda.reset(joint_positions)
            # Update the robot state
            target_position = state["ee-position"]
            target_quaternion = state["ee-quaternion"]
            gripper_open = state["gripper-open"]
            if gripper_open:
                panda.open_gripper()
            else:
                panda.close_gripper()
            
            env_state = panda.get_state()
            # TODO trying to make it move to the target position while it is not there. rather than generic for loop for x steps
            while np.linalg.norm(np.array(env_state["ee-position"]) - np.array(target_position)) > ERROR:
                panda.move_to_pose(ee_position=target_position, ee_quaternion=target_quaternion, positionGain=0.05)
                p.stepSimulation()
                time.sleep(control_dt)
                env_state = panda.get_state()
        # Sleep for 0.5s to prevent multiple loads if the button is held down
        time.sleep(0.5)

    # step simulation
    p.stepSimulation()
    time.sleep(control_dt)