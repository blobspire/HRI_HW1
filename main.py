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
while True:
    # update the target pose
    action = teleop.get_action()
    target_position = target_position + action[0:3]
    target_quaternion = p.multiplyTransforms([0, 0, 0], p.getQuaternionFromEuler(action[3:6]),
                                                [0, 0, 0], target_quaternion)[1]
    # move to the target pose
    panda.move_to_pose(ee_position=target_position, ee_quaternion=target_quaternion)

    # open or close the gripper
    if action[6] == +1:
        panda.open_gripper()
    elif action[6] == -1:
        panda.close_gripper()

    # print when "." is pressed
    if action[7] == +1:
        print("button pressed")

    # step simulation
    p.stepSimulation()
    time.sleep(control_dt)