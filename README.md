# HW1

Dylan Losey, Virginia Tech.

In this homework assignment we will record and replay demonstrations on a robot arm.

## Install and Run

```bash

# Download
git clone https://github.com/vt-hri/HW1.git
cd HW1

# Create and source virtual environment
# If you are using Mac or Conda, modify these two lines as shown in [HW0](https://github.com/vt-hri/HW0)
python3 -m venv venv
source venv/bin/activate

# Install dependencies
# If you are using Mac or Conda, modify this line as shown in [HW0](https://github.com/vt-hri/HW0)
pip install numpy pybullet

# Run the script
python main.py
```

## Expected Output

Use your keyboard to teleoperate the robot.
See `teleop.py` to understand which keys map to which axis of robot motion.
<img src="env.gif" width="750">

## Assignment

Modify the provided code to complete the following steps:
1. Implement a limiter to prevent users from teleoperating the robot into the table or out of the robot's workspace.
2. Implement a script so that users can "record" a state by pressing the toggle button. The recorded states should be saved to a JSON file (or similar output).
3. Provide and record a demonstration lifting one block and placing it on the other.
4. Create a new script (called `replay.py`) that will replay the recorded trajectory. This file should load the JSON file and control the robot to move to each of the recorded states.
5. In `replay.py`, change the location of one of the blocks. Purely replaying the recorded trajectory should now fail (since the block location has changed). Enable the user to intervene and modify the robot's motion.
6. Your final version of `replay.py` should control the robot to follow the keypoints saved in the JSON file. The user should be able to press a button and teleoperate the robot. When the user is done, the robot should resume its original trajectory (while skipping the modified parts).
