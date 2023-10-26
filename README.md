# control_ur_robot_with_omega_haptic

Control UR5e Robot with Omega3 Haptic Device.

## Installation

'''bash
pip install urx
'''

## Initialization

Initialize Omega Haptic Device.

'''bash
cd /catkin_ws/src/(your_project_file)

./omega_haptic/sdk-3.16.1/bin/HapticInit

'''

Click 'Initialize' button.

## Run

'''bash
rosrun ur5e_urx_control ur5e_test.py
'''
'''bash
rosrun omega_haptic omega_haptic
'''

## Issues

URX package is not working well in python3.8. You should change some variables in urx library.
