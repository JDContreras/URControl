# -*- coding: utf-8 -*-
"""
Created on Thu Aug 29 12:27:38 2024

@author: juandavc
"""

from URControl import UR5CB2Control, UR5CommandBuilder
import time
import numpy as np
from transformations import add_roll, add_x, add_z

"""
weights:
lentil - 7.5 kg
chick fil a - 3.85
"""

def get_parameters(width, length, dz):
    angle = round(np.arcsin(dz/width),3)
    dx = round(length*np.sin(angle),3)
    return angle, dx

def test_com():
    robot = UR5CB2Control("192.168.1.109")
    msg = UR5CommandBuilder()
    msg.set_analog_output(0, 0.9)
    msg.set_digital_output(2, True)
    msg.sleep(1)
    msg.set_analog_output(0, 0.0)
    msg.set_digital_output(2, False)
    robot.send_command(msg.get_command())
    robot.close()
    
def perform_robot_motion(dz, W, L, vel, accel):
    
    # Define output addresses
    analog_output_fingers = 0
    digital_output_vacuum = 0
    digital_output_fingers = 1
    finger_distance = 0.9
    # Step 1: Calculate parameters
    A, dx = get_parameters(W, L, dz)
    dx0 = 0
    approach_distance = 0.04
    A2 = A + np.radians(2)

    # Step 2: Initialize robot and starting positions
    robot = UR5CB2Control("169.254.22.200")
    p0 = np.array([0.481, 0.354, 0.202, -1.1625, 1.2719, -1.2720])
    p1 = add_x(p0, approach_distance)
    p2 = add_x(add_z(add_roll(p1, -A), dz), -(dx - dx0))
    p3 = add_roll(p2, A2)
    p4 = add_x(p3, -0.08)
    p5 = add_x(p2, -0.1)
    p6 = add_z(add_x(p0, -0.03),0.03)

    # Step 3: Initialize command builder and prepare the robot
    msg = UR5CommandBuilder()
    msg.set_analog_output(analog_output_fingers, 0.05)
    msg.sleep(2.5)

    # Step 4: Move to initial approach position
    msg.movel(p0, v=vel, a=accel)
    msg.sleep(3)

    # Step 5: Activate vacuum
    msg.set_digital_output(digital_output_vacuum, True)
    msg.movel(p1, v=vel, a=accel)
    msg.sleep(1)
    # Step 6: Lift and rotate the object
    msg.movel(p2, v=vel, a=accel)

    # Step 7: Activate fingers for gripping
    #msg.set_digital_output(digital_output_fingers, True)
    msg.set_analog_output(analog_output_fingers, finger_distance)
    msg.sleep(2.5)

    # Step 8: Move to the final pose
    msg.movel(p3, v=vel, a=accel)
    msg.sleep(1)
    msg.movel(p4, v=vel, a=accel)
    msg.sleep(2)

    # Step 9: Return to the initial pose after the task
    msg.movel(p2, v=vel, a=accel)

    # Step 10: Deactivate vacuum and retract fingers
    msg.set_analog_output(analog_output_fingers, 0.05)
    msg.sleep(4)
    msg.movel(p1, v=vel, a=accel)
    msg.set_digital_output(digital_output_vacuum, False)
    msg.sleep(1)  #wait for the boz to roll down
    #msg.movel(p5, v=vel, a=accel)
    #msg.set_digital_output(digital_output_fingers, False)


    # Step 11: Move to the final resting position
    msg.movel(p6, v=vel, a=accel)

    # Step 12: Send command to robot and close connection
    robot.send_command(msg.get_command())
    robot.close()



# Dictionary containing the parameters for each box
boxes_parameters = {
    1: {
        'name': 'chick fil a Salsa',
        'Weight': 3.85,
        'L': 0.155,
        'W': 0.18,
        'D': 0.18
    },
    2: {
        'name': 'Nature Valey Biscuits',
        'Weight': 3.53,
        'L': 0.135,
        'W': 0.3,
        'D': 0.43
    },
    3: {
        'name': 'Rice Crisps',
        'Weight': 2.26,
        'L': 0.26,
        'W': 0.35,
        'D': 0.4
    },
    4: {
        'name': 'SlimFast',
        'Weight': 4.85,
        'L': 0.13,
        'W': 0.185,
        'D': 0.385
    },
    5: {
        'name': 'SlimFast',
        'Weight': 4.85,
        'L': 0.13,
        'W': 0.385,
        'D': 0.185
    }
}
box_id = 5
box = boxes_parameters[box_id]
W = box['W']
L = box['L']
dz = 0.04
vel = 0.1
accel = 0.1
#perform_robot_motion(dz, W, L, vel, accel)
test_com()