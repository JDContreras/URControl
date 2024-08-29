# -*- coding: utf-8 -*-
"""
Created on Thu Aug 29 12:27:38 2024

@author: juandavc
"""

from URControl import UR5CB2Control

    
def generate_robot_poses(initial_position, x_distance1, z_distance, x_distance2, low_speed, high_speed):
    # Define the target poses
    initial_pose = initial_position

    # First move: Initial position
    pose1 = initial_pose
    accel = 1.2
    # Second move: Move a relative distance in X direction
    pose2 = [
        pose1[0] + x_distance1,
        pose1[1],
        pose1[2],
        pose1[3],
        pose1[4],
        pose1[5],
        low_speed,  # High speed for this move
        accel           # Acceleration (you can adjust if needed)
    ]

    # Third move: Move a relative distance in Z direction
    pose3 = [
        pose2[0],
        pose2[1],
        pose2[2] + z_distance,
        pose2[3],
        pose2[4],
        pose2[5],
        low_speed,         # Standard speed for this move
        accel          # Acceleration (you can adjust if needed)
    ]

    # Fourth move: Move a new distance in X direction at high speed
    pose4 = [
        pose3[0] + x_distance2,
        pose3[1],
        pose3[2],
        pose3[3],
        pose3[4],
        pose3[5],
        high_speed,  # High speed for this move
        accel           # Acceleration (you can adjust if needed)
    ]

    return [pose1, pose2, pose3, pose4]

# Example usage:
initial_position = [0.300, -0.145, 0.245, -2.0, 0.6, -2.0, 0.1, 1.2]
x_distance1 = 0.200  # Move 100 mm in X direction
z_distance = 0.050   # Move 50 mm in Z direction
x_distance2 = -0.250  # Move -50 mm in X direction
low_speed = 0.1   # High speed for first X movement (500 mm/s)
high_speed = 0.40   # High speed for second X movement (400 mm/s)

poses = generate_robot_poses(initial_position, x_distance1, z_distance, x_distance2, low_speed, high_speed)
robot = UR5CB2Control("169.254.22.200")
robot.send_movel_commands(poses)
robot.close()
    
