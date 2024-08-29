# -*- coding: utf-8 -*-
"""
Created on Thu Aug 29 13:20:16 2024

@author: juandavc
"""

from URControl import UR5CB2Control
import time

def test_monitoring(robot_ip):
    robot = UR5CB2Control(robot_ip)
    
    try:
        # Let's monitor for a total of 10 seconds, printing the data every second
        start_time = time.time()
        while time.time() - start_time < 10:  # Continue until 10 seconds have passed
            if robot.pose:
                print(f"Current Pose: {robot.pose}")
            if robot.joints:
                print(f"Joint Angles: {robot.joints}")
            print(f"Robot State: {'Stopped' if robot.state else 'Moving'}")
            time.sleep(1)  # Wait for a second between prints
    finally:
        # Ensure the robot connection is closed properly
        robot.close()

# Example usage
test_monitoring("169.254.22.200")