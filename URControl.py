# -*- coding: utf-8 -*-
"""
Created on Thu Aug 29 11:15:19 2024

@author: juandavc
"""

import socket
import struct
import threading
from MonitorData import RobotData
UR_GET_TIME = 1
UR_GET_JOINT_POSITIONS = 252
UR_GET_JOINT_SPEEDS = 300
UR_GET_JOINT_CURRENTS = 348
UR_GET_TCP_FORCES = 540
UR_GET_MODE = 756
UR_GET_PROGRAM_STATE = 1052
UR_GET_TCP_VECTOR = 444
UR_GET_TCP_SPEED = 492


class UR5CB2Control:
    def __init__(self, robot_ip, robot_port=30002):
        """
        Initializes the UR5CB2Control class with the robot's IP and port.
        Establishes a connection to the robot via TCP.
        """
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.socket = None
        self.pose = []
        self.joints = []
        self.connect()
        self.data =  RobotData(self.socket)

    def connect(self):
        """Establishes a TCP connection to the robot."""
        try:
            self.socket = socket.create_connection((self.robot_ip, self.robot_port))
            print(f"Connected to UR5 robot at {self.robot_ip}:{self.robot_port}")
        except Exception as e:
            print(f"Failed to connect to the robot: {e}")

    def movej(self, x, y, z, rx, ry, rz):
        """
        Moves the robot using the movej URScript command.
        Takes in the target Cartesian coordinates and orientations.
        """
        try:
            command = f"movej(p[{x*0.001:.6f},{y*0.001:.6f},{z*0.001:.6f},{rx:.6f},{ry:.6f},{rz:.6f}])\n"
            self.socket.send(command.encode('utf-8'))
            print(f"Command sent: {command.strip()}")
        except Exception as e:
            print(f"Error sending movej command: {e}")
    def send_movel_commands(self, poses):
        """
        Receives a list of poses and generates URScript text to move the robot through each pose using the movel command.
        Each pose is in the format [x, y, z, rx, ry, rz, v, a].
        The generated commands are then sent to the robot.
        """
        try:
            script_text = "def custom_move():\n"
            
            for pose in poses:
                x, y, z, rx, ry, rz, v, a = pose
                command = f"   movel(p[{x:.4f},{y:.4f},{z:.4f},{rx:.4f},{ry:.4f},{rz:.4f}], a={a}, v={v})\n"
                script_text += command
            
            script_text += "end\n"
            script_text += "custom_move()\n"
            
            # Sending the generated script to the robot
            self.socket.send(script_text.encode('utf-8'))
            print(f"Script sent:\n{script_text}")

        except Exception as e:
            print(f"Error generating or sending movel commands: {e}")

        
    def close(self):
        """Closes the TCP connection and stops the monitoring thread."""
        self.data.stop_monitoring()
        if self.socket:
            self.socket.close()
            print("Connection closed.")
            

# Example usage
# robot = UR5CB2Control("192.168.0.10")
# state = robot.read_state()
# positions = robot.read_joint_positions()
# robot.movej(355, 73, 50, 0.080, -3.109, 0.095)
# robot.close()

# Example usage with send_movel_commands
# robot = UR5CB2Control("192.168.0.10")
# poses = [
#     [0.6206, -0.1497, 0.2609, 2.2919, -2.1463, -0.0555, 0.25, 1.2],
#     [0.6206, -0.1497, 0.3721, 2.2919, -2.1463, -0.0555, 0.25, 1.2],
#     [0.6206, -0.1497, 0.4658, 2.2919, -2.1463, -0.0555, 0.25, 1.2]
# ]
# robot.send_movel_commands(poses)
# robot.close()
