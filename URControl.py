# -*- coding: utf-8 -*-
"""
Created on Thu Aug 29 11:15:19 2024

@author: juandavc
"""

import socket
import struct
import threading
from MonitorData import RobotData


class UR5CB2Control:
    def __init__(self, robot_ip, robot_port=30001):
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
        self.data =  RobotData(self.robot_ip)

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
            
    def send_command(self, command):
        """
        Sends a custom URScript command to the robot.
        """
        try:
            # Ensure the command ends with a newline character for execution
            if not command.endswith('\n'):
                command += '\n'
            self.socket.send(command.encode('utf-8'))
            print(f"Command sent: {command.strip()}")
        except Exception as e:
            print(f"Error sending command: {e}")
            
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
        
    def set_digital_output(self, output_number, state):
        """
        Sets a digital output to the specified state and optionally waits for a specified delay.
        :param output_number: The digital output number (index)
        :param state: True for on, False for off
        """
        # Format the command string based on the specified output number and state
        command = f'set_digital_out({output_number},{str(state)})'
        self.send_command(command)
    
    def set_analog_output(self, output_number, value):
        """
        Sets a digital output to the specified state and optionally waits for a specified delay.
        :param output_number: The analog output number [1,1]
        :param value: 0.0 - 1.0
        """
        if (value >= 0 and value <= 1.0) and (output_number == 0 or output_number ==1):     
            # Format the command string based on the specified output number and value
            command = f'set_analog_out({output_number},{value:.2f})'
            self.send_command(command)
        else:
            print("invalid parameters")
        
    def close(self):
        """Closes the TCP connection and stops the monitoring thread."""
        self.data.stop_monitoring()
        if self.socket:
            self.socket.close()
            print("Connection closed.")
            
class UR5CommandBuilder:
    def __init__(self):
        """
        Initializes the UR5CommandBuilder class with an empty command script.
        """
        self.command = "def custom_command():\n\nend\ncustom_command()\n"
        
    def add_command(self, custom_command):
        """
        Adds a custom URScript command to the command script.
        
        :param custom_command: The custom command string to add
        """
        # Ensure the custom command ends with a newline
        if not custom_command.endswith('\n'):
            custom_command += '\n'
        custom_command = f"    {custom_command}"  # Indent to match URScript format
        self._insert_into_command(custom_command)
        
    def movel(self, pose, v=0.1, a=0.1):
        """
        Adds a movel command to the custom command script.
    
        :param pose: A list or tuple containing the target pose [x, y, z, rx, ry, rz]
        :param v: Speed for the movement (optional, default=0.1)
        :param a: Acceleration for the movement (optional, default=0.1)
        """
        [x, y, z, rx, ry, rz] = pose
        movel_command = f"    movel(p[{x:.4f},{y:.4f},{z:.4f},{rx:.4f},{ry:.4f},{rz:.4f}], a={a}, v={v})\n"
        self._insert_into_command(movel_command)
        
    def movel2(self, q, v=0.1, a=0.1):
        """
        Adds a movel command to the custom command script.
    
        :param pose: A list or tuple containing the target pose [x, y, z, rx, ry, rz]
        :param v: Speed for the movement (optional, default=0.1)
        :param a: Acceleration for the movement (optional, default=0.1)
        """
        [q1, q2, q3, q4, q5, q6] = q
        movel_command = f"    movel([{q1:.4f},{q2:.4f},{q3:.4f},{q4:.4f},{q5:.4f},{q6:.4f}], a={a}, v={v})\n"
        self._insert_into_command(movel_command)

    def movej(self, pose, v=0.1, a=0.1, r=0):
        """
        Adds a movej command to the custom command script.
    
        :param pose: A list or tuple containing the target pose [x, y, z, rx, ry, rz]
        :param v: Speed for the movement (optional, default=0.1)
        :param a: Acceleration for the movement (optional, default=0.1)
        """
        [x, y, z, rx, ry, rz] = pose
        movel_command = f"    movej(p[{x:.4f},{y:.4f},{z:.4f},{rx:.4f},{ry:.4f},{rz:.4f}], a={a}, v={v}, r={r})\n"
        self._insert_into_command(movel_command)
    
    def movej2(self, q, v=0.1, a=0.1, r=0):
        """
        Adds a movej command to the custom command script.
    
        :param pose: A list or tuple containing the target pose [x, y, z, rx, ry, rz]
        :param v: Speed for the movement (optional, default=0.1)
        :param a: Acceleration for the movement (optional, default=0.1)
        """
        [q1, q2, q3, q4, q5, q6] = q
        movel_command = f"    movej([{q1:.4f},{q2:.4f},{q3:.4f},{q4:.4f},{q5:.4f},{q6:.4f}], a={a}, v={v}, r={r})\n"
        self._insert_into_command(movel_command)
        
    def sleep(self, duration):
        """
        Adds a sleep command to the custom command script.
        
        :param duration: Time to wait in seconds
        """
        sleep_command = f"    sleep({duration})\n"
        self._insert_into_command(sleep_command)

    def set_digital_output(self, output_number, state):
        """
        Adds a set_digital_output command to the custom command script.
        
        :param output_number: The digital output number (index)
        :param state: True for on, False for off
        """
        state_value = 'True' if state else 'False'
        set_digital_command = f"    set_digital_out({output_number}, {state_value})\n"
        self._insert_into_command(set_digital_command)
    
    def set_analog_output(self, output_number, value):
        """
        Adds a set_standard_analog_out command to the custom command script.
        
        :param output_number: The analog output number [0,1]
        :param value: 0.0 - 1.0
        """
        if (value >= 0 and value <= 1.0) and (output_number == 0 or output_number ==1):     
            # Format the command string based on the specified output number and value
            set_analog_command = f"    set_analog_out({output_number}, {value:.2f})\n"
            self._insert_into_command(set_analog_command)
        else:
            print("invalid parameters")
            

    def _insert_into_command(self, new_command):
        """
        Helper method to insert new command lines into the custom command script.
        
        :param new_command: The new command line to add
        """
        # Insert new command before the "end" line
        self.command = self.command.replace("\nend\n", f"{new_command}end\n")

    def get_command(self):
        """
        Returns the complete command script.
        
        :return: The full command string
        """
        return self.command

    def clear_command(self):
        """
        Clears the current command script to start a new one.
        """
        self.command = "def custom_command():\n\nend\ncustom_command()\n"
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
