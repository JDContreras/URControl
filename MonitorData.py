# -*- coding: utf-8 -*-
"""
Created on Thu Aug 29 18:07:19 2024

@author: juandavc
"""

from typing import NamedTuple, List
import struct
import socket
import struct
import threading
import time

class CartesianInfo(NamedTuple):
    X: float
    Y: float
    Z: float
    Rx: float
    Ry: float
    Rz: float

class JointData(NamedTuple):
    q_actual: float
    q_target: float
    qd_actual: float
    I_actual: float
    V_actual: float
    T_motor: float
    T_micro: float
    jointMode: int

class RobotModeData(NamedTuple):
    isRobotConnected: bool
    isRealRobotEnabled: bool
    isPowerOnRobot: bool
    isEmergencyStopped: bool
    isSecurityStopped: bool
    isProgramRunning: bool
    isProgramPaused: bool
    robotMode: int
    speedFraction: float

class RobotData:
    def __init__(self, socket):
        self.socket = socket
        self.robot_mode_data = None
        self.cartesian_info = None
        self.joint_data = []
        self.digital_inputs = []
        self.digital_outputs = []
        self.analog_inputs = []
        self.analog_outputs = []
        self.is_running = False

    def read_data(self):
        """Read the latest data from the socket and update attributes."""
        try:
            data = self.socket.recv(1254)  # Receive data from the socket
            messageSize, messageType = struct.unpack_from('!iB', data, 0)

            if messageType == 16:  # Check if no error
                # Call extraction methods for each type of data
                self.extract_robot_mode_data(data)
                self.extract_cartesian_info(data)
                self.extract_joint_data(data)
                self.extract_digital_inputs(data)
                self.extract_digital_outputs(data)
                self.extract_analog_inputs(data)
                self.extract_analog_outputs(data)

        except Exception as e:
            print(f"Error reading data: {e}")

    def extract_robot_mode_data(self, data, offset=5):
        # Assuming 'data' is a bytes object containing the message received from the robot
        # The format string needs to be adjusted based on actual data structure:
        # Each '?' in the format string represents a boolean value.
        data_format = '!7?'  # Update this if there are different types or more/less data
        self.robot_mode_data = RobotModeData._make(struct.unpack_from(data_format, data, offset))
    
    def extract_cartesian_info(self, data, offset=256):
        """Extract and update cartesian info from the given data starting at the specified offset."""
        data_format = '!dddddd'  # Format for 6 double precision floats.
        # Unpack the data directly into a new NamedTuple instance and assign it to the class attribute.
        self.cartesian_info = CartesianInfo._make(struct.unpack_from(data_format, data, offset))
        print("Cartesian info updated:", self.cartesian_info)
        
    def extract_joint_data(self, data, offset=300):
        """Extract and update joint data from the given data starting at the specified offset."""
        self.joint_data.clear()  # Clear existing joint data
        for _ in range(6):  # Assuming there are 6 joints
            joint_info = struct.unpack_from('!dddffffB', data, offset)
            self.joint_data.append(JointData._make(joint_info))
            offset += 41  # Move to the next joint's data    
 
    def extract_digital_inputs(self, data, offset=400):
        """Extract digital inputs from data starting at the specified offset."""
        digital_input_bits = struct.unpack_from('!h', data, offset)[0]
        self.digital_inputs = [bool(digital_input_bits & (1 << n)) for n in range(16)]  # Assuming 16 inputs

    def extract_digital_outputs(self, data, offset=500):
        """Extract digital outputs from data starting at the specified offset."""
        digital_output_bits = struct.unpack_from('!h', data, offset)[0]
        self.digital_outputs = [bool(digital_output_bits & (1 << n)) for n in range(16)]  # Assuming 16 outputs
        
    def extract_analog_inputs(self, data, offset=600):
        """Extract analog inputs from data starting at the specified offset."""
        self.analog_inputs = list(struct.unpack_from('!dd', data, offset))

    def extract_analog_outputs(self, data, offset=700):
        """Extract analog outputs from data starting at the specified offset."""
        self.analog_outputs = list(struct.unpack_from('!dd', data, offset))
        
    def monitor(self):
        """Run the monitoring process in a background thread."""
        if not self.is_running:
            self.is_running = True
            self.thread = threading.Thread(target=self.run_monitoring)
            self.thread.start()

    def run_monitoring(self):
        """Background task to continuously read data."""
        while self.is_running:
            self.read_data()

    def stop_monitoring(self):
        """Stop the monitoring thread."""
        self.is_running = False
        if self.thread:
            self.thread.join()  # Ensure the thread has finished executing
            self.thread = None
