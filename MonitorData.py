# -*- coding: utf-8 -*-
"""
Created on Thu Aug 29 18:07:19 2024

@author: juandavc
"""

from typing import NamedTuple
import struct
import threading
import time
import socket

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

class RobotData:
    def __init__(self, ip):
        self.ip = ip
        self.port = 30002
        self.robot_mode_data = None
        self.cartesian_info = None
        self.joint_data = []
        self.digital_inputs = []
        self.digital_outputs = []
        self.analog_inputs = []
        self.analog_outputs = []
        self.is_running = False
        self.thread = None

    def read_data(self):
        """Read the latest data from the socket and update attributes."""
        time.sleep(0.1)
        try:
            s = socket.create_connection((self.ip, self.port))
            msg = s.recv(1254)  # Receive data from the socket
            msg_len = len(msg)
            messageSize, messageType = struct.unpack_from('!iB', msg, 0)
            print("messageSize size is ", messageSize)
            print("data size is ", len(msg))
            if messageType == 16 and msg_len == 1254 :  # Check if no error
                # Call extraction methods for each type of data
                self.extract_robot_mode_data(msg)
                self.extract_cartesian_info(msg)
                self.extract_joint_data(msg)
                #self.extract_digital_inputs(data)
                self.extract_digital_outputs(msg)
                self.extract_analog_inputs(msg)
                self.extract_analog_outputs(msg)

        except Exception as e:
            print(f"Error reading data: {e}")
        finally:
            s.close()

    def extract_robot_mode_data(self, data, offset=10):
        # Assuming 'data' is a bytes object containing the message received from the robot
        # The format string needs to be adjusted based on actual data structure:
        # Each '?' in the format string represents a boolean value.
        data_format = '!7?'  # Update this if there are different types or more/less data
        self.robot_mode_data = RobotModeData._make(struct.unpack_from(data_format, data, offset))
    
    def extract_cartesian_info(self, data, offset=290):
        """Extract and update cartesian info from the given data starting at the specified offset."""
        data_format = '!dddddd'  # Format for 6 double precision floats.
        # Unpack the data directly into a new NamedTuple instance and assign it to the class attribute.
        self.cartesian_info = CartesianInfo._make(struct.unpack_from(data_format, data, offset))
        
    def extract_joint_data(self, data, offset=39):
        """Extract and update joint data from the given data starting at the specified offset."""
        self.joint_data.clear()  # Clear existing joint data
        for _ in range(6):  # Assuming there are 6 joints
            joint_info = struct.unpack_from('!dddffffB', data, offset)
            self.joint_data.append(JointData._make(joint_info))
            offset += 41  # Move to the next joint's data    
 
    def extract_digital_inputs(self, data, offset=621):
        """Extract digital inputs from data starting at the specified offset."""
        digital_input_bits = struct.unpack_from('!h', data, offset)[0]
        self.digital_inputs = [bool(digital_input_bits & (1 << n)) for n in range(16)]  # Assuming 16 inputs

    def extract_digital_outputs(self, data, offset=623):
        """Extract digital outputs from data starting at the specified offset."""
        digital_output_bits = struct.unpack_from('!h', data, offset)[0]
        self.digital_outputs = [bool(digital_output_bits & (1 << n)) for n in range(16)]  # Assuming 16 outputs
        
    def extract_analog_inputs(self, data, offset=627):
        """Extract analog inputs from data starting at the specified offset."""
        self.analog_inputs = list(struct.unpack_from('!dd', data, offset))

    def extract_analog_outputs(self, data, offset=645):
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
