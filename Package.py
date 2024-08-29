# -*- coding: utf-8 -*-
"""
Created on Thu Aug 29 15:35:44 2024

@author: juandavc
"""
import socket
import struct

I only want to map digitalInputBits, digitalOutputBits, analogInputs, analogOutputs as list, ignore all other. to make it more efficient we should check for data position directly (offset). Lest create a 

try:
    cartesian_info = {}
    robot_data = {
        'Cartesian Info': {},
        'Joint Data': [],
        'Robot Mode Data': {},
        'MASTERBOARD_DATA': {}
    }
    s = socket.create_connection(("169.254.22.200", 30002))
    data = s.recv(1254)
    offset = 0

    # Read message size and type
    messageSize = struct.unpack_from('!i', data, offset)[0]
    offset += 4
    messageType = struct.unpack_from('!B', data, offset)[0]
    offset += 1

    print(f"Message Size: {messageSize}, Message Type: {messageType}")

    if messageType == 16:  # ROBOT_STATE
        end_of_message = offset + messageSize - 5
        while offset < end_of_message:
            packageSize = struct.unpack_from('!i', data, offset)[0]
            packageType = struct.unpack_from('!B', data, offset + 4)[0]
            print(f"Package Type: {packageType}, Package Size: {packageSize}")

            next_offset = offset + packageSize  # Correct calculation for next package start
            offset += 5  # Move past the size and type fields

            if packageType == 4:  # CARTESIAN_INFO
                print("cartesian_info data ", offset)
                X, Y, Z, Rx, Ry, Rz = struct.unpack_from('!dddddd', data, offset)
                cartesian_info = {'X': X, 'Y': Y, 'Z': Z, 'Rx': Rx, 'Ry': Ry, 'Rz': Rz}
                #print("Cartesian info:", cartesian_info)
            
            elif packageType == 1:  # JOINT_DATA
                print("joint data ", offset)
                joint_data = []
                for _ in range(6):  # Adjusted for clarity
                    joint_info = struct.unpack_from('!dddffffB', data, offset)
                    joint_data.append(dict(zip(
                        ['q_actual', 'q_target', 'qd_actual', 'I_actual', 'V_actual', 'T_motor', 'T_micro', 'jointMode'],
                        joint_info
                    )))
                    offset += 41  # Adjust offset for each joint
                robot_data['Joint Data'] = joint_data
            elif packageType == 0:  # ROBOT_MODE_DATA
                print("ROBOT_MODE_DATA ", offset)
                timestamp, = struct.unpack_from('!Q', data, offset)
                offset += 8
                flags = struct.unpack_from('!???????', data, offset)
                offset += 7
                robotMode, = struct.unpack_from('!B', data, offset)
                offset += 1
                speedFraction, = struct.unpack_from('!d', data, offset)
                offset += 8
                robot_data['Robot Mode Data'] = {
                    'timestamp': timestamp,
                    'isRobotConnected': flags[0],
                    'isRealRobotEnabled': flags[1],
                    'isPowerOnRobot': flags[2],
                    'isEmergencyStopped': flags[3],
                    'isSecurityStopped': flags[4],
                    'isProgramRunning': flags[5],
                    'isProgramPaused': flags[6],
                    'robotMode': robotMode,
                    'speedFraction': speedFraction
                }
                #print("Robot Mode Data:", robot_data['Robot Mode Data'])
            
            elif packageType == 3:  # MASTERBOARD_DATA
                # Start unpacking masterboard data
                print("digitalInputBits ", offset)
                digitalInputBits, digitalOutputBits = struct.unpack_from('!hh', data, offset)
                offset += 4
                analogInputRange0, analogInputRange1 = struct.unpack_from('!BB', data, offset)
                offset += 2
                print("analogInput ", offset)
                analogInput0, analogInput1 = struct.unpack_from('!dd', data, offset)
                offset += 16
                analogOutputDomain0, analogOutputDomain1 = struct.unpack_from('!BB', data, offset)
                offset += 2
                print("analogOutput ", offset)
                analogOutput0, analogOutput1 = struct.unpack_from('!dd', data, offset)
                offset += 16
                masterBoardTemperature, robotVoltage48V, robotCurrent, masterIOCurrent = struct.unpack_from('!ffff', data, offset)
                offset += 16
                masterSaftyState, masterOnOffState, euromap67InterfaceInstalled = struct.unpack_from('!BBB', data, offset)
                offset += 3
            
                # Construct the masterboard data dictionary
                masterboard_data = {
                    'digitalInputBits': digitalInputBits,
                    'digitalOutputBits': digitalOutputBits,
                    'analogInputRange': (analogInputRange0, analogInputRange1),
                    'analogInput': (analogInput0, analogInput1),
                    'analogOutputDomain': (analogOutputDomain0, analogOutputDomain1),
                    'analogOutput': (analogOutput0, analogOutput1),
                    'masterBoardTemperature': masterBoardTemperature,
                    'robotVoltage48V': robotVoltage48V,
                    'robotCurrent': robotCurrent,
                    'masterIOCurrent': masterIOCurrent,
                    'masterSafetyState': masterSaftyState,
                    'masterOnOffState': masterOnOffState,
                    'euromap67InterfaceInstalled': bool(euromap67InterfaceInstalled)
                }
            
                # Check if euromap67 interface is installed
                if euromap67InterfaceInstalled == 1:
                    euromapInputBits, euromapOutputBits = struct.unpack_from('!ii', data, offset)
                    euromapVoltage, euromapCurrent = struct.unpack_from('!hh', data, offset + 8)
                    offset += 12  # Additional euromap data bytes
                    masterboard_data['euromapData'] = {
                        'euromapInputBits': euromapInputBits,
                        'euromapOutputBits': euromapOutputBits,
                        'euromapVoltage': euromapVoltage,
                        'euromapCurrent': euromapCurrent
                    }
            
                robot_data['MASTERBOARD_DATA'] = masterboard_data
            elif packageType == 7:  # FORCE_MODE_DATA
                # Unpack Force mode data
                X, Y, Z, Rx, Ry, Rz, robotDexterity = struct.unpack_from('!ddddddd', data, offset)
                force_mode_data = {
                    'X': X,
                    'Y': Y,
                    'Z': Z,
                    'Rx': Rx,
                    'Ry': Ry,
                    'Rz': Rz,
                    'robotDexterity': robotDexterity
                }
                robot_data['FORCE_MODE_DATA'] = force_mode_data
                #print("Force Mode Data:", force_mode_data)
                
                
            offset = next_offset  # Move to the next package correctly

finally:
    s.close()
