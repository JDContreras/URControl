# -*- coding: utf-8 -*-
"""
Created on Tue Sep  3 15:52:07 2024

@author: juandavc
"""

import numpy as np

def rpy_to_rotation_vector(roll, pitch, yaw):
    # Step 1: Convert RPY to rotation matrix
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])
    
    R = Rz @ Ry @ Rx

    # Step 2: Convert rotation matrix to rotation vector
    theta = np.arccos((np.trace(R) - 1) / 2)
    
    if theta == 0:
        return np.array([0, 0, 0])
    
    wx = (R[2, 1] - R[1, 2]) / (2 * np.sin(theta))
    wy = (R[0, 2] - R[2, 0]) / (2 * np.sin(theta))
    wz = (R[1, 0] - R[0, 1]) / (2 * np.sin(theta))
    
    omega = np.array([wx, wy, wz])
    
    rotation_vector = theta * omega

    return rotation_vector


def add_roll(pose, delta_roll):
    """
    Increment the roll angle of a pose array by a desired value.
    
    Args:
        pose (list or np.ndarray): The pose array in the format [x, y, z, rx, ry, rz].
        delta_roll (float): The incremental value in radians to add to the roll angle.
    
    Returns:
        np.ndarray: The new pose array with the incremented roll angle.
    """
    x, y, z, rx, ry, rz = pose
    
    # Convert rotation vector back to rotation matrix
    theta = np.linalg.norm([rx, ry, rz])
    
    if theta == 0:
        roll, pitch, yaw = 0, 0, 0
    else:
        omega = np.array([rx, ry, rz]) / theta
        R = np.eye(3) + np.sin(theta) * skew_symmetric(omega) + (1 - np.cos(theta)) * (skew_symmetric(omega) @ skew_symmetric(omega))
        
        # Extract roll, pitch, and yaw from the rotation matrix
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arcsin(-R[2, 0])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    
    # Increment roll by delta_roll
    new_roll = roll + delta_roll

    # Convert the new RPY back to a rotation matrix
    R_new = rpy_to_rotation_matrix(new_roll, pitch, yaw)
    
    # Convert the new rotation matrix back to a rotation vector
    rotation_vector_new = rotation_matrix_to_rotation_vector(R_new)
    
    # Return the new pose array with the incremented roll
    return np.array([x, y, z, *rotation_vector_new])

def skew_symmetric(v):
    """
    Generate a skew-symmetric matrix from a 3-element vector.
    
    Args:
        v (np.ndarray): A 3-element vector.
    
    Returns:
        np.ndarray: The skew-symmetric matrix.
    """
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])

def rpy_to_rotation_matrix(roll, pitch, yaw):
    """
    Convert roll, pitch, yaw to a rotation matrix.
    
    Args:
        roll (float): Roll angle in radians.
        pitch (float): Pitch angle in radians.
        yaw (float): Yaw angle in radians.
    
    Returns:
        np.ndarray: A 3x3 rotation matrix.
    """
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])
    
    return Rz @ Ry @ Rx

def rotation_matrix_to_rotation_vector(R):
    """
    Convert a rotation matrix to a rotation vector.
    
    Args:
        R (np.ndarray): A 3x3 rotation matrix.
    
    Returns:
        np.ndarray: A rotation vector.
    """
    theta = np.arccos((np.trace(R) - 1) / 2)
    
    if theta == 0:
        return np.array([0, 0, 0])
    
    wx = (R[2, 1] - R[1, 2]) / (2 * np.sin(theta))
    wy = (R[0, 2] - R[2, 0]) / (2 * np.sin(theta))
    wz = (R[1, 0] - R[0, 1]) / (2 * np.sin(theta))
    
    omega = np.array([wx, wy, wz])
    
    return theta * omega

def add_x(pose, delta_x):
    """
    Increment the x position of a pose array by a desired value.
    
    Args:
        pose (list or np.ndarray): The pose array in the format [x, y, z, rx, ry, rz].
        delta_x (float): The incremental value to add to the x position.
    
    Returns:
        np.ndarray: The new pose array with the incremented x position.
    """
    x, y, z, rx, ry, rz = pose
    
    # Increment x position
    new_x = x + delta_x
    
    # Return the new pose with incremented x
    return np.array([new_x, y, z, rx, ry, rz])

def add_z(pose, delta_z):
    """
    Increment the z position of a pose array by a desired value.
    
    Args:
        pose (list or np.ndarray): The pose array in the format [x, y, z, rx, ry, rz].
        delta_z (float): The incremental value to add to the z position.
    
    Returns:
        np.ndarray: The new pose array with the incremented z position.
    """
    x, y, z, rx, ry, rz = pose
    
    # Increment z position
    new_z = z + delta_z
    
    # Return the new pose with incremented z
    return np.array([x, y, new_z, rx, ry, rz])

def pose_mm2m(pose):
    """
    Convert a pose from millimeters to meters.
    
    Args:
        pose (list or np.ndarray): A pose in the format [x, y, z, rx, ry, rz] where
                                   x, y, z are in millimeters and rx, ry, rz are in radians.
    
    Returns:
        np.ndarray: A new pose in the format [x, y, z, rx, ry, rz] where
                    x, y, z are in meters.
    """
    # Convert the x, y, z from millimeters to meters by dividing by 1000
    x, y, z, rx, ry, rz = pose
    return np.array([x / 1000, y / 1000, z / 1000, rx, ry, rz])

def joints_deg2rad(joint_positions):
    """
    Convert an array of robot joint positions from degrees to radians.
    
    Args:
        joint_positions (list or np.ndarray): An array of joint positions in degrees.
    
    Returns:
        np.ndarray: An array of joint positions in radians.
    """
    # Use numpy's deg2rad function to convert from degrees to radians
    return np.deg2rad(joint_positions)


