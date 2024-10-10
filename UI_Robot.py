# -*- coding: utf-8 -*-
"""
Created on Mon Oct  7 17:42:49 2024

@author: juandavc
"""

import tkinter as tk
from URControl import UR5CB2Control, UR5CommandBuilder
import numpy as np
from transformations import add_roll, add_x, add_z, pose_mm2m, joints_deg2rad
from dataclasses import dataclass

@dataclass
class ToolPos:
    x: float
    y: float
    z: float
    rx: float  # Rotation around the X-axis
    ry: float  # Rotation around the Y-axis
    rz: float  # Rotation around the Z-axis
    
    def pose(self):
        return [self.x / 1000, self.y / 1000, self.z / 1000, self.rx, self.ry, self.rz]

@dataclass
class RobotConf:
    base: float
    shoulder: float
    elbow: float
    w1: float  # Wrist 1
    w2: float  # Wrist 2
    w3: float  # Wrist 3
    
    def jointPos(self):
        return joints_deg2rad([self.base, self.shoulder, self.elbow, self.w1, self.w2, self.w3])

class RobotControlApp:
    def __init__(self, master):
        self.master = master
        self.master.title("UR5 Robot Control")
        
        # Set window size
        self.master.geometry("400x800")
        
        # Create a main frame to hold left and right sections
        main_frame = tk.Frame(self.master)
        main_frame.pack(fill="both", expand=True)
        
        # Create a left-side frame for the new button
        left_frame = tk.Frame(main_frame)
        left_frame.pack(side="left", padx=10, pady=10)
        
        # Create a right-side frame for the existing buttons
        right_frame = tk.Frame(main_frame)
        right_frame.pack(side="right", padx=10, pady=10)
        
        # UI Elements with fixed size and border
        button_style = {
            "width": 20,       # Fixed width
            "height": 3,       # Fixed height
            "borderwidth": 4,  # Thicker border
            "padx": 5,         # Padding
            "pady": 5,         # Padding
        }
        
        self.connect_button = tk.Button(left_frame, text="Connect to Robot", command=self.connect_to_robot, **button_style)
        self.connect_button.pack(pady=10)
        
        self.pick_button = tk.Button(left_frame, text="Pick", command=self.pick, state="disabled", **button_style)
        self.pick_button.pack(pady=10)
        
        self.place1 = tk.Button(left_frame, text="Place Pos 1", command=self.place1, state="disabled", **button_style)
        self.place1.pack(pady=10)
        
        self.place2 = tk.Button(left_frame, text="Place Pos 2", command=self.place2, state="disabled", **button_style)
        self.place2.pack(pady=10)
        
        self.move_home_button = tk.Button(left_frame, text="Move to Home", command=self.move_to_home, state="disabled", **button_style)
        self.move_home_button.pack(pady=10)
        
        self.retract = tk.Button(right_frame, text="Retract Fingers", command=self.retract_fingers, state="disabled", **button_style)
        self.retract.pack(pady=10)
        
        self.extent = tk.Button(right_frame, text="Extent Fingers", command=self.extent_fingers, state="disabled", **button_style)
        self.extent.pack(pady=10)
        
        self.vaccum_on = tk.Button(right_frame, text="Activate Vaccum", command=self.activate_vaccum, state="disabled", **button_style)
        self.vaccum_on.pack(pady=10)
        
        self.vaccum_off = tk.Button(right_frame, text="Deactivate Vaccum", command=self.deactivate_vaccum, state="disabled", **button_style)
        self.vaccum_off.pack(pady=10)
        
        self.status_label = tk.Label(right_frame, text="Not connected", borderwidth=3, relief="solid")
        self.status_label.pack(pady=10)

        # Handle window close event
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        #Robot parameters
        self.finger_output = 0
        self.vaccum_output = 0
        self.cylinder_output = 1 
        
        #  -----  Positions ------#
        
        #pick positions
        self.idle_conf = RobotConf(base=87.723, 
                                   shoulder=0.838, 
                                   elbow=-121.037, 
                                   w1=-61.055, 
                                   w2=95.594, 
                                   w3=92.147)

        self.pick_approach_conf = RobotConf(base=157.225, 
                                            shoulder=8.123, 
                                            elbow=-107.994, 
                                            w1=-82.965, 
                                            w2=26.121, 
                                            w3=94.817)
        
        self.prepick_conf = RobotConf(base = 170.791, 
                                       shoulder = -0.964, 
                                       elbow = -36.411, 
                                       w1 = -149.035, 
                                       w2 = 11.254, 
                                       w3 = 98.558) 
        
        self.pick_pos = ToolPos(x = 878.402, 
                                y = 158.739, 
                                z = 418.974, 
                                rx = -1.230, 
                                ry = 1.240, 
                                rz = 1.251) #w3
        

        self.lift_pos = ToolPos(x = self.pick_pos.x - 50.000, 
                                y = self.pick_pos.y - 45.000, 
                                z = self.pick_pos.z, 
                                rx = -1.395, 
                                ry = 1.127, 
                                rz = 1.408) #w4
        
        self.rot_pos = ToolPos(x = self.lift_pos.x, 
                                y = self.lift_pos.y, 
                                z = self.lift_pos.z, 
                                rx = self.pick_pos.rx, 
                                ry = self.pick_pos.ry, 
                                rz = self.pick_pos.rz) #w5
        
        self.takeoff_pos = ToolPos(x = self.rot_pos.x - 320.000, #config for box size
                                y = self.rot_pos.y, 
                                z = self.rot_pos.z, 
                                rx = self.rot_pos.rx, 
                                ry = self.rot_pos.ry, 
                                rz = self.rot_pos.rz) #w6
        
        self.halfway_conf = RobotConf(base = 155.100, 
                                      shoulder = 20.030, 
                                      elbow = -115.684, 
                                      w1 = -87.118, 
                                      w2 = 26.770, 
                                      w3 = 6.241)
        
        self.pallet_edge_conf = RobotConf(base = 155.100, 
                                          shoulder = 20.030, 
                                          elbow = -115.684, 
                                          w1 = -87.118, 
                                          w2 = 26.770, 
                                          w3 = 6.241)
        
        
        #place 1 positions
        
        self.placement_init_conf = RobotConf(base = 155.100, 
                                             shoulder = -0.249, 
                                             elbow = -99.193, 
                                             w1 = -83.330, 
                                             w2 = 26.770, 
                                             w3 = 6.241)
        
        self.lower_plane_conf = RobotConf(base = 180.432, 
                                          shoulder = 0.542, 
                                          elbow = -96.002, 
                                          w1 = -85.326, 
                                          w2 = 4.224, 
                                          w3 = 2.536)
        
        self.approach_drop1_conf = RobotConf(base = 177.749, 
                                             shoulder = -82.140, 
                                             elbow = -92.756, 
                                             w1 = -14.895, 
                                             w2 = 3.129, 
                                             w3 = 10.854)
        
        self.predrop1_pos = ToolPos(x = -345.000, 
                                    y = 535.000, 
                                    z = 870.000, 
                                    rx = -1.575, 
                                    ry = 0.000, 
                                    rz = 0.000)
        
        self.release1_pos = ToolPos(x = self.predrop1_pos.x, 
                                    y = self.predrop1_pos.y, 
                                    z = self.predrop1_pos.z, 
                                    rx = -1.780,
                                    ry = self.predrop1_pos.ry, 
                                    rz = self.predrop1_pos.rz)
        
        self.drop1_pos = ToolPos(x = self.predrop1_pos.x, 
                                 y = self.predrop1_pos.y + 25.000, 
                                 z = self.predrop1_pos.z - 5.000, 
                                 rx = -1.682, 
                                 ry = self.predrop1_pos.ry, 
                                 rz = self.predrop1_pos.rz)
        
        self.postdrop1_pos = ToolPos(x = self.predrop1_pos.x, 
                                     y = self.drop1_pos.y - 190.000, 
                                     z = self.drop1_pos.z - 50.000, 
                                     rx = -1.59, 
                                     ry = self.predrop1_pos.ry, 
                                     rz = self.predrop1_pos.rz)
        
        #place 2 positions
        
        self.approach_drop2_conf = RobotConf(base = 138.312, 
                                             shoulder = -42.052, 
                                             elbow = -111.872, 
                                             w1 = -23.859, 
                                             w2 = 42.522, 
                                             w3 = 1.655)
        
        self.predrop2_pos = ToolPos(x = self.predrop1_pos.x + 320.000, 
                                    y = self.predrop1_pos.y, 
                                    z = self.predrop1_pos.z, 
                                    rx = self.predrop1_pos.rx, 
                                    ry = self.predrop1_pos.ry, 
                                    rz = self.predrop1_pos.rz)
        
        
        self.release2_pos = ToolPos(x = self.predrop2_pos.x, 
                                    y = self.predrop2_pos.y, 
                                    z = self.predrop2_pos.z, 
                                    rx = -1.780,
                                    ry = self.predrop2_pos.ry, 
                                    rz = self.predrop2_pos.rz)
        
        self.drop2_pos = ToolPos(x = self.predrop2_pos.x, 
                                 y = self.predrop2_pos.y + 25.000, 
                                 z = self.predrop2_pos.z - 5.000, 
                                 rx = -1.682, 
                                 ry = self.predrop2_pos.ry, 
                                 rz = self.predrop2_pos.rz)
        
        self.postdrop2_pos = ToolPos(x = self.predrop2_pos.x, 
                                     y = self.predrop2_pos.y - 190.000, 
                                     z = self.predrop2_pos.z - 50.000, 
                                     rx = -1.59, 
                                     ry = self.predrop2_pos.ry, 
                                     rz = self.predrop2_pos.rz)
        
        
        self.l_velocity = 0.2
        self.l_acceleration = 0.5
        self.j_velocity = 0.2
        self.j_acceleration = 0.5
        
        # Variables
        self.robot = None
        self.connected = False


    def connect_to_robot(self):
        """Connect to the UR5 robot."""
        if not self.connected:
            try:
                # Attempt to connect to the robot
                self.robot = UR5CB2Control("192.168.1.109")  # Replace with your robot's IP
                
                # Check if the socket is not None to confirm a successful connection
                if self.robot.socket is None:
                    raise ConnectionError("Failed to establish a connection to the robot.")
                    
                # If the connection is successful, update the status label and enable buttons
                self.status_label.config(text="Connected to Robot")
                self.connected = True
                # Enable action buttons
                self.pick_button.config(state="normal")
                self.place1.config(state="normal")
                self.place2.config(state="normal")
                self.move_home_button.config(state="normal")
                self.retract.config(state="normal")
                self.extent.config(state="normal")
                self.vaccum_on.config(state="normal")
                self.vaccum_off.config(state="normal")
                
            except Exception as e:
                # Handle the exception and display an error message
                self.status_label.config(text=f"Failed to connect to the robot: {e}")
                self.connected = False  # Ensure that 'connected' stays False
        else:
            self.status_label.config(text="Already connected")


    def on_closing(self):
        """Handle window close and disconnect the robot if connected."""
        if self.robot:
            self.robot.close()
            self.status_label.config(text="Robot disconnected")
        self.master.destroy()

    # Functions for each robot action
    def pick(self):
        """Define the robot's pick action for moving a box from a pallet."""
        msg = UR5CommandBuilder()
        
    
        # Kinematics
        vel = self.l_velocity*0.5
        accel = self.l_acceleration
        jvel = self.j_velocity
        jaccel = self.j_acceleration
        r = 0.05 #radius
        
        # outputs
        finger_pos = 0.6
        # 1. Move to the approach position near the box (joint movement)
        
        msg.movej2(q = self.idle_conf.jointPos(), v = jvel, a = jaccel, r = r)
        msg.movej2(q = self.pick_approach_conf.jointPos(), v = jvel, a = jaccel, r = r)
        msg.movej2(q = self.prepick_conf.jointPos(), v = jvel, a = jaccel, r = 0.0)
        
        # 2. Activate vacuum gripper to prepare for picking up the box
        msg.set_digital_output(self.vaccum_output, True)
        msg.sleep(1)
        
        # 3. Move to the box position with linear motion
        msg.movel(pose = self.pick_pos.pose(), v = vel, a = accel)
        msg.sleep(0.5)
        
        # 4. Engage vacuum and gently lift the box
        msg.movel(pose = self.lift_pos.pose(), v = vel, a = accel)
        
        # 5. Insert fingers for additional support
        msg.set_analog_output(output_number = self.finger_output, value = finger_pos)
        msg.sleep(2)
        msg.movel(pose = self.rot_pos.pose(), v = vel, a = accel)
        
        # 6. Move out of the pallet area with the box (linear move)
        msg.movel(pose = self.takeoff_pos.pose(), v = vel, a = accel)
        
        # 7. move to next function position
        msg.movej2(q = self.halfway_conf.jointPos(), v = jvel, a = jaccel, r = r)
        msg.movej2(q = self.pallet_edge_conf.jointPos(), v = jvel, a = jaccel, r = 0.0)
        
        self.robot.send_command(msg.get_command())
    
        self.status_label.config(text="Pick action sent")
        
        msg.clear_command()

    def place1(self):
        
        msg = UR5CommandBuilder()
               
        # Kinematics
        vel = self.l_velocity*0.5
        accel = self.l_acceleration
        jvel = self.j_velocity*0.5
        jaccel = self.j_acceleration
        r = 0.05 #radius
        
        # 8. Rotate the end effector to the drop-off orientation
        msg.movej2(q = self.placement_init_conf.jointPos(), v = jvel, a = jaccel, r = r)
        msg.movej2(q = self.lower_plane_conf.jointPos(), v = jvel, a = jaccel, r = r)
        msg.movej2(q = self.approach_drop1_conf.jointPos(), v = jvel, a = jaccel, r = 0.0)
        
        # 9. Move to the release one edge - position (linear move)
        msg.movel(pose = self.predrop1_pos.pose(), v = vel, a = accel)
        msg.movel(pose = self.release1_pos.pose(), v = vel, a = accel)
        
        # 10. Retract fingers 
        msg.set_analog_output(output_number = self.finger_output, value = 0.0)
        msg.sleep(2)
        
        # 11. Rotate and release the box completely at the drop-off point
        msg.movel(pose = self.drop1_pos.pose(), v = vel, a = accel)
        
        # 12. deactivate vaccum 
        msg.set_digital_output(self.vaccum_output, False)
        msg.sleep(1)
        
        # 13. move to Idle position
        msg.movel(pose = self.postdrop1_pos.pose(), v = vel, a = accel)
        msg.movej2(q = self.lower_plane_conf.jointPos(), v = jvel, a = jaccel, r = r)
        msg.movej2(q = self.placement_init_conf.jointPos(), v = jvel, a = jaccel, r = r)
        msg.movej2(q = self.idle_conf.jointPos(), v = jvel, a = jaccel, r = r)

        self.robot.send_command(msg.get_command())
        self.status_label.config(text="Place action sent")

    def place2(self):
        
        msg = UR5CommandBuilder()
               
        # Kinematics
        vel = self.l_velocity*0.5
        accel = self.l_acceleration
        jvel = self.j_velocity*0.5
        jaccel = self.j_acceleration
        r = 0.05 #radius
        
        # 8. Rotate the end effector to the drop-off orientation
        msg.movej2(q = self.placement_init_conf.jointPos(), v = jvel, a = jaccel, r = r)
        msg.movej2(q = self.lower_plane_conf.jointPos(), v = jvel, a = jaccel, r = r)
        msg.movej2(q = self.approach_drop2_conf.jointPos(), v = jvel, a = jaccel, r = 0.0)
        
        # 9. Move to the release one edge - position (linear move)
        msg.movel(pose = self.predrop2_pos.pose(), v = vel, a = accel)
        msg.movel(pose = self.release2_pos.pose(), v = vel, a = accel)
        
        # 10. Retract fingers 
        msg.set_analog_output(output_number = self.finger_output, value = 0.0)
        msg.sleep(2)
        
        # 11. Rotate and release the box completely at the drop-off point
        msg.movel(pose = self.drop2_pos.pose(), v = vel, a = accel)
        
        # 12. deactivate vaccum 
        msg.set_digital_output(self.vaccum_output, False)
        msg.sleep(1)
        
        # 13. move to Idle position
        msg.movel(pose = self.postdrop2_pos.pose(), v = vel, a = accel)
        msg.movej2(q = self.lower_plane_conf.jointPos(), v = jvel, a = jaccel, r = r)
        msg.movej2(q = self.placement_init_conf.jointPos(), v = jvel, a = jaccel, r = r)
        msg.movej2(q = self.idle_conf.jointPos(), v = jvel, a = jaccel, r = 0.0)

        self.robot.send_command(msg.get_command())
        self.status_label.config(text="Place action sent")
        
    def move_to_home(self):

        msg = UR5CommandBuilder()
        
        msg.movej2(q = self.idle_conf.jointPos(), v = self.j_velocity*0.5, a = self.j_acceleration, r = 0.0)
        
        self.robot.send_command(msg.get_command())
        
        self.status_label.config(text="Moving to Home")
        
    def activate_vaccum(self):

        msg = UR5CommandBuilder()
        
        msg.set_digital_output(output_number = self.vaccum_output, state = True)
        
        self.robot.send_command(msg.get_command())
        
        self.status_label.config(text="Vaccum On")
        
    def deactivate_vaccum(self):

        msg = UR5CommandBuilder()
        
        msg.set_digital_output(output_number=self.vaccum_output, state=False)
        
        self.robot.send_command(msg.get_command())
        
        self.status_label.config(text="Vaccum Off")
        
    def extent_fingers(self):

        msg = UR5CommandBuilder()
        
        msg.set_analog_output(output_number=self.finger_output, value=0.99)
        
        self.robot.send_command(msg.get_command())
        
        self.status_label.config(text="Finger out")
        
    def retract_fingers(self):

        msg = UR5CommandBuilder()
        
        msg.set_analog_output(output_number=self.finger_output, value=0.01)
        
        self.robot.send_command(msg.get_command())
        
        self.status_label.config(text="Finger in")


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotControlApp(root)
    root.mainloop()
