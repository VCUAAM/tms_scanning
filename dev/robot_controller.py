import rtde_control, rtde_receive
import tkinter as tk
from tkinter import messagebox, simpledialog
import time
import socket
from math import *
import numpy as np

class RobotController:
    def __init__(self):
        # Robot parameters
        self.ip = "192.168.1.102"
        self.home_pose = [-0.005, -0.526, 0.222, 2.48, -0.919, 0.712] # x,y,z,rx,ry,rz figure of 8
        self.debugging = False
        self.xcoord = None
        self.ycoord = None

        # Normalization parameters
        self.rx = .25 # Initial angular change for each iteration in x
        self.ry = .25 # Initial angular change for each iteration in y
        self.err = .5 # Acceptable error in N

        while True:
            try:
                self.rtde_c = rtde_control.RTDEControlInterface(self.ip)
                self.rtde_r = rtde_receive.RTDEReceiveInterface(self.ip)
            except:
                root = tk.Tk()
                root.withdraw()
                messagebox.showerror("Error", "Turn remote control mode on and then press ok")
                root.destroy()
                continue
            break

        print("Connected. Press Ctrl+C to stop.")
    
    # Convert axis-angle vector (rx, ry, rz) to rotation matrix R
    def axis_angle_to_matrix(self,omega):
        theta = np.linalg.norm(omega)

        # Checking if there needs to be any rotation
        if theta < 1e-8:
            return np.eye(3)
        
        u = omega / theta
        ux, uy, uz = u

        # Cross-product matrix of u
        Ux = np.array([[   0, -uz,  uy],
                    [ uz,   0, -ux],
                    [-uy,  ux,   0]])
        
        # Outer product u u^T
        uuT = np.outer(u, u)
        R = cos(theta) * np.eye(3) + (1 - cos(theta)) * uuT + sin(theta) * Ux

        return R

    # Convert rotation matrix R to axis-angle vector (rx, ry, rz)
    def matrix_to_axis_angle(self,R):
        # Computing angle
        trace = np.trace(R)
        cos_theta = (trace - 1) / 2
        cos_theta = max(min(cos_theta, 1.0), -1.0)
        theta = acos(cos_theta)

        # Checking if there needs to be any rotation
        if abs(theta) < 1e-8:
            return np.array([0.0, 0.0, 0.0])

        sin_theta = sin(theta)

        # Axis from off-diagonal elements
        rx = (R[2,1] - R[1,2]) / (2 * sin_theta)
        ry = (R[0,2] - R[2,0]) / (2 * sin_theta)
        rz = (R[1,0] - R[0,1]) / (2 * sin_theta)
        u = np.array([rx, ry, rz])

        # Normalizing axis, then multiplying by angle
        u_norm = np.linalg.norm(u)
        if u_norm < 1e-8:
            u = np.array([1.0, 0.0, 0.0])
        else:
            u = u / u_norm

        return u * theta

    # Invert a pose (x, y, z, rx, ry, rz) to get the inverse pose
    def pose_inv(self,pose):
        x, y, z, rx, ry, rz = pose
        
        T = np.array([x, y, z])
        R = self.axis_angle_to_matrix(np.array([rx, ry, rz]))

        # Invert: R_inv = R.T, T_inv = - R.T * T
        R_inv = R.T
        T_inv = - R_inv.dot(T)

        # Convert back to axis-angle
        omega_inv = self.matrix_to_axis_angle(R_inv)

        return np.array([T_inv[0], T_inv[1], T_inv[2],
                            omega_inv[0], omega_inv[1], omega_inv[2]])
    
    # Ensuring that robot is available, otherwise it's hard to parse connection error
    def socket_check(self):
        while True:
            try:
                socket.create_connection((self.ip,30004), timeout=1)
            except OSError:
                root = tk.Tk()
                root.withdraw()
                messagebox.showerror("Error", "Make sure robot is on and computer is connected to VCU_UR5e network")
                continue
            break

    # For asynchronous moves, this is required to keep the robot busy, otherwise it will not work
    def steady_check(self,sleep_time=0.1):
        while not self.rtde_c.isSteady():
            time.sleep(sleep_time)

    # Used as a shorthand to extract pose in joint positions to save
    def print_current_pos(self):
        print([round(i,3) for i in self.get_tcp()])

    # Helper function to quickly get the current TCP position, with terminal output for debugging
    def get_tcp(self):
        return self.rtde_r.getActualTCPPose()
    
    # Executes a MoveJ command to the given position. abs_j dictates if the pose is in joint positions (true) or a cartesian pose (false), and will run IK if needed
    def moveJ(self, pose, ik=True):
        if ik:
            pose = self.rtde_c.getInverseKinematics(pose)
        self.rtde_c.moveJ(pose)
        self.steady_check()
    
    # Executes a MoveL command. abs_j dictates if the pose is in joint positions (true) or a cartesian pose (false), and will run IK if needed
    def moveL(self, pose,vel=0.001,ik=False,asynchronous=False):
        if ik:
            pose = self.rtde_c.getInverseKinematics(pose)
        self.rtde_c.moveL(pose,speed=vel,asynchronous=asynchronous)

        if not asynchronous:
            self.steady_check()
            self.rtde_c.stopL()
    
    # Function to find a pose offset from the current pose by a given offset vector
    def pose_offset(self, pos_offset,tcp_offset=[0,0,0]):
        frame_offset = self.rtde_c.getTCPOffset()[3:]
        tcp_offset = [frame_offset[i] + tcp_offset[i] for i in range(3)]
        
        offset = [*pos_offset,*tcp_offset]
        pose = self.rtde_c.poseTrans(self.get_tcp(), offset)

        return pose

    def move_home(self):
        self.moveJ(self.home_pose)

    # Function to move the robot in the tool frame at a given distance
    def tool_z(self,distance=10,vel=0.001,contact=False):
        self.moveL(self.pose_offset([0, 0, distance/1000]),vel=vel,asynchronous=contact)

        if contact:
            self.rtde_c.startContactDetection()

            while not self.rtde_c.readContactDetection():
                time.sleep(0.01)

            self.rtde_c.stopL()
            self.rtde_c.stopContactDetection()

    # Function to normalize the coil position to the phantom 
    def normalize_coil(self):
        rx,ry,err = self.rx,self.ry,self.err
        num_cycles=25

        i = 0
        xdir_i,ydir_i = 1,1
        xdir,ydir = xdir_i,ydir_i
        xgood,ygood = False, False
        
        while (not xgood or not ygood) and i < num_cycles:
            self.tool_z(distance=-2,vel=0.01)

            if i > 0:
                ang_offset = [radians(rx*xdir*1),radians(ry*ydir*1),0]
                new_pose = self.pose_offset([0,0,0],tcp_offset=ang_offset)
                self.moveJ(new_pose)

            self.tool_z(contact=True)
            self.steady_check()

            # Collect force with respect to tool frame
            force = self.rtde_r.getActualTCPForce()
            tcp = self.get_tcp()
            tcp_t = tcp
            tcp_t[3:] = [0,0,0]
            tcp_f = self.rtde_c.poseTrans(self.pose_inv(tcp),self.rtde_c.poseTrans(tcp_t,force))
            x_i = tcp_f[0]
            y_i = tcp_f[1]

            if self.debugging:
                print('Fx: %f, Fy: %f' %(round(float(x_i),3),round(float(y_i),3)))

            # Checking if the x angle needs to be adjusted
            if abs(x_i) > err and not xgood:
                xdir = abs(x_i)/x_i
                if xdir != xdir_i:
                    xdir_i = xdir
                    rx = rx/2
            elif not xgood:
                xgood = True
                rx = 0
            
            # Checking if the y angle needs to be adjusted
            if abs(y_i) > err and not ygood:
                ydir = abs(y_i)/y_i
                if ydir != ydir_i:
                    ydir_i = ydir
                    ry = ry/2
            elif not ygood:
                ygood = True
                ry = 0
            
            i += 1
        
        # Indicating outcome of normalization procedure 
        if self.debugging and i < num_cycles:
            print('Coil normalized')
        elif i >= num_cycles:
            print('Normalization failed')

    # Queries user for position
    def move_to_collection_point(self):
        self.tool_z(distance=-2,vel=0.05)
        self.move_home()

        if self.xcoord is None and self.ycoord is None:
            root = tk.Tk()
            root.withdraw()
            self.xcoord = simpledialog.askinteger("Input", "Enter X coordinate relative to this position (mm):")
            self.ycoord = simpledialog.askinteger("Input", "Enter Y coordinate relative to this position (mm):")
            root.destroy()

        self.moveL(self.pose_offset([self.xcoord / 1000, self.ycoord / 1000, 0]),vel=0.05)
        self.tool_z(contact=True)
    
    def shutdown(self):
        if self.rtde_c and self.rtde_c.isConnected():
            self.rtde_c.stopScript()
            self.rtde_c.disconnect()
        if self.rtde_r and self.rtde_r.isConnected():
            self.rtde_r.disconnect()

if __name__ == "__main__":
    rob = RobotController()
    rob.debugging = True

    try:
        rob.normalize_coil()
        #rob.find_robot_position()
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected.")
    finally:
        rob.rtde_c.stopL()
        rob.shutdown()
        print("Shutdown complete.")
