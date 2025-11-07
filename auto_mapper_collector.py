import nidaqmx
from nidaqmx.constants import READ_ALL_AVAILABLE, AcquisitionType
import rtde_control, rtde_receive
import time
import dev.helper_functions as hf
import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
from math import *
from tkinter import messagebox,simpledialog

# Name of the output file
output_file = '100325_qbc_50.txt'

# Home positition of the robot (THIS IS YOUR 0,0 COORDINATE)
home_position = [-0.014, 0.561, 0.236, -2.668, 0.408, -0.592] # x,y,z,rx,ry,rz (BASE FRAME, NOT VIEW)

hf.socket_check()

while True:
    try:
        rtde_c = rtde_control.RTDEControlInterface("192.168.1.102")
        rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.102")
    except:
        root = tk.Tk()
        root.withdraw()
        messagebox.showerror("Error", "Turn remote control mode on and then press ok")
        continue
    break

home_pos_j = rtde_c.getInverseKinematics(home_position) # j1,j2,j3,j4,j5,j6

# DAQ Parameters
sample_rate = 90000 #Hz
num_samples = 300#35 # number of samples per channel
pre_wave_cutoff = 0#7 #number of samples removed from beginning
amplification_factor = 11

# Function to collect magnetic field data from the DAQ
def read_magnetic_field():
    while True:
        while True:
            try:
                with nidaqmx.Task() as ai_task, nidaqmx.Task() as ao_task:

                    # Add analog input channels for three axes
                    ai_task.ai_channels.add_ai_voltage_chan("Dev1/ai0")  # x
                    ai_task.ai_channels.add_ai_voltage_chan("Dev1/ai1")  # y
                    ai_task.ai_channels.add_ai_voltage_chan("Dev1/ai2") #z

                    # Configure timing and trigger from the stimulator
                    ai_task.timing.cfg_samp_clk_timing(sample_rate, sample_mode=AcquisitionType.FINITE, samps_per_chan=num_samples)
                    ai_task.triggers.start_trigger.cfg_dig_edge_start_trig("/Dev1/PFI0")

                    # Add analog output channel to trigger the stimulator
                    ao_task.ao_channels.add_ao_voltage_chan("Dev1/ao0")

                    # Initiate data acquisition
                    ai_task.start()
                    
                    # Clear any previous triggers and send trigger pulses to the stimulator
                    ao_task.write(0)
                    ao_task.write(5)
                    ao_task.write(0)

                    # Read the acquired data from all channels
                    data = ai_task.read(READ_ALL_AVAILABLE)

                    # Stop the tasks
                    ai_task.stop()
                    ao_task.stop()
            except:
                root = tk.Tk()
                root.withdraw()
                messagebox.showerror("Error", "Press OK and then turn the stimulator on")
                continue
            break
        # Processing acquired data for each task. This involves removing initial samples, filtering noisy data collected, and converting to E-field from voltage readings
        ai0 = data[0][pre_wave_cutoff:]
        ai0 = [float(i)/amplification_factor/0.003 for i in ai0]
        mag_x = max([abs(max(hf.noise_filter(ai0))),abs(min(hf.noise_filter(ai0)))])
        ai1 = data[1][pre_wave_cutoff:]
        ai1 = [float(i)/amplification_factor/0.003 for i in ai1]
        mag_y = max([abs(max(hf.noise_filter(ai1))),abs(min(hf.noise_filter(ai1)))])
        ai2 = data[2][pre_wave_cutoff:]
        ai2 = [float(i)/amplification_factor/0.003 for i in ai2]
        mag_z = max([abs(max(hf.noise_filter(ai2))),abs(min(hf.noise_filter(ai2)))])
        mag_total = sqrt(mag_x**2 + mag_y**2 + mag_z**2)

        dt = 1 / sample_rate
        time_axis = np.linspace(0, (num_samples - pre_wave_cutoff - 1) * dt, num_samples - pre_wave_cutoff)

        # Plotting the data
        plt.figure(figsize=(10, 5))
        plt.plot(time_axis, ai0, label="AI0")   
        plt.plot(time_axis, ai1, label="AI1")
        #plt.plot(time_axis, ai2, label="AI2")
        plt.xlabel("Time (s)")
        plt.ylabel("E-Field Magnitude Range (V/m)")
        plt.title("NI-USB 6363 DAQ Data")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show(block=False)

        # Ask user if the pulse is satisfactory
        test = hf.ask_yes_no_popup("Is this a good TMS pulse?")

        if test == True:
            plt.close()
            return [mag_x, mag_y, mag_z, mag_total]
        else:
            plt.close()
            continue

# Function to move the robot in a straight line within tool frame
def find_pose(distance,inv=False):
    pose_wrt = rtde_c.getTCPOffset()
    move = [distance[0],distance[1],distance[2],pose_wrt[3],pose_wrt[4],pose_wrt[5]] 
    if inv:
        return rtde_c.getInverseKinematics(rtde_c.poseTrans(rtde_r.getActualTCPPose(),move))
    return rtde_c.poseTrans(rtde_r.getActualTCPPose(),move)

# Move the robot to the specified coordinate using RTDE communication with the UR5e
def find_robot_position():
    
    # Move robot to 0,0 coordinate
    rtde_c.moveJ(home_pos_j)
    root = tk.Tk()
    root.withdraw()

    xcoord = simpledialog.askinteger("Input", "Enter X coordinate relative to this position (in mm):")
    ycoord = simpledialog.askinteger("Input", "Enter Y coordinate relative to this position (in mm):")
    root.destroy()
    # Move robot to specified coordinate
    rtde_c.moveJ(find_pose([xcoord/1000, ycoord/1000, 0],inv=True))
    while not rtde_c.isSteady():
        time.sleep(0.1)
    #normalize_coil()
    position = rtde_r.getActualTCPPose()

    return [xcoord, ycoord, position[0], position[1], position[2], position[3], position[4], position[5]]

# Function to make the coil normal to the surface
def normalize_coil():
    # Coil normalizing parameters
    rx = .25 # Initial angular change for each iteration in x
    ry = .25 # Initial angular change for each iteration in y
    err = .5 # Acceptable error in N
    num_cycles = 50 # Max number of cycles before raising concern to operator
    i = 0
    contact_distance = 100 # Distance to move down to check for contact (in mm)
    xdir_i,ydir_i = 1,1
    xdir,ydir = xdir_i,ydir_i
    xgood,ygood = False, False

    rtde_c.moveL(find_pose([0,0,contact_distance/1000]),speed=0.015,asynchronous=True)
    rtde_c.startContactDetection()

    while not rtde_c.readContactDetection():
        time.sleep(.01)
    rtde_c.stopContactDetection()

    while not xgood or not ygood:
        rtde_c.moveL(find_pose([0,0,-0.001]),speed=0.002)
        if i > 0:
            pose_wrt = rtde_c.getTCPOffset()
            move = [0, 0, 0, pose_wrt[3] + radians(rx*xdir*1),pose_wrt[4] + radians(ry*ydir*1),pose_wrt[5]] 
            rtde_c.moveJ(rtde_c.getInverseKinematics(rtde_c.poseTrans(rtde_r.getActualTCPPose(),move)))
        # Move robot to surface to check for contact forces
        rtde_c.moveL(find_pose([0,0,contact_distance/1000]),speed=0.0005,asynchronous=True)
        rtde_c.startContactDetection()

        while not rtde_c.readContactDetection():
            time.sleep(.001)
        rtde_c.stopContactDetection()

        # Collect force wrt tool frame
        force = rtde_r.getActualTCPForce()
        tcp = rtde_r.getActualTCPPose()
        tcp_t = tcp
        tcp_t[3:] = [0,0,0]
        tcp_f = rtde_c.poseTrans(hf.pose_inv(tcp),rtde_c.poseTrans(tcp_t,force))
        x_i = tcp_f[0]
        y_i = tcp_f[1]
        print('Fx: %f, Fy: %f' %(round(float(x_i),3),round(float(y_i),3)))
    
        if abs(x_i) > err and not xgood:
            xdir = abs(x_i)/x_i
            if xdir != xdir_i:
                xdir_i = xdir
                rx = rx/2
        elif not xgood:
            xgood = True
            rx = 0
            
        if abs(y_i) > err and not ygood:
            ydir = abs(y_i)/y_i
            if ydir != ydir_i:
                ydir_i = ydir
                ry = ry/2
        elif not ygood:
            ygood = True
            ry = 0
        
        i += 1
    rtde_c.stopL()
    print("Coil in position")
    
if __name__ == "__main__":
        try:
            print("Connected. Press Ctrl+C to stop.")
            while True:
                # Move robot to the specified position
                pos = find_robot_position()
                mag = read_magnetic_field()
                hf.save_to_file(mag,pos,output_file)
                print(f"Data saved to {output_file}")
                hf.plot_map(output_file)
                test = hf.ask_yes_no_popup("Keep collecting points?")
                plt.close()
                if test == True:
                    continue
                break   

        except KeyboardInterrupt:
            print("KeyboardInterrupt detected. Disconnecting...")
            
        finally:
            # Ensure the robot is stopped and the connection is closed
            print("Shutting down RTDE connection.")
            if rtde_c.isConnected():
                rtde_c.stopScript()
                rtde_c.disconnect()
            if rtde_r.isConnected():
                rtde_r.disconnect()
            print("Connection closed.")
