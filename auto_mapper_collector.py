import nidaqmx
import numpy as np
import matplotlib.pyplot as plt
import rtde_control
import rtde_receive
import tkinter as tk
from tkinter import messagebox, simpledialog
from nidaqmx.constants import READ_ALL_AVAILABLE, AcquisitionType

# Name of the output file
output_file = 'test_data.txt'

# Initialize RTDE interfaces
rtde_c = rtde_control.RTDEControlInterface("192.168.1.102")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.102")

# Home positition of the robot (THIS IS YOUR 0,0 COORDINATE)
home_position = [.225,-.139,.286,2.286,-2.184,0.072] # x,y,z,rx,ry,rz (BASE FRAME, NOT VIEW)
home_pos_j = rtde_c.getInverseKinematics(home_position) # j1,j2,j3,j4,j5,j6

# DAQ Parameters
sample_rate = 90000 #Hz
num_samples = 35 # number of samples per channel
pre_wave_cutoff = 7 #number of samples removed from beginning
amplification_factor = 11

# Function to check and filter noise from the data
def noise_filter(data):
    #Calculate mean of absolute value of each value in data
    mean = np.mean(np.abs(data))
    if mean < 0.05:
        return [0 for i in data]
    else:
        return data

# Function to check with user if graph is satisfactory
def ask_yes_no_popup():
    root = tk.Tk()
    root.withdraw()

    response = messagebox.askyesno("Confirmation", "Is this a good TMS pulse?")

    if response:
        root.destroy()
        return True
    else:
        root.destroy()
        return False

# Function to collect magnetic field data from the DAQ
def read_magnetic_field():
    while True:
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
        
        # Processing acquired data for each task. This involves removing initial samples, filtering noisy data collected, and converting to E-field from voltage readings
        ai0 = data[0][pre_wave_cutoff:]
        mag_x = max(noise_filter(ai0)) - min(noise_filter(ai0))
        ai0 = [float(i)/amplification_factor/0.003 for i in ai0]
        ai1 = data[1][pre_wave_cutoff:]
        mag_y = max(noise_filter(ai1)) - min(noise_filter(ai1))
        ai1 = [float(i)/amplification_factor/0.003 for i in ai1]
        ai2 = data[2][pre_wave_cutoff:]
        mag_z = max(noise_filter(ai2)) - min(noise_filter(ai2))
        ai2 = [float(i)/amplification_factor/0.003 for i in ai2]
        mag_total = (mag_x**2 + mag_y**2 + mag_z**2)**0.5

        dt = 1 / sample_rate
        time_axis = np.linspace(0, (num_samples - pre_wave_cutoff - 1) * dt, num_samples - pre_wave_cutoff)

        # Plotting the data
        plt.figure(figsize=(10, 5))
        plt.plot(time_axis, ai0, label="AI0")   
        plt.plot(time_axis, ai1, label="AI1")
        plt.plot(time_axis, ai2, label="AI2")
        plt.xlabel("Time (s)")
        plt.ylabel("E-Field Magnitude Range (V/m)")
        plt.title("NI-USB 6363 DAQ Data")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

        # Ask user if the pulse is satisfactory
        test = ask_yes_no_popup()

        if test == True:
            return [mag_x, mag_y, mag_z, mag_total]
        else:
            continue

# Move the robot to the specified coordinate using RTDE communication with the UR5e
def move_robot():
    # Move robot to 0,0 coordinate
    rtde_c.moveJ(home_pos_j)
    root = tk.Tk()
    root.withdraw()

    xcoord = simpledialog.askfloat("Input", "Enter X coordinate relative to this position:")
    ycoord = simpledialog.askfloat("Input", "Enter Y coordinate relative to this position:")
    
    # Move robot to specified coordinate
    target_position = [xcoord + home_position[0], ycoord + home_position[1], home_position[2], home_position[3], home_position[4], home_position[5]]
    target_tcp = rtde_c.getInverseKinematics(target_position)
    rtde_c.moveJ(target_tcp)

    ########
    # Here I need to either add in a call to the script on the robot or rewrite it within RTDE framework to automatically orient end effector to be normal to the head surface
    #####
    position = rtde_r.getActualTCPPose()

    return [xcoord, ycoord, position[0], position[1], position[2], position[3], position[4], position[5]]


def save_to_file(mag,pos):
    header = 'Mag (V/m)\tx_rel (mm)\ty_rel (mm)\tx_field (V/m)\ty_field (V/m)\tz_field (V/m)\tx (m)\ty (m)\tz (m)\trx (rad)\try (rad)\trz (rad)'
    output = [mag[3], pos[0], pos[1], mag[0], mag[1], mag[2], pos[2], pos[3], pos[4], pos[5], pos[6], pos[7]]

    # Loading existing data from the output file
    data = np.loadtxt('saved_data/'+ output_file,skiprows=1,usecols=(0,1,2))

    # Filter out the lines in data that match the x and y coordinates
    lines = np.array([], dtype=float)
    for line in data:
        if not (line[1] == pos[0] and line[2] == pos[1]):
            lines = np.append(lines,[i for i in line])

    lines = np.append(lines,[output])

    # Reshape to 2D array with 3 columns
    lines = lines.reshape(-1, 12) 

    # Sort by x, then y
    sorted_data = lines[np.lexsort((lines[:, 1], lines[:, 0]))]  

    # Save the sorted data back to the file
    np.savetxt(output_file, sorted_data, fmt='%s', delimiter=' ', header=header, comments='')

def main():
    # Move robot to the specified position
    pos = move_robot()
    mag = read_magnetic_field()

    save_to_file(mag,pos)

if __name__ == "__main__":
    main()
