import nidaqmx
import numpy as np
import matplotlib.pyplot as plt
import rtde_control
import rtde_receive
from nidaqmx.constants import READ_ALL_AVAILABLE, AcquisitionType

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
    
# Function to collect magnetic field data from the DAQ
def read_magnetic_field():
    with nidaqmx.Task() as ai_task, nidaqmx.Task() as ao_task:
    
        ai_task.ai_channels.add_ai_voltage_chan("Dev1/ai0")
        ai_task.ai_channels.add_ai_voltage_chan("Dev1/ai1")
        ai_task.ai_channels.add_ai_voltage_chan("Dev1/ai2")
        ai_task.timing.cfg_samp_clk_timing(sample_rate, sample_mode=AcquisitionType.FINITE, samps_per_chan=num_samples)
        ai_task.triggers.start_trigger.cfg_dig_edge_start_trig("/Dev1/PFI0")

        ao_task.ao_channels.add_ao_voltage_chan("Dev1/ao0")

        ai_task.start()
        ao_task.write(0.0)
        ao_task.write(5.0)
        ao_task.write(0)
        data = ai_task.read(READ_ALL_AVAILABLE)
        ai_task.stop()
        ao_task.stop()
    

# Move the robot to the specified coordinate using RTDE communication with the UR5e
def move_robot(xcoord, ycoord):
    # Move robot to 0,0 coordinate
    rtde_c.moveJ(home_pos_j)
    # Move robot to specified coordinate
    target_position = [xcoord + home_position[0], ycoord + home_position[1], home_position[2], home_position[3], home_position[4], home_position[5]]
    target_tcp = rtde_c.getInverseKinematics(target_position)
    rtde_c.moveJ(target_tcp)

# Collects and returns the current pose of the robot
def get_robot_position():
    pos = 0 # Will be current pose of robot
    
    return pos


def trigger_read_tms_readings():


# x and y are data points in width and height of rectangle, where "nom" is the number of millimeters between each data point
def main(x,y,nom):
    headings = ['Mag','x_pos','y_pos','x_field','y_field','z_field','x_bot','y_bot','z_bot','x_']
    for i in range(-x/2,x/2):
        for j in range(-y/2,y/2):

            # Move robot to the specified position
            move_robot(i*nom, j*nom)
            pos = get_robot_position()
            mag = trigger_read_tms_readings()


