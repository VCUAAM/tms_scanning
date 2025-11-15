import nidaqmx
from nidaqmx.constants import READ_ALL_AVAILABLE, AcquisitionType
import dev.helper_functions as hf
import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
from math import *
from tkinter import messagebox

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
                with nidaqmx.Task() as ai_task:
                    # Add analog input channels for three axes
                    ai_task.ai_channels.add_ai_voltage_chan("Dev1/ai0")  # x
                    ai_task.ai_channels.add_ai_voltage_chan("Dev1/ai1")  # y
                    ai_task.ai_channels.add_ai_voltage_chan("Dev1/ai2") #z

                    # Configure timing and trigger from the stimulator
                    ai_task.timing.cfg_samp_clk_timing(sample_rate, sample_mode=AcquisitionType.FINITE, samps_per_chan=num_samples)
                    ai_task.triggers.start_trigger.cfg_dig_edge_start_trig("/Dev1/PFI0")

                    # Initiate data acquisition
                    ai_task.start()

                    # Read the acquired data from all channels
                    data = ai_task.read(READ_ALL_AVAILABLE)

                    # Stop the tasks
                    ai_task.stop()
            except:
                root = tk.Tk()
                root.withdraw()
                messagebox.showerror("Error", "Press OK and then turn the stimulator on")
                continue
            finally:
                quit()
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

        mag_plt = np.ones(time_axis)*mag_total

        # Plotting the data
        plt.figure(figsize=(10, 5))
        plt.plot(time_axis, ai0, label="AI0")   
        plt.plot(time_axis, ai1, label="AI1")
        plt.plot(time_axis, ai2, label="AI2")
        plt.plot(time_axis, mag_plt, label="magnitude")
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
    
if __name__ == "__main__":
        try:
            while True:
                mag = read_magnetic_field()
                test = hf.ask_yes_no_popup("Keep collecting points?")
                plt.close()
                if test == True:
                    continue
                break   

        except KeyboardInterrupt:
            print("KeyboardInterrupt detected. Disconnecting...")
            
        finally:
            pass
