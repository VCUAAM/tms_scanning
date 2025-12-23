import tkinter as tk
from tkinter import messagebox
import numpy as np
import matplotlib.pyplot as plt
from math import *
import time
import nidaqmx
from nidaqmx.constants import READ_ALL_AVAILABLE, AcquisitionType

class DAQController:
    def __init__(self):
        self.sample_rate = 40000 # Hz
        self.pretrigger_samples = 500 # Number of samples collected before trigger 
        self.num_samples = 600 # Number of samples to collect
        self.pre_cutoff = 93 # Number of samples to remove from the beginning of the signal
        self.post_cutoff = 490 # Number of samples to remove from the end of the signal
        self.amp = 6 # Amplifier gain

        self.intervals = 0.75 # Number of seconds between pulses
        self.samples = 10 # Number of samples for each position
        self.threshold = 200 # Magnitudes measured above this are ignored   
        self.sd_filter = 2 # Values this many standard deviations from the mean are ignored

    # Function to check and filter noise from the data
    def noise_filter(self,data):
        mean = np.mean(np.abs(data))
        if mean < 0.05:
            return [0 for i in data]
        else:
            return data
    
    # Debugging function to test a single pulse, plot it, and check the magnitudes
    def single_pulse(self):
        data = self.collect_pulse()
        mags = self.process_pulse(data)
        print(f"Mag X: {mags[0]:.2f} V/m"
              f"\nMag Y: {mags[1]:.2f} V/m"
              f"\nMag Z: {mags[2]:.2f} V/m"
              f"\nMag Total: {mags[3]:.2f} V/m")
        self.plot_waveform(data)

    # This will be called to pulse the stimulator and collect the data
    def collect_pulse(self):
        while True:
            time.sleep(self.intervals)
            try:
                with nidaqmx.Task() as ai_task, nidaqmx.Task() as ao_task:
                    # Defining channels for analog inputs (phantom probe channels)
                    for ch in ["Dev1/ai0", "Dev1/ai1", "Dev1/ai2"]:
                        ai_task.ai_channels.add_ai_voltage_chan(ch)

                    # Defining timing for analog input task and reference trigger
                    ai_task.timing.cfg_samp_clk_timing(self.sample_rate,sample_mode=AcquisitionType.FINITE,samps_per_chan=self.num_samples)
                    ai_task.triggers.reference_trigger.cfg_dig_edge_ref_trig(trigger_source="/Dev1/PFI0",pretrigger_samples=self.pretrigger_samples)

                    # Creating analog output task to pulse stimulator
                    ao_task.ao_channels.add_ao_voltage_chan("Dev1/ao0")

                    # Starting data listening and pulsing stimulator
                    ai_task.start()
                    ao_task.write(0)
                    ao_task.write(5)
                    ao_task.write(0)

                    data = ai_task.read(READ_ALL_AVAILABLE)

                    return data

            except Exception:
                root = tk.Tk()
                root.withdraw()
                messagebox.showerror("Error","Press OK and then turn the stimulator on")

    # This function determines the magnitude of the pulse
    def process_pulse(self, data):
        mags = []

        for axis in range(3):
            sig = data[axis][self.pre_cutoff:-self.post_cutoff]
            sig = [float(v) / self.amp / 0.003 for v in sig]
            filt = self.noise_filter(sig)
            mags.append(max(abs(max(filt)), abs(min(filt))))

        total = sqrt(sum(m ** 2 for m in mags))

        return mags + [total]

    # Plots the waveform of the pulse, primarily used for debugging
    def plot_waveform(self, data):
        # Filtering excess data from the beginning and end of the signal
        sigs = [data[i][int(5*self.pre_cutoff/6):-int(19*self.post_cutoff/20)] for i in range(3)]

        # Determing time step based on frequency and creating x axis
        dt = 1 / self.sample_rate
        t = np.arange(len(sigs[0]))*dt

        # Plotting the pulse in each channel
        for i, sig in enumerate(sigs):
            plt.plot(t, sig, label=f"AI{i}")

        # Configuring plot parameters 
        plt.xlim(0, 0.001)
        plt.xlabel("Time (s)")
        plt.ylabel("E-Field Magnitude Range (V/m)")
        plt.title("NI-USB 6363 DAQ Data")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()
    
if __name__ == "__main__":
    daq_controller = DAQController()
    daq_controller.single_pulse() 