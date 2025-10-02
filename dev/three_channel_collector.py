import nidaqmx
import numpy as np
import matplotlib.pyplot as plt

from nidaqmx.constants import READ_ALL_AVAILABLE, AcquisitionType

sample_rate = 90000 #Hz
num_samples = 35 # number of samples per channel
pre_wave_cutoff = 7 #number of samples removed from beginning
amplification_factor = 11

def garbage_check(data):
    #Calculate mean of absolute value of each value in data
    mean = np.mean(np.abs(data))
    if mean < 0.05:
        return [0 for i in data]
    else:
        return data
    
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

ai0 = data[0][pre_wave_cutoff:]
mag_x = max((ai0)) - min(garbage_check(ai0))
ai0 = [float(i)/amplification_factor/0.003 for i in ai0]
ai1 = data[1][pre_wave_cutoff:]
mag_y = max(garbage_check(ai1)) - min(garbage_check(ai1))
ai1 = [float(i)/amplification_factor/0.003 for i in ai1]
ai2 = data[2][pre_wave_cutoff:]
mag_z = max(garbage_check(ai2)) - min(garbage_check(ai2))
ai2 = [float(i)/amplification_factor/0.003 for i in ai2]
#print("Acquired data AI0: [" + ", ".join(f"{value:f}" for value in ai0) + "]")
#print("Acquired data AI1: [" + ", ".join(f"{value:f}" for value in ai1) + "]")
#print("Acquired data AI2: [" + ", ".join(f"{value:f}" for value in ai2) + "]")

print(f"Mag X: {mag_x:.2f} V/m"
      f"\nMag Y: {mag_y:.2f} V/m"
      f"\nMag Z: {mag_z:.2f} V/m")
dt = 1 / sample_rate
time_axis = np.linspace(0, (num_samples - pre_wave_cutoff - 1) * dt, num_samples - pre_wave_cutoff)

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