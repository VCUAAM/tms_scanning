import rtde_control
import rtde_receive
import tkinter as tk
from tkinter import messagebox

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

print([round(i,3) for i in rtde_r.getActualTCPPose()])