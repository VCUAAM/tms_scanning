import math
import numpy as np
import warnings
import socket
import tkinter as tk
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from matplotlib.colors import Normalize
from tkinter import messagebox

def socket_check():
    while True:
        try:
            socket.create_connection(("192.168.1.102",30004), timeout=5)
        except OSError:
            root = tk.Tk()
            root.withdraw()
            messagebox.showerror("Error", "Make sure robot is on and computer is connected to VCU_UR5e network")
            continue
        break
# Function to check and filter noise from the data
def noise_filter(data):
    #Calculate mean of absolute value of each value in data
    mean = np.mean(np.abs(data))
    if mean < 0.05:
        return [0 for i in data]
    else:
        return data

# Function to check with user if graph is satisfactory
def ask_yes_no_popup(string):
    root = tk.Tk()
    root.withdraw()

    response = messagebox.askyesno("Confirmation", string)

    if response:
        root.destroy()
        return True
    else:
        root.destroy()
        return False

# Function to plot all the currently collected points and display them to the viewer
def plot_map(save_file):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        data = np.loadtxt('saved_data/'+ save_file,skiprows=1,delimiter=' ')
    
    # If data is 1D, reshape it to 2D with one row
    if data.ndim == 1:
        data = data.reshape(1, -1)

    x_coords = data[:,1]
    y_coords = data[:,2]
    labels = [round(i,2) for i in data[:,0]]

    fig, ax = plt.subplots(constrained_layout=True)

    cmap = plt.get_cmap('cool')

    # Normalize the label values to the range [0, 1]
    norm = Normalize(vmin=np.min(labels), vmax=np.max(labels))

    # We use the invisible scatter plot to help create the color bar later
    # The `c` parameter is where we pass the values to be mapped to colors
    scatter = ax.scatter(x_coords, y_coords, c=labels, cmap=cmap, norm=norm)

    # Loop through each point to add the label inside a circle
    for i, label in enumerate(labels):
        color = cmap(norm(label))  # Get the color from the colormap for the current label
        ax.text(
            x_coords[i], y_coords[i], str(label),
            ha="center", 
            va="center", 
            bbox=dict(
                boxstyle="circle,pad=0.5",
                fc=color,                  # Use the dynamically calculated color
                ec="black",
                lw=1
            )
        )
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_xlim(np.min(x_coords) - 1, np.max(x_coords) + 1)
    ax.set_ylim(np.min(y_coords) - 1, np.max(y_coords) + 1)
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    ax.set_title("Currently Collected Points")

    ax.grid(True) 
    plt.show(block=False)

def save_to_file(mag,pos,save_file):
    header = 'Mag (V/m)\tx_rel (mm)\ty_rel (mm)\tx_field (V/m)\ty_field (V/m)\tz_field (V/m)\tx (m)\ty (m)\tz (m)\trx (rad)\try (rad)\trz (rad)'
    output = [mag[3], pos[0], pos[1], mag[0], mag[1], mag[2], pos[2], pos[3], pos[4], pos[5], pos[6], pos[7]]

    # Loading existing data from the output file
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        data = np.loadtxt('saved_data/'+ save_file,skiprows=1,delimiter=' ')
    
    # If data is 1D, reshape it to 2D with one row
    if data.ndim == 1:
        data = data.reshape(1, -1)

    # Filter out the lines in data that match the x and y coordinates
    lines = np.array([], dtype=float)

    # Only do this if there is existing data
    if data.size > 0:
        for line in data:
            if (line[1] == pos[0] and line[2] == pos[1]):
                    continue
            lines = np.append(lines,[i for i in line])

    lines = np.append(lines,[output])

    # Reshape to 2D array with 12 columns
    lines = lines.reshape(-1, 12) 

    # Sort by x, then y
    sorted_data = lines[np.lexsort((lines[:, 1], lines[:, 0]))]  
    # Save the sorted data back to the file
    np.savetxt('saved_data/' + save_file, sorted_data, fmt='%s', delimiter=' ', header=header, comments='')
    
def axis_angle_to_matrix(omega):
    """Convert axis-angle vector omega = (rx, ry, rz) to rotation matrix R."""
    theta = np.linalg.norm(omega)
    if theta < 1e-8:
        return np.eye(3)
    u = omega / theta
    ux, uy, uz = u
    # Rodrigues’ formula
    c = math.cos(theta)
    s = math.sin(theta)
    one_c = 1 - c
    # Cross-product matrix of u
    Ux = np.array([[   0, -uz,  uy],
                   [ uz,   0, -ux],
                   [-uy,  ux,   0]])
    # Outer product u u^T
    uuT = np.outer(u, u)
    R = c * np.eye(3) + one_c * uuT + s * Ux
    return R

def matrix_to_axis_angle(R):
    """Convert rotation matrix R to axis-angle vector (rx, ry, rz)."""
    # Ensure numerical stability
    eps = 1e-8
    # Compute angle
    trace = np.trace(R)
    cos_theta = (trace - 1) / 2
    # Clamp
    cos_theta = max(min(cos_theta, 1.0), -1.0)
    theta = math.acos(cos_theta)
    if abs(theta) < eps:
        # No rotation
        return np.array([0.0, 0.0, 0.0])
    # If nearly pi, treatment is more subtle
    sin_theta = math.sin(theta)
    # Axis from off-diagonal elements
    rx = (R[2,1] - R[1,2]) / (2 * sin_theta)
    ry = (R[0,2] - R[2,0]) / (2 * sin_theta)
    rz = (R[1,0] - R[0,1]) / (2 * sin_theta)
    u = np.array([rx, ry, rz])
    # Normalize axis, then multiply by angle
    u_norm = np.linalg.norm(u)
    if u_norm < eps:
        # axis direction ill-conditioned; fallback
        # Could pick arbitrary axis
        u = np.array([1.0, 0.0, 0.0])
    else:
        u = u / u_norm
    return u * theta

def pose_inv(pose):
    """
    pose: iterable or list/tuple of 6 numbers [x, y, z, rx, ry, rz]
    Returns: numpy array [x_inv, y_inv, z_inv, rx_inv, ry_inv, rz_inv]
    """
    x, y, z, rx, ry, rz = pose
    T = np.array([x, y, z])
    R = axis_angle_to_matrix(np.array([rx, ry, rz]))
    # Invert: R_inv = R.T, T_inv = - R.T * T
    R_inv = R.T
    T_inv = - R_inv.dot(T)
    # Convert back to axis-angle
    omega_inv = matrix_to_axis_angle(R_inv)
    return np.array([T_inv[0], T_inv[1], T_inv[2],
                     omega_inv[0], omega_inv[1], omega_inv[2]])

# Example test
if __name__ == "__main__":
    inp = [0.2, 0.5, 0.1, 1.57, 0, 3.14]
    out = pose_inv(inp)
    print("Inverse pose:", out)
