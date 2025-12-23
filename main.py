from dev.daq import DAQController
from dev.robot_controller import RobotController
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from matplotlib.colors import Normalize
import tkinter as tk
from tkinter import messagebox
import warnings

class ExperimentController:
    def __init__(self, robot, daq, output_file):
        self.robot = robot
        self.daq = daq
        self.output_file = output_file
        self.visualize = False

    # Function to check with user and ask a question 
    def ask_yes_no_popup(self,string):
        root = tk.Tk()
        root.withdraw()

        response = messagebox.askyesno("Confirmation", string)

        if response:
            root.destroy()
            return True
        else:
            root.destroy()
            return False
    
    # Function to create heatmap of collected data points 
    def create_heatmap(self):
        # Load the data from the text file
        data = np.loadtxt('saved_data/'+ self.output_file,skiprows=1,usecols=(0,1,2))

        # Extract the x, y coordinates and magnitude range
        x = data[:, 1]
        x = [int(i) for i in x]
        y = data[:, 2]
        y = [int(i) for i in y]
        magnitude_range = data[:, 0]

        # Create a 2D grid for the heatmap
        x_unique = np.unique(x)
        y_unique = np.unique(y)
        heatmap = np.zeros((len(y_unique), len(x_unique)))

        # Populate the heatmap with magnitude ranges
        for i in range(len(x)):
            x_index = np.where(x_unique == x[i])[0][0]
            y_index = np.where(y_unique == y[i])[0][0]
            heatmap[y_index, x_index] = magnitude_range[i]

        # Create a heatmap using matplotlib 
        fontsize = 16
        plt.rcParams['mathtext.fontset'] = 'cm'
        plt.imshow(heatmap, extent=(np.min(x_unique), np.max(x_unique), np.min(y_unique), np.max(y_unique)),interpolation='bicubic', origin='lower', cmap='plasma', aspect='auto',vmin=np.min(magnitude_range), vmax=np.max(magnitude_range))
        hm = plt.colorbar()
        hm.set_label('E-Field Magnitude Range (V/m)', fontsize=fontsize,labelpad=15)
        hm.ax.tick_params(labelsize=fontsize-2)
        plt.xlabel('(mm)',fontsize=fontsize)
        plt.ylabel('(mm)',fontsize=fontsize)
        plt.tick_params(axis='both', which='major', labelsize=(fontsize-2))
        plt.grid(False)

        # Configure these for final version of heatmap image if desired
        '''
        xticks = [0, 2, 4, 6, 8, 10]
        yticks = [-5, -3, -1, 1, 3, 5]
        labels = [0,2,4,6,8,10]
        plt.xticks(xticks, labels)
        plt.yticks(yticks, labels)
        '''

        # Show the heatmap
        plt.show()

        # Save the heatmap as an image file
        plt.savefig('heatmap_magnitude_range.png', dpi=300, bbox_inches='tight')
        plt.close()

        print("Heatmap created and saved as 'heatmap_magnitude_range.png'")
        print('Make sure to rename the file before running this function again')
        

    # Function to plot all the currently collected points and display them to the viewer
    def plot_map(self,block=True):
        # Loading data from output file
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            data = np.loadtxt('saved_data/'+ self.output_file,skiprows=1)
        
        # Reshaping data to correct size
        if data.ndim == 1:
            data = data.reshape(1, -1)

        # Extracting relevant data 
        x_coords = data[:,1]
        y_coords = data[:,2]
        labels = [round(i,2) for i in data[:,0]]

        # Creating the plot
        _, ax = plt.subplots(constrained_layout=True)
        cmap = plt.get_cmap('cool')
        norm = Normalize(vmin=np.min(labels), vmax=np.max(labels))
        ax.scatter(x_coords, y_coords, c=labels, cmap=cmap, norm=norm)

        # Loop through each point to add the label inside a circle
        for i, label in enumerate(labels):
            color = cmap(norm(label))  # Get the color from the colormap for the current label
            ax.text(x_coords[i], y_coords[i], str(label),ha="center", va="center",bbox=dict(boxstyle="circle,pad=0.5",fc=color,ec="black",lw=1))
        
        # Setting plot properties
        ax.set_xlabel("x (mm)")
        ax.set_ylabel("y (mm)")
        ax.set_xlim(np.min(x_coords) - 1, np.max(x_coords) + 1)
        ax.set_ylim(np.min(y_coords) - 1, np.max(y_coords) + 1)
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        ax.yaxis.set_major_locator(MaxNLocator(integer=True))
        ax.set_title("Currently Collected Points")
        ax.grid(True) 
        
        # Displaying plot
        plt.show(block=block)
    
    # Function to save the collected data to a file
    def save_to_file(self,mag):

        # Defining format for the header
        fmt = [
            '%8.2f',  # mag_total
            '%6d',    # x_rel
            '%6d',    # y_rel
            '%8.2f',  # mag_x
            '%8.2f',  # mag_y
            '%8.2f',  # mag_z
            '%8.4f',  # x
            '%8.4f',  # y
            '%8.4f',  # z
            '%8.4f',  # rx
            '%8.4f',  # ry
            '%8.4f'   # rz
        ]
        
        # Creating header for the output file
        labels = ['Mag', 'Xrel', 'Yrel', 'Bx', 'By', 'Bz', 'x', 'y', 'z', 'rx', 'ry', 'rz']
        widths = [8, 6, 4, 6, 8, 6, 8, 7, 7, 7, 7, 7]
        header = ''.join(f'{label:<{w}}  ' for label, w in zip(labels, widths)).rstrip()

        # Collecting positional data from the robot
        x = int(self.robot.xcoord)
        y = int(self.robot.ycoord)
        tcp = self.robot.get_tcp()
        
        # Creating the output line
        lines = [[mag[3], x, y, mag[0], mag[1], mag[2], *tcp]]

        # Loading existing data from the output file
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            data = np.loadtxt('saved_data/'+ self.output_file,skiprows=1)
        
        # Reshaping imported data to correct orientation
        if data.ndim == 1:
            data = data.reshape(1, -1)

        # Filter out repeat lines if it matches current position data
        if data.size > 0:
            for line in data:
                if (line[1] == x and line[2] == y):
                        continue
                lines.insert(-2,[i for i in line])

        # Reshape to 2D array with 12 columns
        lines = np.array(lines).reshape(-1, 12) 

        # Sort by x, then y
        sorted_data = lines[np.lexsort((lines[:, 2], lines[:, 1]))]  

        # Save the sorted data back to the file
        np.savetxt('saved_data/' + self.output_file, sorted_data, fmt=fmt, header=header)

    # Function that will collect pulses from the stimulator, process them, and filter out outliers before saving to file
    def collect_point(self):
        mag_buffer = []

        while True:
            # Collecting and processing pulse from stimulator
            pulse = self.daq.collect_pulse()
            mag = self.daq.process_pulse(pulse)

            # This will ignore all pulses above a certain threshold
            if mag[3] > daq.threshold:
                continue
            
            # This will plot the waveform for the first pulse if visualizing
            if self.visualize and len(mag_buffer) == 0:
                self.daq.plot_waveform(pulse)

            # Adding the current pulse the the buffer before averaging
            mag_buffer.append(mag)

            if len(mag_buffer) == 10:
                # Removing outliers based on Median Absolute Deviation
                mag_np = np.array(mag_buffer)
                mag_t = mag_np[:,3]
                med = np.median(mag_t)
                mad = np.median(np.abs(mag_t - med))
                robust_sd = 1.4826 * mad
                mask = np.abs(mag_t - med) <= self.daq.sd_filter * robust_sd
                mag_buffer = mag_np[mask].tolist()

                # Recollecting points if any outliers were removed
                if len(mag_buffer) > np.sum(mask):
                    continue
                
                # Saving the averaged pulse to the file and plotting it
                avg = np.mean(np.array(mag_buffer), axis=0).tolist()
                self.save_to_file(avg)
                self.plot_map(block=False)

                # Checking with user to make sure the pulse is good
                if self.ask_yes_no_popup(f"Is the pulse at {self.robot.xcoord,self.robot.ycoord} good?"):
                    plt.close()
                    return
                else:
                    plt.close()
                    mag_buffer.clear()

    # Iterating function to query desired collection point and pulse data
    def run(self):
        while True:
            self.robot.move_to_collection_point()
            self.collect_point()

            if self.ask_yes_no_popup("Keep collecting points?"):
                return
            rob.xcoord = None
            rob.ycoord = None

if __name__ == "__main__":
    HOME_POSE = [-0.005, -0.526, 0.222, 2.48, -0.919, 0.712] # figure of 8
    #HOME_POSE = [-0.005, -0.526, 0.222, 2.48, -0.919, 0.712] # you can save multiple positions, just comment the ones not in use

    ''' Change this to match the name of the output file'''
    OUTPUT_FILE = "121725_fo8_50.txt"

    rob = RobotController()
    rob.home_pose = HOME_POSE
    rob.debugging = False # If true, will print out force values during normalization procedure

    daq = DAQController()
    daq.sample_rate = 40000 # Hz
    daq.pretrigger_samples = 500 # Number of samples to collect before the pulse
    daq.num_samples = 600 # Number of samples to collect
    daq.pre_cutoff = 93 # Number of samples to remove from the beginning of the signal
    daq.post_cutoff = 490 # Number of samples to remove from the end of the signal

    daq.intervals = 0.75 # Number of seconds between pulses
    daq.samples = 10 # Number of samples for each position
    daq.threshold = 200 # Magnitudes measured above this are ignored
    daq.sd_filter = 2 # Values this many standard deviations from the mean are ignored
    
    exp = ExperimentController(rob, daq, OUTPUT_FILE)

    try:
        ''' Uncomment this line for data collection '''
        #exp.run()

        ''' Uncomment this line to test a single pulse and plot it'''
        #daq.single_pulse()

        ''' Uncomment this line to see the collected data points so far'''
        #exp.plot_map()

        ''' Uncomment this line to create a heatmap of the collected data points '''
        #exp.create_heatmap()

        ''' Uncomment this line to export the current position of the robot '''
        #rob.print_current_pos()

        ''' Uncomment this line to move the robot to the home position '''
        #rob.move_home()

        ''' Uncomment this line to normalize the coil at the current location 
        (You should probably save the robot position first in case it fails)'''
        #rob.normalize_coil()

    except KeyboardInterrupt:
        print("KeyboardInterrupt detected.")
    finally:
        rob.shutdown()
        print("Shutdown complete.")