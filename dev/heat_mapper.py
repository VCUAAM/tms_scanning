import numpy as np
import matplotlib.pyplot as plt

directory = 'Abstract_Excel_Plots/magnitude_range.txt'

# Import the data from the .txt file into a NumPy array with the first two columns as x and y coordinates and the third column as magnitude range
# Load the data from the text file
data = np.loadtxt(directory,skiprows=1,usecols=(0,1,2))

# Extract the x, y coordinates and magnitude range
x = data[:, 0]
x = [int(i) for i in x]
y = data[:, 1]
y = [int(i) for i in y]
magnitude_range = data[:, 2]
magnitude_range = [float(i)/11/0.003 for i in magnitude_range]  # Adjusting the magnitude range
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
fsize = 16
plt.rcParams['mathtext.fontset'] = 'cm'
plt.imshow(heatmap, extent=(np.min(x_unique), np.max(x_unique), np.min(y_unique), np.max(y_unique)),interpolation='gaussian', origin='lower', cmap='plasma', aspect='auto',vmin=np.min(magnitude_range), vmax=np.max(magnitude_range))
hm = plt.colorbar()
hm.set_label('E-Field Magnitude Range (V/m)', fontsize=fsize,labelpad=15)
hm.ax.tick_params(labelsize=fsize-2)
plt.xlabel('(mm)',fontsize=fsize)
plt.ylabel('(mm)',fontsize=fsize)
plt.tick_params(axis='both', which='major', labelsize=(fsize-2))
xticks = [0, 2, 4, 6, 8, 10]
yticks = [-5, -3, -1, 1, 3, 5]
labels = [0,2,4,6,8,10]
plt.xticks(xticks, labels)
plt.yticks(yticks, labels)
plt.grid(False)

# Show the heatmap
plt.show()

# Save the heatmap as an image file
plt.savefig('heatmap_magnitude_range.png', dpi=300, bbox_inches='tight')

# Close the plot to free up memory
plt.close()
