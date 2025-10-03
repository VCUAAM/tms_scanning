import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from matplotlib.colors import Normalize
import numpy as np

data = np.loadtxt('saved_data/test_data.txt',skiprows=1,delimiter=' ')

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
ax.set_xlim(np.min(x_coords) - 1, np.max(x_coords) + 1)
ax.set_ylim(np.min(y_coords) - 1, np.max(y_coords) + 1)
ax.xaxis.set_major_locator(MaxNLocator(integer=True))
ax.yaxis.set_major_locator(MaxNLocator(integer=True))
ax.set_title("Currently Collected Points")

ax.grid(True) 
plt.show()