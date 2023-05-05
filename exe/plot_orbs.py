import os
import glob
import json
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# Define function to load data from CSV file
def load_data(filename):
    data = np.loadtxt(filename, delimiter=',')
    x = data[:, 0]
    y = data[:, 1]
    z = data[:, 2]
    return x, y, z

# Read the path to the CSV files from the JSON file
with open(os.path.expanduser('../generalSettings.json'), 'r') as f:
    settings = json.load(f)
path_to_csv_files = os.path.expanduser(settings['framesOutput'])

# Create a 3D axes object
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Get a list of all CSV files in the directory
files = glob.glob(os.path.join(path_to_csv_files, 'frame_*_orbs.csv'))

# Loop through all CSV files and plot the points
for filename in files:
    x, y, z = load_data(filename)
    ax.scatter(x, y, z, s=1)

# Set the axis labels and limits
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_zlim(-10, 10)

# Add a legend and show the plot
ax.legend()
plt.show()
