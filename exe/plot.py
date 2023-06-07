import os
import glob
import json
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# Define function to load data from CSV file
def load_data(filename):
    data = np.loadtxt(filename, delimiter=',', usecols=[0,1,2])
    x = data[:, 0]
    y = data[:, 1]
    z = data[:, 2]
    return x, y, z

# Create a 3D axes object
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Loop through all CSV files and plot the points
x, y, z = load_data("/home/liam/Documents/slamMaps/example_mapping11/cloud1.csv")
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
