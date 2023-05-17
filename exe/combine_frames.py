import os
import glob
import json
import numpy as np

# Define function to load data from CSV file
def load_data(filename):
    data = np.loadtxt(filename, delimiter=',')
    return data

# Read the path to the CSV files from the JSON file
with open(os.path.expanduser('../generalSettings.json'), 'r') as f:
    settings = json.load(f)
path_to_csv_files = os.path.expanduser(settings['framesOutput'])

# Get a list of all CSV files in the directory
files = glob.glob(os.path.join(path_to_csv_files, 'frame_*_orbs.csv'))

# Initialize an empty array to store the combined data
combined_data = np.empty((0, 3), float)

# Loop through all CSV files and append the data to the combined_data array
for filename in files:
    data = load_data(filename)
    combined_data = np.vstack((combined_data, data))

# Save the combined data to a single CSV file
output_file = os.path.join(path_to_csv_files, '/home/liam/combined.csv')
np.savetxt(output_file, combined_data, delimiter=',')
