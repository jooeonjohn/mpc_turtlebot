import pandas as pd
import numpy as np
from scipy.interpolate import interp1d

# Read the CSV file into a DataFrame without headers
# Manually specify column names as 'x' and 'y'
data = pd.read_csv('/home/jooeon/colcon_ws/src/mpc/path/pathlog.csv', header=None, names=['x', 'y'])

# Extract the x and y coordinates from the DataFrame
x = data['x'].values
y = data['y'].values

# Calculate the Euclidean distance between consecutive points
distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
cumulative_distances = np.concatenate(([0], np.cumsum(distances)))

# Create an interpolation function based on the cumulative distance
interpolation_function = interp1d(cumulative_distances, np.column_stack((x, y)), kind='linear', axis=0, fill_value="extrapolate")

# Define the new distances at a step of 0.1 units
distance_new = np.arange(0, cumulative_distances[-1], 0.1)

# Interpolate the new coordinates based on the cumulative distance
interpolated_coords = interpolation_function(distance_new)

# Create a DataFrame with the interpolated data
interpolated_data = pd.DataFrame(interpolated_coords, columns=['x', 'y'])

# Save the interpolated data to a new CSV file
interpolated_data.to_csv('/home/jooeon/colcon_ws/src/mpc/path/lab_path3.csv', index=False)
