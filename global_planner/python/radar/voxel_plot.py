import matplotlib.pyplot as plt
import numpy as np

# Create a list of voxel coordinates as (x, y, z) tuples
voxel_coordinates = [(1, 1, 1), (2, 2, 2), (3, 3, 3), (4, 4, 4), (5, 5, 5)]

# Determine the dimensions of the grid based on the maximum coordinates
max_coords = np.max(voxel_coordinates, axis=0)
grid_shape = tuple(coord + 1 for coord in max_coords)

# Create a 3D array to represent the voxel grid
voxel_grid = np.zeros(grid_shape)

# Set the voxel values to 1 for the specified coordinates
for coord in voxel_coordinates:
    voxel_grid[coord] = 1

# Create a figure and a 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Use the `voxels` function to plot the voxel grid
ax.voxels(voxel_grid, edgecolor='k')

# Set axis labels
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

# Show the plot
plt.show()
