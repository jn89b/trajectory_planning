import matplotlib.pyplot as plt
import numpy as np

# Define the dimensions of your voxel grid
x_dim = 10  # Number of voxels in the x-direction
y_dim = 10  # Number of voxels in the y-direction
z_dim = 10  # Number of voxels in the z-direction

# Create a 3D NumPy array representing the voxel data
# In this example, we create a 3x3x3 grid with some voxels turned on (1) and others off (0)
voxel_data = np.zeros((x_dim, y_dim, z_dim))

# List of coordinates (including positive and negative coordinates)
coordinates = [(2, 2, 2), (-3, -3, -3), (-1, 1, 1), (3, -2, 2)]

# Translate coordinates into indices within the voxel_data array
for coord in coordinates:
    x, y, z = coord
    x_index = x + (x_dim // 2)  # Translate negative x to array index
    y_index = y + (y_dim // 2)  # Translate negative y to array index
    z_index = z + (z_dim // 2)  # Translate negative z to array index
    voxel_data[x_index, y_index, z_index] = 1

# Create a figure and a 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Use the voxels function to create the voxel plot
ax.voxels(
    voxel_data,
    facecolors='b',  # Color for active (1) voxels
    edgecolor='k'    # Color for voxel edges
)

# Set axis labels
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

# Show the plot
plt.show()
