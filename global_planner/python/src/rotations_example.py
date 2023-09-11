import numpy as np

# Define the original position (x0, y0, z0)
x0, y0, z0 = 1.0, 2.0, 3.0

# Define the spherical coordinates (radius, yaw angle, pitch angle) in radians
radius = 5.0
yaw_angle = np.deg2rad(45)  # Convert degrees to radians
pitch_angle = np.deg2rad(0)  # Convert degrees to radians

x_vec = []
y_vec = [] 
z_vec = []

bearings = np.linspace(0, np.deg2rad(45), 5)
pitchings = np.linspace(np.deg2rad(25), np.deg2rad(65), 5)

for bearing in bearings:
    for pitching in pitchings:
        x_new = x0 + radius * np.sin(pitching) * np.cos(bearing)
        y_new = y0 + radius * np.sin(pitching) * np.sin(bearing)
        z_new = z0 + radius * np.cos(pitching)
        x_vec.append(x_new)
        y_vec.append(y_new)
        z_vec.append(z_new)

# Convert spherical coordinates to Cartesian coordinates
x_new = x0 + radius * np.sin(pitch_angle) * np.cos(yaw_angle)
y_new = y0 + radius * np.sin(pitch_angle) * np.sin(yaw_angle)
z_new = z0 + radius * np.cos(pitch_angle)

# Print the new position
print(f"New position: x={x_new}, y={y_new}, z={z_new}")
# plot this 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Create a figure and a 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the original point
ax.scatter(x0, y0, z0, c='k', s=100, label='original point')

for x,y,z in zip(x_vec, y_vec, z_vec):
    ax.scatter(x, y, z, c='r', s=100)

# Plot the point after the first rotation

# Plot the point after both rotations
ax.scatter(x_new, y_new, z_new, c='b', s=100, label='after both rotations')

# Set axis labels
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

# Set axis limits
ax.set_xlim3d(-10, 10)
ax.set_ylim3d(-10, 10)
ax.set_zlim3d(-10, 10)

# Add a legend
ax.legend()

# Show the plot
plt.show()
