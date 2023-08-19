import numpy as np

#compute the rotation matrix in 3d
def rot3d(roll, pitch, yaw):
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    #yaw pitch roll
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R

#example of rotating a point
point = np.array([1, 0, 0])
roll = np.deg2rad(45)
pitch = np.deg2rad(45)
yaw = np.deg2rad(45)

rotated_point = rot3d(roll, pitch, yaw) @ point
# print("rotated point: ", rotated_point)

#animate the rotation of a point
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

#set the limits of the plot
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)

#set the labels of the plot
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

#set limits
limit = 100
ax.set_xlim(-limit, limit)
#set the initial point
point = np.array([1, 0, 0])
point = point.reshape(3, 1)
point_plot, = ax.plot(point[0], point[1], point[2], 'ro')

#plot the initial point
ax.plot(0, 0, 0, 'o', label='origin')

#set the initial rotation angles
roll = np.deg2rad(0)
pitch = np.deg2rad(0)
yaw = np.deg2rad(0)

#function to update the plot
def update(i):
    global roll, pitch, yaw
    roll = np.deg2rad(0)
    pitch = np.deg2rad(0)
    yaw = np.deg2rad(i)
    rotated_point = rot3d(roll, pitch, yaw) @ point
    point_plot.set_data(rotated_point[0], rotated_point[1])
    point_plot.set_3d_properties(rotated_point[2])

    #update title of the plot based on current rotation angles
    plt.title("roll: " + str(np.rad2deg(roll)) + " pitch: " + str(np.rad2deg(pitch)) + " yaw: " + str(np.rad2deg(yaw)))

    return point_plot

#animate the plot
anim = animation.FuncAnimation(fig, update, frames=360, interval=1)
plt.legend()
plt.show()


# #rotate based on new origin
# def rotate_point(point, origin, roll, pitch, yaw):
#     #translate point to origin
#     point = point - origin
#     #rotate point
#     point = rot3d(roll, pitch, yaw) @ point
#     #translate point back to original position
#     point = point + origin
#     return point

# #example of rotating based on new origin
# point = np.array([5, 0, 0])
# origin = np.array([0, 0, 0])
# roll = np.deg2rad(45)
# pitch = np.deg2rad(45)
# yaw = np.deg2rad(0)

# rotated_point = rotate_point(point, origin, roll, pitch, yaw)
# print("new point: ", rotated_point)

# #animate the rotation of a point based on new origin
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# #set the limits of the plot
# ax.set_xlim(-10, 10)
# ax.set_ylim(-10, 10)
# ax.set_zlim(-10, 10)

# #set the labels of the plot
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')

# #set limits
# limit = 100
# #set the initial point
# point = np.array([1, 0, 0])

# #set the origin
# origin = np.array([0, 0, 0])

# #plot the initial point
# ax.plot(point[0], point[1], point[2], 'ro', label='point')
# ax.plot(origin[0], origin[1], origin[2], 'o', label='origin')

# #set the initial rotation angles
# roll = np.deg2rad(0)
# pitch = np.deg2rad(0)
# yaw = np.deg2rad(0)

# #function to update the plot
# def update(i):
#     global roll, pitch, yaw
#     roll = np.deg2rad(i)
#     pitch = np.deg2rad(i)
#     yaw = np.deg2rad(0)
#     rotated_point = rotate_point(point, origin, roll, pitch, yaw)
#     ax.plot(rotated_point[0], rotated_point[1], rotated_point[2], 'ro')

#     #update title of the plot based on current rotation angles
#     plt.title("roll: " + str(np.rad2deg(roll)) + " pitch: " + str(np.rad2deg(pitch)) + " yaw: " + str(np.rad2deg(yaw)))

#     return ax

# #animate the plot
# anim = animation.FuncAnimation(fig, update, frames=360, interval=1)
# plt.legend()
# plt.show()

