"""
Fast voxel algorithm

https://github.com/cgyurgyik/fast-voxel-traversal-algorithm/blob/master/overview/FastVoxelTraversalOverview.md

https://github.com/cgyurgyik/fast-voxel-traversal-algorithm

http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
"""

import numpy as np
import matplotlib.pyplot as plt

class PositionVector():
    def __init__(self, x:float, y:float, z:float=0) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.vec = np.array([x, y, z])

min_bound_x = 0 
min_bound_y = 0
min_bound_z = 0

max_bound_y = 30
max_bound_x = 30
max_bound_z = 100

# implementation of the fast voxel algorithm
#assume we have radar point somewhere
radar_x = 5
radar_y = 5
radar_pos = PositionVector(radar_x, radar_y)

#we have radar azmith angle
azmith_angle_rad = np.deg2rad(30)
max_fov_rad = np.deg2rad(45)

# we queries our obstacles and get one 
obstacle_x = 10
obstacle_y = 10
radius_obstacle_m = 3
obs_pos = PositionVector(obstacle_x, obstacle_y)

# Right now compute the angle in the x-y plane
# get the vector ray expressed as in
# r = u + tv where u = origin point, convert to numpy to ma  
r_dist = np.linalg.norm(radar_pos.vec - obs_pos.vec)


#vector to center of radar to obs 
r_radar_to_obs = (obs_pos.vec - radar_pos.vec) 

#compute the position on the outside of the circle from the radar
r_outs_x = obstacle_x - r_dist*np.cos(azmith_angle_rad)
r_outs_y = obstacle_y - r_dist*np.sin(azmith_angle_rad)

r_north_x = obstacle_x - r_dist*np.cos(azmith_angle_rad + max_fov_rad/2)
r_north_y = obstacle_y - r_dist*np.sin(azmith_angle_rad + max_fov_rad/2)

r_south_x = obstacle_x - r_dist*np.cos(azmith_angle_rad - max_fov_rad/2)
r_south_y = obstacle_y - r_dist*np.sin(azmith_angle_rad - max_fov_rad/2)

# --------- begin fast voxel algorithm right here 2d for now ----------------------
# x0 = radar_x
# y0 = radar_y
# x1 = r_outs_x
# y1 = r_outs_y

"""this is using floating point path"""
# dx = abs(x1 - x0)
# dy = abs(y1 - y0)

# x = int(np.floor(x0))
# y = int(np.floor(y0))

# n = 1
# x_inc = 0   
# y_inc = 0
# error = 0

# if (dx == 0):
#     xinc = 0
#     error = float('inf')
# elif( x1 > x0):
#     x_inc = 1
#     n += int(np.floor(x1)) - x
#     error = (np.floor(x0) + 1 - x0) * dy
# else:
#     x_inc = -1
#     n += x - int(np.floor(x1))
#     error = (x0 - np.floor(x0)) * dy

# if (dy == 0):
#     y_inc = 0
#     error -= float('inf')
# elif (y1 > y0):
#     y_inc = 1
#     n += int(np.floor(y1)) - y
#     error -= (np.floor(y0) + 1 - y0) * dx
# else:
#     y_inc = -1
#     n += y - int(np.floor(y1))
#     error -= (y0 - np.floor(y0)) * dx

# print(n)
# cell_rays = []
# for i in range(n):
#     print(x,y)
#     cell_rays.append((x,y))
#     if (error > 0):
#         y += y_inc
#         error -= dx
#     else:
#         x += x_inc
#         error += dy
#     n += 1
#     print(n)
def fast_voxel_algo(x0:int, y0:int, x1:int, y1:int) -> list:
    """this uses integer math"""
    dx = int(abs(x1 - x0))
    dy = int(abs(y1 - y0))
    x = int(np.floor(x0))
    y = int(np.floor(y0))
    n = int(1 + dx + dy)
    if (x1 > x0):
        x_inc = 1
    else:
        x_inc = -1

    if (y1 > y0):
        y_inc = 1
    else:
        y_inc = -1

    error = dx - dy

    dx *= 2
    dy *= 2

    cell_rays = []
    for i in range(n):
        # print(x,y)
        cell_rays.append((x,y))
        if (error > 0):
            x += x_inc
            error -= dy
        else:
            y += y_inc
            error += dx
        n += 1
        # print(n)

    return cell_rays

# --------- end fast voxel algorithm right here ----------------------
center_rays = fast_voxel_algo(radar_x, radar_y, r_outs_x, r_outs_y)
north_rays = fast_voxel_algo(radar_x, radar_y, r_north_x, r_north_y)
south_rays = fast_voxel_algo(radar_x, radar_y, r_south_x, r_south_y)

cell_rays = []
cell_rays.extend(center_rays)
cell_rays.extend(fast_voxel_algo(radar_x, radar_y, r_north_x, r_north_y))
cell_rays.extend(fast_voxel_algo(radar_x, radar_y, r_south_x, r_south_y))

# plot for sanity check
fig,axis = plt.subplots(1)
axis.set_xlim([min_bound_x, max_bound_x])
axis.set_ylim([min_bound_y, max_bound_y])

#plot circle based on radar position and radius
radar = plt.Circle((radar_x, radar_y), 1, 
                    color='r', fill=False)

obs = plt.Circle((obstacle_x, obstacle_y), radius_obstacle_m,
                    color='g', fill=False)

outside_obs = plt.Circle((r_outs_x, r_outs_y), 1,
                    color='black', fill=False)

for square in cell_rays:
    square = plt.Rectangle(square, 1, 1, color='blue', fill=True)
    axis.add_artist(square)

center_x = [center_rays[i][0] for i in range(len(center_rays))]
center_y = [center_rays[i][1] for i in range(len(center_rays))]
center_ray_x = [center_rays[0][0],center_rays[-1][0]]
center_ray_y = [center_rays[0][1],center_rays[-1][1]]

north_ray_x = [north_rays[0][0],north_rays[-1][0]]
north_ray_y = [north_rays[0][1],north_rays[-1][1]]

south_ray_x = [south_rays[0][0],south_rays[-1][0]]
south_ray_y = [south_rays[0][1],south_rays[-1][1]]

axis.plot(center_ray_x, center_ray_y, '-o', color='black')
axis.plot(north_ray_x, north_ray_y, '-o', color='black')
axis.plot(south_ray_x, south_ray_y, '-o', color='black')


axis.add_artist(radar)
axis.add_artist(obs)
axis.add_artist(outside_obs)

plt.show()




