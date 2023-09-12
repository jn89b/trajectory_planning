"""
Fast voxel algorithm

https://github.com/cgyurgyik/fast-voxel-traversal-algorithm/blob/master/overview/FastVoxelTraversalOverview.md

https://github.com/cgyurgyik/fast-voxel-traversal-algorithm

http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html

https://www.redblobgames.com/articles/visibility/


To do:
- Ray trace with obstacles in 2D and 3D
- Ray trace with multiple obstacles in 2D and 3D
- 

"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 


class PositionVector():
    def __init__(self, x:float, y:float, z:float=0) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.vec = np.array([x, y, z])

class Obstacle():
    def __init__(self, position:PositionVector, radius:float) -> None:
        self.position = position
        self.radius = radius

    def is_inside2D(self, position:PositionVector, agent_radius:float) -> bool:
        """
        Check if position is inside obstacle
        """
        total_radius = self.radius + agent_radius

        dist = np.sqrt((position.x - self.position.x)**2 +
                          (position.y - self.position.y)**2)
        
        if dist <= total_radius:
            return True
        
        return False

min_bound_x = -30
min_bound_y = -30
min_bound_z = 0

max_bound_y = 30
max_bound_x = 30
max_bound_z = 100

# implementation of the fast voxel algorithm
#assume we have radar point somewhere
radar_x = 0
radar_y = 0
radar_pos = PositionVector(radar_x, radar_y)

#we have radar azmith angle
azmith_angle_rad = np.deg2rad(45)
max_fov_rad = np.deg2rad(90)
radar_range_m = 15

# we queries our obstacles and get one 
obstacle_x = 10
obstacle_y = 10
radius_obstacle_m = 3
obs_pos = PositionVector(obstacle_x, obstacle_y)
some_obs = Obstacle(obs_pos, radius_obstacle_m)
obs_list = [some_obs]

#### This is max position FOV ########
north_fov_angle_rd = azmith_angle_rad+(max_fov_rad/2)
south_fov_angle_rd = azmith_angle_rad-(max_fov_rad/2)

fov_north = PositionVector(
    radar_pos.x + radar_range_m*np.cos(azmith_angle_rad+(max_fov_rad/2)),
    radar_pos.y + radar_range_m*np.sin(azmith_angle_rad+(max_fov_rad/2))
)

fov_south = PositionVector(
    radar_pos.x + radar_range_m*np.cos(azmith_angle_rad-(max_fov_rad/2)),
    radar_pos.y + radar_range_m*np.sin(azmith_angle_rad-(max_fov_rad/2))
)

#### THIS IS FOV FOR OBSTACLES ####### 
#compute north and south side angle
los_angle_rad = np.arctan2(obs_pos.y - radar_pos.y,
                           obs_pos.x - radar_pos.x)
import numpy as np
north_los_rad = los_angle_rad + np.pi/2
south_los_rad = los_angle_rad - np.pi/2

print("los angle for obstacle rad", np.rad2deg(los_angle_rad))
print("north obstacle", np.rad2deg(north_los_rad))
print("south obstacle", np.rad2deg(south_los_rad))

# Right now compute the angle in the x-y plane
# get the vector ray expressed as in
# r = u + tv where u = origin point, convert to numpy to ma  
r_dist = np.linalg.norm(radar_pos.vec - obs_pos.vec)

#vector to center of radar to obs 
r_radar_to_obs = (obs_pos.vec - radar_pos.vec) 

#compute the position on the outside of the circle from the radar
r_outs_x = obstacle_x - radius_obstacle_m*np.cos(los_angle_rad)
r_outs_y = obstacle_y - radius_obstacle_m*np.sin(los_angle_rad)

r_north_x = obstacle_x + radius_obstacle_m*np.cos(north_los_rad)
r_north_y = obstacle_y + radius_obstacle_m*np.sin(north_los_rad)

r_south_x = obstacle_x +radius_obstacle_m*np.cos(south_los_rad)
r_south_y = obstacle_y + radius_obstacle_m*np.sin(south_los_rad)

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


def fast_voxel_algo3D(x0:float, y0:float, z0:float, 
                      x1:float, y1:float, z1:float, 
                      obs_list=[]) ->list:
    
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    dz = abs(z1 - z0)

    x = int(np.floor(x0))
    y = int(np.floor(y0))
    z = int(np.floor(z0))

    dt_dx = 1/ dx
    dt_dy = 1/ dy
    dt_dz = 1/ dz

    t = 0
    n = 1 
    t_next_horizontal = 0
    t_next_vertical = 0
    t_next_height = 0

    if (dx == 0):
        x_inc = 0
        t_next_horizontal = dt_dx 
    elif (x1 > x0):
        x_inc = 1
        n += int(np.floor(x1)) - x
        t_next_horizontal = (np.floor(x0) + 1 - x0) * dt_dx
    else:
        x_inc = -1
        n += x - int(np.floor(x1))
        t_next_horizontal = (x0 - np.floor(x0)) * dt_dx

    if (dy == 0):
        y_inc = 0
        t_next_vertical = dt_dy 
    elif (y1 > y0):
        y_inc = 1
        n += int(np.floor(y1)) - y
        t_next_vertical = (np.floor(y0) + 1 - y0) * dt_dy
    else:
        y_inc = -1
        n += y - int(np.floor(y1))
        t_next_vertical = (y0 - np.floor(y0)) * dt_dy

    if (dz == 0):
        z_inc = 0
        t_next_height = dt_dz 
    elif (z1 > z0):
        z_inc = 1
        n += int(np.floor(z1)) - z
        t_next_height = (np.floor(z0) + 1 - z0) * dt_dz
    else:
        z_inc = -1
        n += z - int(np.floor(z1))
        t_next_height = (z0 - np.floor(z0)) * dt_dz

    cell_rays = []
    for i in range(n):
        print(x,y,z)
        
        # if obs_list:
        #     for obs in obs_list:
        #         pos = PositionVector(x,y)
        #         if obs.is_inside2D(pos,0.0) == True:
        #             return cell_rays

        cell_rays.append((x,y,z))
        # dy < dx then I need to go up since 
        if (t_next_horizontal < t_next_vertical):
            if (t_next_horizontal < t_next_height):
                x += x_inc
                t = t_next_horizontal
                t_next_horizontal += dt_dx
            else:
                z += z_inc
                t = t_next_height
                t_next_height += dt_dz
        else:
            if (t_next_vertical < t_next_height):
                y += y_inc
                t = t_next_vertical
                t_next_vertical += dt_dz
            else:
                z += z_inc
                t = t_next_height
                t_next_height += dt_dz
            
        # print(n)

    return cell_rays


def fast_voxel_algo(x0:int, y0:int, x1:int, y1:int, obs_list=[]) -> list:
    """this uses integer math"""
    dx = int(abs(x1 - x0))
    dy = int(abs(y1 - y0))
    #dz = int(abs(z1 - z0))

    x = int(np.floor(x0))
    y = int(np.floor(y0))
    #z = int(np.floor(z0))

    n = int(1 + dx + dy)
    if (x1 > x0):
        x_inc = 1
    else:
        x_inc = -1

    if (y1 > y0):
        y_inc = 1
    else:
        y_inc = -1
        
    error = dx - dy #- dz

    dx *= 2
    dy *= 2
    #dz *= 2

    cell_rays = []
    for i in range(n):
        # print(x,y)
        
        if obs_list:
            for obs in obs_list:
                pos = PositionVector(x,y)
                if obs.is_inside2D(pos,0.0) == True:
                    return cell_rays
                
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

## pseudocode:

# define min and max bearings
# define cell_detection arrays
rays = []
north_los_dg = np.rad2deg(north_fov_angle_rd)
south_los_dg = np.rad2deg(south_fov_angle_rd)

if south_los_dg > north_los_dg:
    max_dg = south_los_dg
    min_dg = north_los_dg
else:
    max_dg = north_los_dg
    min_dg = south_los_dg

# from min to max bearing: 
    # fast voxel from init bearing step
    #   inside fast voxel check if it hits obstacle 
    #   if so return 
    # append to cell_detection_arrays
    # go to next bearing step
azmith_bearing_dgs = np.arange(min_dg-1, max_dg+1)

#could do this in parallel 
for bearing in azmith_bearing_dgs:

    r_max_x = radar_pos.x + radar_range_m*np.cos(np.deg2rad(bearing))
    r_max_y = radar_pos.y + radar_range_m*np.sin(np.deg2rad(bearing))
    bearing_rays = fast_voxel_algo(radar_pos.x , radar_pos.y, 
                                   r_max_x, r_max_y, obs_list)
    rays.extend(bearing_rays)

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
                    color='r', fill=False, label='radar')

obs = plt.Circle((obstacle_x, obstacle_y), radius_obstacle_m,
                    color='g', fill=False, label='obstacle')

outside_obs = plt.Circle((r_outs_x, r_outs_y), 1,
                    color='black', fill=False)

for square in rays:
    square = plt.Rectangle(square, 1, 1, color='red', fill=True)
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
axis.plot(north_ray_x, north_ray_y, '-o', color='black', label='north obs')
axis.plot(south_ray_x, south_ray_y, '-o', color='black', label='south obs')

axis.scatter(fov_north.x, fov_north.y, c='g', label='north fov')
axis.scatter(fov_south.x, fov_south.y, c='g', label='south fov')

axis.add_artist(radar)
axis.add_artist(obs)
axis.add_artist(outside_obs)
axis.legend()
plt.tight_layout()
plt.show()



#### Testing 3D voxel
# def fast_voxel_algo3D(x0:float, y0:float, z0:float, 
#                       x1:float, y1:float, z1:float, 
#                       obs_list=[]) ->list:

center_rays_3D = fast_voxel_algo3D(radar_x, 
                                   radar_y, 
                                   0, 
                                   10, 
                                   10, 
                                   10)

# Determine the dimensions of the grid based on the maximum coordinates
max_coords = np.max(center_rays_3D, axis=0)
grid_shape = tuple(coord + 1 for coord in max_coords)

# Create a 3D array to represent the voxel grid
voxel_grid = np.zeros(grid_shape)

# Set the voxel values to 1 for the specified coordinates
for coord in center_rays_3D:
    voxel_grid[coord] = 1


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter3D(radar_pos.x, radar_pos.y, radar_pos.z , s=100, label='radar')
ax.scatter3D(10, 10, 10 , s=100, c='k', label='random point')
# Use the `voxels` function to plot the voxel grid
ax.voxels(voxel_grid, facecolors=[1,0,0,0.3], edgecolor='k')

# Set axis labels
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

ax.legend()
plt.show()