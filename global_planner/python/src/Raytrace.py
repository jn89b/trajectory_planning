import numpy as np

from src.PositionVector import PositionVector

def fast_voxel_algo3D(x0:float, y0:float, z0:float, 
                      x1:float, y1:float, z1:float, 
                      obs_list=[]) ->list:
    
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    dz = abs(z1 - z0)

    x = int(np.floor(x0))
    y = int(np.floor(y0))
    z = int(np.floor(z0))

    if dx == 0:
        dt_dx = 1000000
    else:
        dt_dx = 1/ dx
    
    if dy == 0:
        dt_dy = 1000000
    else:
        dt_dy = 1/ dy
    
    if dz == 0:
        dt_dz = 1000000
    else:
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
        if obs_list:
            for obs in obs_list:
                pos = PositionVector(x,y)
                if obs.is_inside2D(pos,0.0) == True:
                    return cell_rays

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


def another_fast_voxel(x0:float, y0:float, z0:float, 
                       x1:float, y1:float, z1:float, 
                       obs_list=[]) -> list:
    direction_x = x1 - x0
    direction_y = y1 - y0
    direction_z = z1 - z0

    #init tmax
    min_x_bound = 0
    current_x_index = x0#np.ceil(x0 - min_x_bound)
    end_x_index = x1#np.ceil(x1 - min_x_bound)

    min_y_bound = 0
    current_y_index = y0#np.ceil(y0 - min_y_bound)
    end_y_index = y1#np.ceil(y1 - min_y_bound)

    min_z_bound = 0
    current_z_index = z0 #np.ceil(z0 - min_z_bound)
    end_z_index = z1#np.ceil(z1 - min_z_bound)

    #init steps
    if direction_x < 0:
        step_x = -1
        t_delta_x = step_x / direction_x
        t_max_x = (min_x_bound + current_x_index - x0) / direction_x

    elif direction_x > 0:
        step_x = 1
        t_delta_x = step_x / direction_x
        t_max_x = (min_x_bound + current_x_index - x0) / direction_x
    else:
        step_x = 0
        t_delta_x = np.inf
        t_max_x = np.inf

    if direction_y < 0:
        step_y = -1
        t_delta_y = step_y / direction_y
        t_max_y = (min_y_bound + current_y_index - y0) / direction_y
    elif direction_y > 0:
        step_y = 1
        t_delta_y = step_y / direction_y
        t_max_y = (min_y_bound + current_y_index - y0) / direction_y
    else:
        step_y = 0
        t_delta_y = np.inf
        t_max_y = np.inf

    if direction_z < 0:
        step_z = -1
        t_delta_z = step_z / direction_z
        t_max_z = (min_z_bound + current_z_index - z0) / direction_z
    elif direction_z > 0:
        step_z = 1
        t_delta_z = step_z / direction_z
        t_max_z = (min_z_bound + current_z_index - z0) / direction_z
    else:
        step_z = 0
        t_delta_z = np.inf
        t_max_z = np.inf

    rays_3D = []

    while (current_x_index != end_x_index or \
           current_y_index != end_y_index or \
           current_z_index != end_z_index):

        # if obs_list:
        for obs in obs_list:
            pos = PositionVector(current_x_index,current_y_index)
            if obs.is_inside2D(pos,0.0) == True:
                return rays_3D

        if t_max_x < t_max_y:
            if t_max_x < t_max_z:
                current_x_index += step_x
                t_max_x += t_delta_x
            else:
                current_z_index += step_z
                t_max_z += t_delta_z
        else:
            if t_max_y < t_max_z:
                current_y_index += step_y
                t_max_y += t_delta_y
            else:
                current_z_index += step_z
                t_max_z += t_delta_z

        rays_3D.append((current_x_index, current_y_index, current_z_index))
    
    return rays_3D




    
