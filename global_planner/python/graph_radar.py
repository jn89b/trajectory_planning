from src.Raytrace import fast_voxel_algo, fast_voxel_algo3D
from src.Radar import Radar
from src.PositionVector import PositionVector
from src.Obstacle import Obstacle

import plotly.graph_objects as go

import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
    
    radar_pos = PositionVector(0, 0, 0)

    radar_params = {
        'pos': radar_pos,
        'azimuth_angle_dg': 90,
        'elevation_angle_dg': 90, #this is wrt to z axis
        'radar_range_m': 15,
        'max_fov_dg': 45,
        # 'max_vert_fov_dg': 5,
        'c1': -0.3,
        'c2': 1200
    }
    min_bound_x = -10
    max_bound_x = 10
    min_bound_y = -10
    max_bound_y = 10

    radar_example = Radar(radar_params)
    

    """to do"""
    # set obstacles, update this to feed in list of obstacles 
    # from the map, grab the ones within the radar fov and then 
    # compute the cost map 
    obs_x = 10 
    obs_y = 10
    obs_z = 10
    obs_position = PositionVector(obs_x, obs_y, obs_z)
    radius_obs_m = 5
    some_obstacle = Obstacle(obs_position, radius_obs_m)
    obs_list = []


    ### ------ compute rays ####### 
    # rays = radar_example.compute_fov_cells_2d(obs_list)

    # 2Dplot for sanity check
    # fig,axis = plt.subplots(1)
    # axis.set_xlim([min_bound_x, max_bound_x])
    # axis.set_ylim([min_bound_y, max_bound_y])

    # #plot circle based on radar position and radius
    # radar = plt.Circle((radar_example.pos.x, radar_example.pos.y), 1, 
    #                     color='r', fill=False, label='radar')

    # obs = plt.Circle((obs_x, obs_z), some_obstacle.radius_m,
    #                     color='g', fill=False, label='obstacle')

    # for square in rays:
    #     square = plt.Rectangle(square, 1, 1, color='red', fill=True)
    #     axis.add_artist(square)    

    # axis.add_artist(radar)
    # axis.add_artist(obs)
    # axis.legend()
    # plt.tight_layout()
    # plt.show()


    center_rays_3D = radar_example.compute_fov_cells_3d(obs_list)
    max_coords = np.max(center_rays_3D, axis=0)
    grid_shape = tuple(coord + 1 for coord in max_coords)
    print(grid_shape)
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
    ax.voxels(voxel_grid, facecolors=[1,0,0,0.3], edgecolor='w')
    ax.scatter(radar_example.lat_fov_low_pos.x, radar_example.lat_fov_low_pos.y, 
               radar_example.lat_fov_low_pos.z, c='b', s=100, label='fov_low')

    ax.scatter(radar_example.lat_fov_upp_pos.x, radar_example.lat_fov_upp_pos.y,
                radar_example.lat_fov_upp_pos.z, c='b', s=100, label='fov_upp')
    
    ax.scatter(radar_example.vert_fov_low_pos.x, radar_example.vert_fov_low_pos.y,
                radar_example.vert_fov_low_pos.z, c='g', s=100, label='vert_fov_low')
    
    ax.scatter(radar_example.vert_fov_upp_pos.x, radar_example.vert_fov_upp_pos.y,
                radar_example.vert_fov_upp_pos.z, c='g', s=100, label='vert_fov_upp')
    

    # for i, pos in enumerate(radar_example.detection_positions):
    #     # print every 10th point
    #     if i % 10 == 0:
    #         ax.scatter(pos[0], pos[1], pos[2], c='k', s=10)
        
    # Set axis labels
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    #set ratio of axes
    ax.set_aspect('equal')

    ax.legend()
    # plt.show()

    voxel_x = [x[0] for x in center_rays_3D]
    voxel_y = [x[1] for x in center_rays_3D]
    voxel_z = [x[2] for x in center_rays_3D]

    max_x_data = [x[0] for x in radar_example.detection_positions]
    max_y_data = [x[1] for x in radar_example.detection_positions]
    max_z_data = [x[2] for x in radar_example.detection_positions]

    voxel_data = go.Scatter3d(
        x=voxel_x,
        y=voxel_y,
        z=voxel_z,
        opacity=0.2,
        mode='markers',
        name='voxel_data',
        marker=dict(
            color='darkseagreen',
            size=3,
            line=dict(
                color='gold',
                width=0.1
            )
        )
    )


    marker_data = go.Scatter3d(
        x=max_x_data, 
        y=max_y_data, 
        z=max_z_data,  
        opacity=0.5, 
        mode='markers',
        name='detection_points',
        marker=dict(
            color='blueviolet',
            size=3,
            line=dict(
                color='gold',
                width=0.1
            )
        )
    )

    radar_data = go.Scatter3d(
        x=[radar_pos.x],
        y=[radar_pos.y],
        z=[radar_pos.z],
        opacity=1.0,
        mode='markers',
        name='radar',
        marker=dict(
            color='red',
            size=10,
            line=dict(
                color='gold',
                width=1.0
            )
        )
    )

    fig=go.Figure(data=marker_data)
    fig.add_trace(voxel_data)
    fig.add_trace(radar_data)

    fig.show()
    fig.write_html("radar_voxel.html")