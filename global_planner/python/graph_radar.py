from src.Raytrace import fast_voxel_algo, fast_voxel_algo3D
from src.Radar import Radar
from src.PositionVector import PositionVector
from src.Obstacle import Obstacle

import plotly.graph_objects as go

import matplotlib.pyplot as plt
import numpy as np


#https://community.plotly.com/t/basic-3d-cylinders/27990/2

def cylinder(x, y, z, r, dz):
    """Create a cylindrical mesh located at x, y, z, with radius r and height dz"""
    center_z = np.linspace(0, dz, 15)
    theta = np.linspace(0, 2*np.pi, 15)
    theta_grid, z_grid = np.meshgrid(theta, center_z)
    x_grid = r * np.cos(theta_grid) + x
    y_grid = r * np.sin(theta_grid) + y
    z_grid = z_grid + z
    return x_grid, y_grid, z_grid

def circle(x, y, z, r):
    """Create a circular mesh located at x, y, z with radius r"""
    r_discr = np.linspace(0, r, 2)
    theta_discr = np.linspace(0, 2*np.pi, 15)
    r_grid, theta_grid = np.meshgrid(r_discr, theta_discr)
    x_circle = r_grid * np.cos(theta_grid) + x
    y_circle = r_grid * np.sin(theta_grid) + y
    z_circle = np.zeros_like(x_circle) + z
    return x_circle, y_circle, z_circle

if __name__ == '__main__':
    
    radar_pos = PositionVector(0, 0, 0)

    radar_params = {
        'pos': radar_pos,
        'azimuth_angle_dg': 45,
        'elevation_angle_dg': 30, #this is wrt to z axis
        'radar_range_m': 40,
        'max_fov_dg': 75,
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
    obs_x = 20 
    obs_y = 20
    obs_z = 10
    obs_position = PositionVector(obs_x, obs_y, obs_z)
    radius_obs_m = 5
    some_obstacle = Obstacle(obs_position, radius_obs_m)
    obs_list = [some_obstacle]

    center_rays_3D = radar_example.compute_fov_cells_3d(obs_list)

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
        mode='markers',
        name='voxel_data',
        marker=dict(
            color='blueviolet',
            size=3,
            opacity=0.1,
            line=dict(
                # color='gold',
                width=0.1
            )
        )
    )

    marker_data = go.Scatter3d(
        x=max_x_data, 
        y=max_y_data, 
        z=max_z_data,  
        opacity=0.3, 
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

    colorscale = [[0, '#636EFA'],
                [1, '#636EFA']]

    #plot obstacle 
    x_cyl, y_cyl, z_cyl = cylinder(obs_x, obs_y, 0, 
                                   radius_obs_m, obs_z)
    x_circle, y_circle, z_circle = circle(obs_x, obs_y, obs_z,
                                            radius_obs_m)
    
    top = go.Surface(x=x_cyl, y=y_cyl, z=z_cyl, 
                     colorscale='Earth',

                     opacity=1.0),
    walls = go.Surface(x=x_circle, y=y_circle, z=z_circle, opacity=0.5),
    # bottom = go.Surface(x=x_circle2, y=y_circle2, z=z_circle2, 
    #                     showscale=False, opacity=0.5),


    fig=go.Figure([
         go.Surface(x=x_cyl, y=y_cyl, z=z_cyl, 
                     colorscale='Earth', 
                     opacity=0.5),
        go.Surface(x=x_circle, y=y_circle, z=z_circle, opacity=0.5),
    ])
    fig.add_trace(marker_data)
    fig.add_trace(voxel_data)
    fig.add_trace(radar_data)
    # fig.add_trace(top)
    # fig.add_trace(walls)

    fig.show()
    fig.write_html("radar_voxel.html")