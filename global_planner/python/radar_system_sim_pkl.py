import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import pickle as pkl

from src.PositionVector import PositionVector
from src.Grid import Grid, FWAgent
from src.Radar import Radar
from src.Obstacle import Obstacle
from src.SparseAstar import SparseAstar

"""
To do 
Make this small scale first

Create a simple c space with one radar
Have some obstacles placed within range

"""

import plotly.graph_objects as go
import plotly.express as px

def get_voxels(detection_info:tuple, vox_step:int=100) -> list:
    """
    Pig: 
    Inputs: take in detection_info item 

    Processing is the implementation/ how do we make the sausage
    
    Sausage: 
    returns: radar information which is a dictionary that has 
    the following information
        x values
        y values 
        z values
        voxel_values 
    """
    radar_info = {
        'voxel_x': None,
        'voxel_y': None,
        'voxel_z': None,
        'voxel_vals': None
    }

    voxels = []
    voxel_vals = []
    for k,v in detection_info:
        pos = v[1]
        voxels.append([pos.x, pos.y, pos.z])
        voxel_vals.append(v[0])

    voxel_x = []
    voxel_y = []
    voxel_z = []
    for i, voxel in enumerate(voxels):
        if i % vox_step == 0:
            voxel_x.append(voxel[0])
            voxel_y.append(voxel[1])
            voxel_z.append(voxel[2])

    radar_info['voxel_x'] = voxel_x
    radar_info['voxel_y'] = voxel_y
    radar_info['voxel_z'] = voxel_z
    radar_info['voxel_vals'] = voxel_vals


    return radar_info

def get_visual_scatter_radar(radar_info:dict):
    """
    Pig:
    radar info x, y, z, vals

    Implementation:
    voxel_data = go.scatter
    
    Sausage:
    The scatter
    """
    voxel_visualizer = go.Scatter3d(
    x=radar_info['voxel_x'],
    y=radar_info['voxel_y'],
    z=radar_info['voxel_z'],
    mode='markers',
    name='voxel_data',
    marker=dict(
        color=radar_info['voxel_vals'],
        colorscale='Viridis',
        # color_discrete_sequence=px.colors.qualitative.Plotly,
        size=3,
        opacity=0.1
        )
    )

    return voxel_visualizer

pickle_plot_counter = 1
def pickle_plot(save_data):
    global pickle_plot_counter
    with open('radar_data'+str(pickle_plot_counter)+'.pickle', 'wb') as file:
        pkl.dump(save_data, file)
    pickle_plot_counter = pickle_plot_counter + 1


if __name__ == '__main__':

    ## create agent
    start_position = PositionVector(50,15,5)
    goal_position = PositionVector(10,80,5)
    fw_agent_psi_dg = 90
    fw_agent = FWAgent(start_position, 0, fw_agent_psi_dg)
    fw_agent.vehicle_constraints(horizontal_min_radius_m=60, 
                                 max_climb_angle_dg=5)
    fw_agent.leg_m = 4

    fw_agent.set_goal_state(goal_position)

    ## create grid
    grid = Grid(fw_agent, 100, 100, 100, 5, 5, 0)
    obs_positions = [(45, 45, 10),
                     (25, 65, 10),
                     (55, 30, 10)]
    
    # obs_list = []
    for pos in obs_positions:
        obs_position = PositionVector(pos[0], pos[1], pos[2])
        radius_obs_m = 5
        some_obstacle = Obstacle(obs_position, radius_obs_m)
        # obs_list.append(some_obstacle)
        grid.insert_obstacles(some_obstacle)

    # Set radar params
    radar_pos = PositionVector(50, 0, 0)
    radar_params = {
        'pos': radar_pos,
        'azimuth_angle_dg': 30,
        'elevation_angle_dg': 80, #this is wrt to z axis
        'radar_range_m': 30,
        'max_fov_dg': 120, 
        'vert_max_fov_dg': 80,
        'c1': -0.29,
        'c2': 1200,
        'radar_fq_hz': 10000,
        'grid': grid
    }

    radar_pos2 = PositionVector(-50, 0, 0)
    radar_params2 = {
        'pos': radar_pos2,
        'azimuth_angle_dg': 100,
        'elevation_angle_dg': 80, #this is wrt to z axis
        'radar_range_m': 30,    
        'max_fov_dg': 120, 
        'vert_max_fov_dg': 80,
        'c1': -0.29,
        'c2': 1200,
        'radar_fq_hz': 10000,
        'grid': grid
    }

    # INITIALIZE RADAR INSTANCES FROM RADAR CLASS
    radar1 = Radar(radar_params)
    radar2 = Radar(radar_params2)
    radars = [radar1, radar2]

    pickle_plot(radars)

    detection_info = radar1.compute_fov_cells_3d(grid.obstacles)
    detection_info2 = radar2.compute_fov_cells_3d(grid.obstacles)

    detect = [detection_info.items(), detection_info2.items()]

    plt.show()


    fig = go.Figure([])
    #PLOT

    visualizer_list = []
    for info in detect:
        data_info = get_voxels(info, 10)
        data_vis = get_visual_scatter_radar(data_info)
        visualizer_list.append(data_vis)
        fig.add_trace(data_vis)
        # pickle_plot(data_info)

    fig.show()
    fig.write_html("path_traj.html")