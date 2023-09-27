import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import pickle as pkl
import math as m
import plotly.graph_objects as go
import plotly.express as px

from src.PositionVector import PositionVector, rotation_z, \
    rotation_x, rotation_y
from src.Radar import Radar
from src.Grid import Grid, FWAgent
from src.Radar import Radar
from src.Obstacle import Obstacle
from src.SparseAstar import SparseAstar

from src.Config.radar_config import radar_inputs
from src.Raytrace import fast_voxel_algo, fast_voxel_algo3D, another_fast_voxel

class NettedRadarSys():
    def __init__(self, radar_list:list) -> None:
        self.radars = radar_list

    def get_voxels(self, detection_info:tuple, vox_step:int=100, 
                   radar_position:list=[0,0,0]) -> list:
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
        # print(detection_info)
        for k,v in detection_info.items():
            pos = v[1]
            voxels.append([pos.x, pos.y, pos.z])
            voxel_vals.append(v[0])

        voxel_x = []
        voxel_y = []
        voxel_z = []
        voxel_vals = []
        for i, voxel in enumerate(voxels):
            if i % vox_step == 0:
                voxel_x.append(voxel[0])
                voxel_y.append(voxel[1])
                voxel_z.append(voxel[2])
                distance = m.dist(voxel, radar_position)
                voxel_vals.append(distance)

        radar_info['voxel_x'] = voxel_x
        radar_info['voxel_y'] = voxel_y
        radar_info['voxel_z'] = voxel_z
        radar_info['voxel_vals'] = voxel_vals


        return radar_info

    def get_visual_scatter_radar(self, radar_info:dict, radar_index:int=0):
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
        name='Radar ' + str(radar_index),
        marker=dict(
            color=radar_info['voxel_vals'],
            colorscale='Viridis',
            # color_discrete_sequence=px.colors.qualitative.Plotly,
            size=3,
            opacity=0.1,
            colorbar=dict(
                title='Range From Radar',
                x=0)
            )
        )

        return voxel_visualizer

with open ('single_radar_param.pickle', 'rb' ) as file:
    radars_controlled = pkl.load(file)

# with open ('radar_cells.pickle', 'rb' ) as file:
#     radar_cells = pkl.load(file)

# for radar in radars_controlled:
    # print(radar.c1)
    # print(radar.pos.vec)

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
    
    obs_list = []
    for pos in obs_positions:
        obs_position = PositionVector(pos[0], pos[1], pos[2])
        radius_obs_m = 5
        some_obstacle = Obstacle(obs_position, radius_obs_m)
        obs_list.append(some_obstacle)
        grid.insert_obstacles(some_obstacle)


    plt.show()


    fig = go.Figure([])
    #%% 
    #PLOT
    network_radar_system = NettedRadarSys(radars_controlled)
    visualizer_list = []
    for radar in network_radar_system.radars:
        print(radar.pos.vec)
        data_info = network_radar_system.get_voxels(radar.detection_info, 5,radar.pos.vec)
        data_vis = network_radar_system.get_visual_scatter_radar(data_info)
        visualizer_list.append(data_vis)
        fig.add_trace(data_vis)
    # fig.update_layout(
    #     title={
    #         'text': "Single Radar with R^4 Range ",
    #         'y':0.9,
    #         'x':0.5,
    #         'xanchor': 'center',
    #         'yanchor': 'top'}
        
    #     )
    
    layout = go.Layout(
            scene=dict(
                xaxis=dict(
                    title='X Axis',
                    gridcolor='rgb(255, 255, 255)',
                    zerolinecolor='rgb(255, 255, 255)',
                    showbackground=True,
                    backgroundcolor='rgb(230, 230,230)'
                ),
                yaxis=dict(
                    title='Y Axis',
                    gridcolor='rgb(255, 255, 255)',
                    zerolinecolor='rgb(255, 255, 255)',
                    showbackground=True,
                    backgroundcolor='rgb(230, 230,230)'
                ),
                zaxis=dict(
                    title='Z Axis',
                    gridcolor='rgb(255, 255, 255)',
                    zerolinecolor='rgb(255, 255, 255)',
                    showbackground=True,
                    backgroundcolor='rgb(230, 230,230)'
                )
            ),
            title='Approach Vector to Target'
        )
    
    fig.show()
    fig.write_html("path_traj.html")