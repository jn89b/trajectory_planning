import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

import seaborn as sns
import pickle as pkl

from src.PositionVector import PositionVector
from src.Grid import Grid, FWAgent
from src.Radar import Radar
from src.Obstacle import Obstacle
from src.SparseAstar import SparseAstar
from src.Config.radar_config import RADAR_AIRCRAFT_HASH_FILE
from src.DataContainer import SimDataContainer

"""
To do 
Make this small scale first

Create a simple c space with one radar
Have some obstacles placed within range

"""

import plotly.graph_objects as go
import plotly.express as px
import pickle as pkl


sns.set_palette("colorblind")

def load_pickle():
    """
    pass
    """
    with open('radar_params_obs.pickle', 'rb') as file:
        loaded_data = pkl.load(file)

    return loaded_data


if __name__ == '__main__':

    # load rcs values
    # get pwd and append info/rcs_hash.csv
    USE_SAVED_RADARS = True
    
    pwd = os.getcwd()
    info_dir = 'info/hash/'
    save_dir = 'figures/' + RADAR_AIRCRAFT_HASH_FILE
    rcs_file = info_dir+ RADAR_AIRCRAFT_HASH_FILE + '.csv'
    df = pd.read_csv(rcs_file, header=None)
    #get first column
    rpy_keys = df.iloc[:, 0]
    rcs_vals = df.iloc[:, 1]
    
    max_rcs_val = min(rcs_vals)

    #convert to dictionary
    rcs_hash = dict(zip(rpy_keys, rcs_vals))

    ## create agent
    
    start_position = PositionVector(10,60,5)
    goal_position = PositionVector(90,60)
    fw_agent_psi_dg = 90
    fw_agent = FWAgent(start_position, 0, fw_agent_psi_dg)
    fw_agent.vehicle_constraints(horizontal_min_radius_m=60, 
                                 max_climb_angle_dg=5)
    fw_agent.leg_m = 4

    fw_agent.set_goal_state(goal_position)

    ## create grid
    grid = Grid(fw_agent, 120, 120, 100, 5, 5, 0)
    # obs_positions = [(45, 45, 10),
    #                  (25, 65, 10),
    #                  (55, 30, 10)]  
    
    obs_positions = [(40,60,10)]
    obs_list = []
    for pos in obs_positions:
        obs_position = PositionVector(pos[0], pos[1], pos[2])
        radius_obs_m = 5
        some_obstacle = Obstacle(obs_position, radius_obs_m)
        obs_list.append(some_obstacle)
        grid.insert_obstacles(some_obstacle)


    if USE_SAVED_RADARS  == True:
        radars = load_pickle()
    else:
        # Set radar params
        radar_pos = PositionVector(15, 15, 0)
        radar_params = {
            'pos': radar_pos,
            'azimuth_angle_dg': 45,
            'elevation_angle_dg': 80, #this is wrt to z axis
            'radar_range_m': 80,    
            'max_fov_dg': 160, 
            'vert_max_fov_dg': 80,
            'c1': -0.29,
            'c2': 1200,
            'radar_fq_hz': 10000,
            'grid': grid
        }
        radar1 = Radar(radar_params)
        detection_info = radar1.compute_fov_cells_3d(grid.obstacles)
        radar_pos2 = PositionVector(70, 15, 0)
        radar_params = {
            'pos': radar_pos2,
            'azimuth_angle_dg': 135,
            'elevation_angle_dg': 80, #this is wrt to z axis
            'radar_range_m': 80,    
            'max_fov_dg': 160, 
            'vert_max_fov_dg': 80,
            'c1': -0.29,
            'c2': 1200,
            'radar_fq_hz': 10000,
            'grid': grid
        }
        radar2 = Radar(radar_params)
        detection_info2 = radar2.compute_fov_cells_3d(grid.obstacles)
        radars = [radar1, radar2]


    weight_list = [0, 5, 10, 15]
    paths = []
    for weight in weight_list:
        sparse_astar = SparseAstar(grid, 2, True, rcs_hash, radars, weight, max_rcs_val)
        sparse_astar.init_nodes()
        sparse_astar.update_radar_weight(weight)
        returned_path = sparse_astar.search()
        sparse_astar.clear_sets()
        paths.append(returned_path)
        
    # plot for sanity check 
    fig, ax = plt.subplots()
    ax.set_xlim(grid.x_min_m-20, grid.x_max_m+20)
    ax.set_ylim(grid.y_min_m-20, grid.y_max_m+20)

    # make a subplot with 3 rows
    fig2,ax2 = plt.subplots(3,1)
    fig3,ax3 = plt.subplots()
    fig4,ax4 = plt.subplots()

    #plot path
    for i,wp_path in enumerate(paths):
        path_x = [x[0] for x in wp_path]
        path_y = [x[1] for x in wp_path]
        path_z = [x[2] for x in wp_path]
        roll_dg = [x[3] for x in wp_path]
        pitch_dg = [x[4] for x in wp_path]
        yaw_dg = [x[5] for x in wp_path]
        f_cost = [x[6] for x in wp_path]
        radar_detect = [x[7] for x in wp_path]

        # print("path length", len(path_x))
        # print("radar detect length", len(radar_detect))

        ax.plot(path_x, path_y, '-o', label=str(weight_list[i]))
        
        ax2[0].plot(roll_dg, '-o', label=str(weight_list[i]))
        ax2[1].plot(pitch_dg,'-o', label=str(weight_list[i]))
        ax2[2].plot(yaw_dg, '-o', label=str(weight_list[i]))
        color = ax2[0].lines[-1].get_color()
        ax3.plot(f_cost, '-o', label=str(weight_list[i])+'rcs_value')
        ax4.plot(radar_detect,'-o', label=str(weight_list[i])+'radar detect')

    sim_data = SimDataContainer()
    sim_data.sim_results['paths'] = paths
    sim_data.sim_results['weights'] = weight_list
    sim_data.sim_results['radars'] = radars

    rcs_vals = []
    rcs_probs = []
    for i, wp_path in enumerate(paths):    
        rcs_vals.append([x[6] for x in wp_path])
        rcs_probs.append([x[7] for x in wp_path])

    sim_data.sim_results['rcs_vals'] = rcs_vals
    sim_data.sim_results['rcs_probs'] = rcs_probs
    sim_data.sim_results['obstacles'] = obs_list
    sim_data.sim_results['start_position'] = start_position.vec
    sim_data.sim_results['goal_position'] = goal_position.vec
    sim_data.sim_results['grid'] = grid
    
    pickle_dir = 'data_analysis/' + RADAR_AIRCRAFT_HASH_FILE + '.pkl'
    sim_data.pickle_data(save_dir + '.pkl')

    ax2[0].set_title("Roll")
    ax2[1].set_title("Pitch")
    ax2[2].set_title("Yaw")

    ax3.set_ylabel("RCS")
    ax3.legend()

    ax4.set_ylabel("Radar Detection")
    ax4.legend()

    #plot start and goal
    ax.plot(start_position.x, start_position.y, 'bo')
    ax.plot(goal_position.x,  goal_position.y,  'bo')

    # plot with obstacles
    for obs in grid.obstacles:
        obs_image = plt.Circle((obs.position.x, obs.position.y), 
                               obs.radius_m, 
                               color='k', fill=False)
        ax.add_artist(obs_image)
    
    for i, ra in enumerate(radars):

        # plot radar fov
        colors = ['r', 'gray']
        radar_image = plt.Circle((ra.pos.x, ra.pos.y), 
                                ra.radar_range_m, 
                                color=colors[i], fill=False)
        
        ax.plot(ra.pos.x, ra.pos.y, 'ro', label='radar')

        #plot radar pixels
        radar_values = []
        voxel_positions = set()
        for k,v in ra.detection_info.items():
            value = v[0]
            pos = v[1]
            if (pos.x, pos.y) in voxel_positions:
                continue

            voxel_positions.add((pos.x, pos.y))
            radar_values.append(value)
            
        #set color map
        for pos in voxel_positions:
            voxel_image = plt.Rectangle((pos[0], pos[1]),
                                        1, 1, 
                                        color=colors[i], fill=False,
                                        alpha=0.1)
            
            ax.add_artist(voxel_image)
        
    ax.add_artist(radar_image)
    ax.legend()

    # save plot
    fig.savefig(save_dir+"path.png")
    fig2.savefig(save_dir+"path_rpy.png")
    fig3.savefig(save_dir+"path_rcs.png")
    fig4.savefig(save_dir+"path_radar.png")

    #save as svg
    fig.savefig(save_dir+"path.svg")
    fig2.savefig(save_dir+"path_rpy.svg")
    fig3.savefig(save_dir+"path_rcs.svg")
    fig4.savefig(save_dir+"path_radar.svg")
    plt.show()    

    voxels = []
    voxel_vals = []
    for k,v in detection_info.items():
        pos = v[1]
        if pos.x == 11 and pos.y == 90:
            print(pos.x, pos.y, pos.z)
        voxels.append([pos.x, pos.y, pos.z])
        voxel_vals.append(v[0]) 

    voxel_step = 10
    voxel_x = []
    voxel_y = []
    voxel_z = []
    for i, voxel in enumerate(voxels):
        if i % voxel_step == 0:
            voxel_x.append(voxel[0])
            voxel_y.append(voxel[1])
            voxel_z.append(voxel[2])

    voxel_data = go.Scatter3d(
        x=voxel_x,
        y=voxel_y,
        z=voxel_z,
        mode='markers',
        name='voxel_data',
        marker=dict(
            color=voxel_vals,
            colorscale='Viridis',
            # color_discrete_sequence=px.colors.qualitative.Plotly,
            size=3,
            opacity=0.1
        )
    )

    path_data = go.Scatter3d(
        x=path_x,
        y=path_y,
        z=path_z,
        mode='lines',
        name='path',
        opacity=1.0,
        line={'width': 6}
    )

    fig = go.Figure([])
    fig.add_trace(voxel_data)
    fig.add_trace(path_data)
    fig.show()
    fig.write_html(save_dir+"path.html")




    

    

    
    