import numpy as np
import matplotlib.pyplot as plt

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

if __name__ == '__main__':


    ## create agent
    start_position = PositionVector(50,15,5)
    goal_position = PositionVector(10,80,5)
    fw_agent_psi_dg = 90
    fw_agent = FWAgent(start_position, 0, fw_agent_psi_dg)
    fw_agent.vehicle_constraints(horizontal_min_radius_m=60, 
                                 max_climb_angle_dg=5)
    fw_agent.leg_m = 5

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
    radar_pos = PositionVector(15, 15, 0)
    radar_params = {
        'pos': radar_pos,
        'azimuth_angle_dg': 45,
        'elevation_angle_dg': 80, #this is wrt to z axis
        'radar_range_m': 80,    
        'max_fov_dg': 90, 
        'vert_max_fov_dg': 80,
        'c1': -0.1,
        'c2': 500,
        'grid': grid
    }

    radar1 = Radar(radar_params)
    detection_info = radar1.compute_fov_cells_3d(grid.obstacles)
    radars = [radar1]
    

    weight_list = [0, 10, 100, 1000]
    paths = []
    for weight in weight_list:
        sparse_astar = SparseAstar(grid, True, radars, 10)
        sparse_astar.init_nodes()
        sparse_astar.update_radar_weight(weight)
        returned_path = sparse_astar.search()
        sparse_astar.clear_sets()
        paths.append(returned_path)
        
    # plot for sanity check 
    fig, ax = plt.subplots()
    ax.set_xlim(grid.x_min_m, grid.x_max_m)
    ax.set_ylim(grid.y_min_m, grid.y_max_m)

    #plot path

    for i,wp_path in enumerate(paths):
        path_x = [x[0] for x in wp_path]
        path_y = [x[1] for x in wp_path]
        path_z = [x[2] for x in wp_path]
        ax.plot(path_x, path_y, label=str(weight_list[i]))
                
    #plot start and goal
    ax.plot(start_position.x, start_position.y, 'bo')
    ax.plot(goal_position.x, goal_position.y, 'bo')
    
    # plot with obstacles
    for obs in grid.obstacles:
        obs_image = plt.Circle((obs.position.x, obs.position.y), 
                               obs.radius_m, 
                               color='k', fill=False)
        ax.add_artist(obs_image)
    
    # plot radar fov
    radar_image = plt.Circle((radar_pos.x, radar_pos.y), 
                             radar1.radar_range_m, 
                             color='r', fill=False)
    
    ax.add_artist(radar_image)
    ax.legend()

    #plot radar pixels
    radar_values = []
    voxel_positions = set()
    for k,v in detection_info.items():
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
                                    color='r', fill=False)
        
        ax.add_artist(voxel_image)

    plt.show()    

    voxels = []
    voxel_vals = []
    for k,v in detection_info.items():
        pos = v[1]
        voxels.append([pos.x, pos.y, pos.z])
        voxel_vals.append(v[0]) 

    voxel_x = [x[0] for x in voxels]
    voxel_y = [x[1] for x in voxels]
    voxel_z = [x[2] for x in voxels]

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
            opacity=0.1,
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
    fig.write_html("path_traj.html")





    

    

    
    