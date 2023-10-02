from src.PositionVector import PositionVector
from src.Grid import Grid, FWAgent
from src.Radar import Radar
from src.Obstacle import Obstacle

import numpy as np
import pickle as pkl


def pickle_radar_params(save_data):
    for radars in range(len(radar_list)):
        with open('radar_params_obs.pickle', 'wb') as file:
            pkl.dump(save_data, file)

# def pickle_radar_cells(save_data):
#     for cells in range(len(radar_cells)):
#         with open('radar_cells.pickle', 'wb') as file:
#             pkl.dump(save_data, file)

# CREATE GRID AND AGENT FOR RADAR POSITIONS
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
obs_positions = [(45, 45, 50),
                    (25, 65, 10),
                    (55, -30, 10),
                    (-40,80,25),
                    (-50,30,20),
                    (-60,-60,40),]


obs_positions = [(40,60,10)]
obs_list = []
for pos in obs_positions:
    obs_position = PositionVector(pos[0], pos[1], pos[2])
    radius_obs_m = 5
    some_obstacle = Obstacle(obs_position, radius_obs_m)
    obs_list.append(some_obstacle)
    grid.insert_obstacles(some_obstacle)


# # obs_list = []
# for pos in obs_positions:
#     obs_position = PositionVector(pos[0], pos[1], pos[2])
#     radius_obs_m = 5
#     some_obstacle = Obstacle(obs_position, radius_obs_m)
#     # obs_list.append(some_obstacle)
#     grid.insert_obstacles(some_obstacle)

# DEFINE RADAR INSTANCES

# radar_pos = PositionVector(10, 0, 0)
# radar_params = {
#     'pos': radar_pos,
#     'azimuth_angle_dg': 0,
#     'elevation_angle_dg': 50, #this is wrt to z axis
#     'radar_range_m': 120,
#     'max_fov_dg': 120, 
#     'vert_max_fov_dg': 80,
#     'c1': -0.29,
#     'c2': 1200,
#     'radar_fq_hz': 10000,
#     'grid': grid
# }

# radar_pos2 = PositionVector(-10, 0, 0)
# radar_params2 = {
#     'pos': radar_pos2,
#     'azimuth_angle_dg': 180,
#     'elevation_angle_dg': 50, #this is wrt to z axis
#     'radar_range_m': 120,    
#     'max_fov_dg': 120, 
#     'vert_max_fov_dg': 80,
#     'c1': -0.29,
#     'c2': 1200,
#     'radar_fq_hz': 10000,
#     'grid': grid
# }

# radar_pos3 = PositionVector(0, 10, 0)
# radar_params3 = {
#     'pos': radar_pos3,
#     'azimuth_angle_dg': 90,
#     'elevation_angle_dg': 50, #this is wrt to z axis
#     'radar_range_m': 120,    
#     'max_fov_dg': 120, 
#     'vert_max_fov_dg': 80,
#     'c1': -0.29,
#     'c2': 1200,
#     'radar_fq_hz': 10000,
#     'grid': grid
# }

# radar_pos4 = PositionVector(0, -10, 0)
# radar_params4 = {
#     'pos': radar_pos4,
#     'azimuth_angle_dg': 270,
#     'elevation_angle_dg': 50, #this is wrt to z axis
#     'radar_range_m': 120,
#     'max_fov_dg': 120, 
#     'vert_max_fov_dg': 80,
#     'c1': -0.29,
#     'c2': 1200,
#     'radar_fq_hz': 10000,
#     'grid': grid
# }

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

radar_params2 = {
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

radar2 = Radar(radar_params2)
detection_info2 = radar2.compute_fov_cells_3d(grid.obstacles)

radar1 = Radar(radar_params)
radar2 = Radar(radar_params2)
# radar3 = Radar(radar_params3)
# radar4 = Radar(radar_params4)
radar1_cells = radar1.compute_fov_cells_3d(grid.obstacles)
radar2_cells = radar2.compute_fov_cells_3d(grid.obstacles)
# radar3_cells = radar3.compute_fov_cells_3d(grid.obstacles)
# radar4_cells = radar4.compute_fov_cells_3d(grid.obstacles)
radar_list = [radar1, radar2]
# radar_cells = [radar1_cells]

pickle_radar_params(radar_list)
