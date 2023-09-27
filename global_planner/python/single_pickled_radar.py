from src.PositionVector import PositionVector
from src.Grid import Grid, FWAgent
from src.Radar import Radar
from src.Obstacle import Obstacle

import numpy as np
import pickle as pkl


def pickle_radar_params(save_data):
    for radars in range(len(radar_list)):
        with open('single_radar_param.pickle', 'wb') as file:
            pkl.dump(save_data, file)

# def pickle_radar_cells(save_data):
#     for cells in range(len(radar_cells)):
#         with open('single_radar_cells.pickle', 'wb') as file:
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
obs_positions = [(45, 45, 10),
                 (25, 65, 10),
                 (55, 30, 10)]
# obs_positions = [(40,0,15),(0,40,15),(-40,0,15),(0,-40,15)]

# DEFINE RADAR INSTANCES

radar_pos = PositionVector(0, 0, 0)
radar_params = {
    'pos': radar_pos,
    'azimuth_angle_dg': 0,
    'elevation_angle_dg': 50, #this is wrt to z axis
    'radar_range_m': 120,
    'max_fov_dg': 120, 
    'vert_max_fov_dg': 80,
    'c1': -0.29,
    'c2': 1200,
    'radar_fq_hz': 10000,
    'grid': grid
}


radar1 = Radar(radar_params)
radar1_cells = radar1.compute_fov_cells_3d(grid.obstacles)

radar_list = [radar1]
radar_cells = [radar1_cells]

pickle_radar_params(radar_list)
# pickle_radar_cells(radar_cells)
