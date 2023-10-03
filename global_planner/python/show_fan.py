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

if __name__ == '__main__':


    # load rcs values
    # get pwd and append info/rcs_hash.csv
    pwd = os.getcwd()
    info_dir = 'info/'
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
    start_position = PositionVector(50,5,5)
    goal_position = PositionVector(10,90,5)
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
    

    # obs_positions = []
    obs_list = []
    for pos in obs_positions:
        obs_position = PositionVector(pos[0], pos[1], pos[2])
        radius_obs_m = 5
        some_obstacle = Obstacle(obs_position, radius_obs_m)
        obs_list.append(some_obstacle)
        grid.insert_obstacles(some_obstacle)

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
    radars = [radar1]

    weight_list = [0, 5, 10, 15]
    paths = []
    
    sparse_astar = SparseAstar(grid, 2, True, rcs_hash, 
                               radars, weight_list[0], 
                               max_rcs_val)

    sparse_astar.init_nodes()
    sparse_astar.update_radar_weight(weight_list[0])
    path, open_set = sparse_astar.search()

    import pickle as pkl
    with open('data_analysis/open_set.pkl', 'wb') as f:
        pkl.dump(open_set, f)

    #dump path
    with open('data_analysis/path.pkl', 'wb') as f:
        pkl.dump(path, f)




