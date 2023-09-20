"""
Define goal location

Define obstacles within area

Define P_density required 

Define Max effector range

From goal location:
    
    raytrace:
        
        during traversal compute power density 
        and store in array

        set p_dens_sum to sum of array
    
        Traverse until obstacle or max range
        if obstacle:
            then return 
        
        If max range and p_density < p_dens_sum:
            return the array locations 

Need to consider yaw angle of approach too 
            
"""

import numpy as np
from src.PositionVector import PositionVector

from src.Raytrace import fast_voxel_algo,  another_fast_voxel
from src.Obstacle import Obstacle
from src.MaxPriorityQueue import MaxPriorityQueue

import plotly.graph_objects as go
import plotly.express as px


class ApproachGoal():
    def __init__(self, 
                 goal_params:dict) -> None:
        

        self.pos = goal_params['pos']
        self.azmith_angle_dg = goal_params['azimuth_angle_dg']
        self.elevation_angle_dg = goal_params['elevation_angle_dg']

        self.max_effect_range_m = goal_params['max_effect_range_m']
        self.max_fov_dg = goal_params['max_fov_dg']
        self.max_fov_rad = np.deg2rad(self.max_fov_dg)

        self.vert_max_fov_dg = goal_params['vert_max_fov_dg']
        self.vert_max_fov_rad = np.deg2rad(self.vert_max_fov_dg)

        self.azmith_angle_rad = np.deg2rad(self.azmith_angle_dg)
        self.elevation_angle_rad = np.deg2rad(self.elevation_angle_dg)

        self.compute_lat_max_fov()
        self.compute_vert_max_fox()

        self.grid = goal_params['grid']
        self.effector_power = goal_params['effector_power']

        self.fov_dg_steps = goal_params['fov_dg_steps']
        self.vert_fov_dg_steps = goal_params['vert_fov_dg_steps']

        self.detection_info = {}

        self.detection_priority = MaxPriorityQueue()

    def compute_lat_max_fov(self):
        """computes the lateral bounds of the radar fov"""
        self.lat_fov_upp_pos = PositionVector(
            self.pos.x + self.max_effect_range_m*np.cos(self.azmith_angle_rad+(self.max_fov_rad/2)),
            self.pos.y + self.max_effect_range_m*np.sin(self.azmith_angle_rad+(self.max_fov_rad/2))
        )

        self.lat_fov_low_pos = PositionVector(
            self.pos.x + self.max_effect_range_m*np.cos(self.azmith_angle_rad-(self.max_fov_rad/2)),
            self.pos.y + self.max_effect_range_m*np.sin(self.azmith_angle_rad-(self.max_fov_rad/2))
        )

        self.lat_fov_upp_rad = self.azmith_angle_rad + (self.max_fov_rad/2)
        self.lat_fov_low_rad = self.azmith_angle_rad - (self.max_fov_rad/2)

    def compute_vert_max_fox(self):
        """computes the vertical bounds of the radar fov"""
        self.vert_fov_upp_pos = PositionVector(
            self.pos.x + self.max_effect_range_m*np.cos(self.elevation_angle_rad+(self.vert_max_fov_rad/2)),
            self.pos.y + self.max_effect_range_m*np.sin(self.elevation_angle_rad+(self.vert_max_fov_rad/2)),
            self.pos.z + self.max_effect_range_m*np.cos(self.elevation_angle_rad+(self.vert_max_fov_rad/2))
        )

        self.vert_fov_low_pos = PositionVector(
            self.pos.x + self.max_effect_range_m*np.cos(self.elevation_angle_rad-(self.vert_max_fov_rad/2)),
            self.pos.y + self.max_effect_range_m*np.sin(self.elevation_angle_rad-(self.vert_max_fov_rad/2)),
            self.pos.z + self.max_effect_range_m*np.cos(self.elevation_angle_rad-(self.vert_max_fov_rad/2))
        )

        self.vert_fov_upp_rad = self.elevation_angle_rad + (self.vert_max_fov_rad/2)
        self.vert_fov_low_rad = self.elevation_angle_rad - (self.vert_max_fov_rad/2)

    def get_obs_within_fov(self) -> list:
        """returns obstacles within fov"""
        return []
    
    def compute_fov_cells_2d(self, obs_list=[]) -> list:
        """
        returns the cells that are within the radar fov
        in 2d scale
        """
        detection_voxels = []
        fov_upp_dg = np.rad2deg(self.lat_fov_upp_rad)
        fov_low_dg = np.rad2deg(self.lat_fov_low_rad)

        if fov_low_dg > fov_upp_dg:
            max_dg = fov_low_dg
            min_dg = fov_upp_dg
        else:
            max_dg = fov_upp_dg
            min_dg = fov_low_dg

        azmith_bearing_dgs = np.arange(min_dg-1, max_dg+1)
        
        #could do this in parallel 
        for bearing in azmith_bearing_dgs:

            r_max_x = self.pos.x + self.max_effect_range_m*np.cos(np.deg2rad(bearing))
            r_max_y = self.pos.y + self.max_effect_range_m*np.sin(np.deg2rad(bearing))
            bearing_rays = fast_voxel_algo(self.pos.x , self.pos.y, 
                                        r_max_x, r_max_y, obs_list)
            detection_voxels.extend(bearing_rays)

        return detection_voxels

    def get_possible_approaches(self, required_pow_density:float, 
                                obs_list=[]) -> list:
        """returns """
        lat_fov_upp_dg = np.rad2deg(self.lat_fov_upp_rad)
        lat_fov_low_dg = np.rad2deg(self.lat_fov_low_rad)

        vert_fov_upp_dg = np.rad2deg(self.vert_fov_upp_rad)
        vert_fov_low_dg = np.rad2deg(self.vert_fov_low_rad)

        if lat_fov_low_dg > lat_fov_upp_dg:
            max_lat_dg = lat_fov_low_dg
            min_lat_dg = lat_fov_upp_dg
        else:
            max_lat_dg = lat_fov_upp_dg
            min_lat_dg = lat_fov_low_dg

        if vert_fov_low_dg > vert_fov_upp_dg:
            max_vert_dg = vert_fov_low_dg
            min_vert_dg = vert_fov_upp_dg
        else:
            max_vert_dg = vert_fov_upp_dg
            min_vert_dg = vert_fov_low_dg

        azmith_bearing_dgs = np.arange(min_lat_dg, max_lat_dg+1, self.fov_dg_steps)
        elevation_bearing_dgs = np.arange(min_vert_dg, max_vert_dg+1, self.vert_fov_dg_steps)

        overall_position_density_vals = []        
        for bearing in azmith_bearing_dgs:

            for elevation in elevation_bearing_dgs:

                r_max_x = self.pos.x + (self.max_effect_range_m*np.cos(np.deg2rad(bearing)) * \
                      np.sin(np.deg2rad(elevation)))
                
                r_max_y = self.pos.y + (self.max_effect_range_m*np.sin(np.deg2rad(bearing)) * \
                        np.sin(np.deg2rad(elevation)))
                
                r_max_z = self.pos.z + self.max_effect_range_m*np.cos(np.deg2rad(elevation))

                #round to nearest whole number
                r_max_x = round(r_max_x)
                r_max_y = round(r_max_y)
                r_max_z = round(r_max_z)
                
                bearing_rays = another_fast_voxel(self.pos.x , self.pos.y, self.pos.z,
                                            r_max_x, r_max_y, r_max_z, obs_list)
                
                start_ray_pos = PositionVector(int(bearing_rays[0][0]),
                                               int(bearing_rays[0][1]),
                                               int(bearing_rays[0][2]))

                end_ray_pos = PositionVector(int(bearing_rays[-1][0]),
                                            int(bearing_rays[-1][1]),
                                            int(bearing_rays[-1][2]))
                
                dist = np.linalg.norm(end_ray_pos.vec - start_ray_pos.vec)

                position_density_vals = []
                positions = []
                sum_power_density = 0
                for br in bearing_rays[15:]:
                    pos = PositionVector(br[0], br[1], br[2])
                    # if pos not in self.detection_info:
                    dist = np.linalg.norm(pos.vec - self.pos.vec)
                    p_density = self.compute_power_density(dist)

                    # self.detection_info[pos] = (p_density, pos)
                    sum_power_density += p_density
                    position_density_vals.append((p_density, pos))
                    positions.append((pos.x, pos.y, pos.z))

                self.detection_priority.push(positions, sum_power_density)

                if sum_power_density <= required_pow_density:
                    continue
                
                overall_position_density_vals.append((sum_power_density, 
                                                      position_density_vals))

        return overall_position_density_vals, self.detection_priority


    def compute_power_density(self, target_distance:float) -> float:
        """computes the power density and returns the value"""
        return self.effector_power / (target_distance * 4*np.pi)
    

    def get_best_approaches(self, num_approaches:int=5) -> list:
        """returns the n best approaches to the goal location """
        best_approaches = []

        for i in range(num_approaches+1):
            best_approaches.append(self.detection_priority.pop_max())

        return best_approaches
    

if __name__ == '__main__':
    origin_pos = PositionVector(10 , 10 ,10)
    goal_params = {
        'pos': origin_pos,
        'azimuth_angle_dg': 45,
        'elevation_angle_dg': 90,
        'max_effect_range_m': 50,
        'max_fov_dg': 300,
        'vert_max_fov_dg': 60,
        'grid': None,
        'effector_power': 50,
        'fov_dg_steps': 15,
        'vert_fov_dg_steps': 15
    }

    required_sum_power_density = 1.0
    
    obs_positions = [(45, 45, 10),
                     (25, 65, 10),
                     (55, 30, 10)]
    
    obs_list = []
    for pos in obs_positions:
        obs_position = PositionVector(pos[0], pos[1], pos[2])
        radius_obs_m = 5
        some_obstacle = Obstacle(obs_position, radius_obs_m)
        obs_list.append(some_obstacle)

    ag = ApproachGoal(goal_params)
    detection_info,detection_priority = ag.get_possible_approaches(required_sum_power_density, 
                                                obs_list)
    

    best_approaches = ag.get_best_approaches(5)

    highest_value_waypoint = detection_priority.pop_max()
    print("highest value waypoint: ", highest_value_waypoint)

    x_vals = []
    y_vals = []
    z_vals = []
    p_dense_vals = []
    for detec in detection_info:
        for vals in detec[1]:
            p_dense = vals[0]
            pos = vals[1]
            x_vals.append(pos.x)
            y_vals.append(pos.y)
            z_vals.append(pos.z)
            p_dense_vals.append(p_dense)

        
    voxel_data = go.Scatter3d(
        x=x_vals,
        y=y_vals,
        z=z_vals,
        mode='markers',
        name='voxel_data',
        marker=dict(
            color=p_dense_vals,
            colorscale='Viridis',
            # color_discrete_sequence=px.colors.qualitative.Plotly,
            size=3,
            opacity=0.1,
            colorbar=dict(
                title='Power Density',
                x=0)
        )
    )

    origin_data = go.Scatter3d(
        x=[origin_pos.x],
        y=[origin_pos.y],
        z=[origin_pos.z],
        mode='markers',
        name='origin',
        marker=dict(
            color='red',
            size=3,
            opacity=1.0
        )
    )

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

    fig = go.Figure(data=[voxel_data], layout=layout)
    fig.add_trace(origin_data)
    fig.show()
    fig.write_html("effector.html")
