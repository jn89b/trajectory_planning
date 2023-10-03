
import numpy as np

from src.PositionVector import PositionVector, rotation_z, \
    rotation_x, rotation_y

from src.Config.radar_config import radar_inputs
from src.Raytrace import fast_voxel_algo, fast_voxel_algo3D, another_fast_voxel


class Radar():
    def __init__(self, 
                 radar_params:dict) -> None:
        
        #check if all params are present
        for key in radar_params.keys():
            if key not in radar_inputs.keys():
                raise ValueError(f"Missing parameter {key} in radar_params")
            
        self.pos = radar_params['pos']
        self.azmith_angle_dg = radar_params['azimuth_angle_dg']
        self.elevation_angle_dg = radar_params['elevation_angle_dg']

        self.radar_range_m = radar_params['radar_range_m']
        self.max_fov_dg = radar_params['max_fov_dg']
        self.max_fov_rad = np.deg2rad(self.max_fov_dg)
        self.radar_fq_hz = radar_params['radar_fq_hz']

        self.vert_max_fov_dg = radar_params['vert_max_fov_dg']
        self.vert_max_fov_rad = np.deg2rad(self.vert_max_fov_dg)

        self.azmith_angle_rad = np.deg2rad(self.azmith_angle_dg)
        self.elevation_angle_rad = np.deg2rad(self.elevation_angle_dg)

        self.c1 = radar_params['c1']
        self.c2 = radar_params['c2']

        self.compute_lat_max_fov()
        self.compute_vert_max_fox()

        self.grid = radar_params['grid']

        self.detection_info = {}

    def compute_lat_max_fov(self):
        """computes the lateral bounds of the radar fov"""
        self.lat_fov_upp_pos = PositionVector(
            self.pos.x + self.radar_range_m*np.cos(self.azmith_angle_rad+(self.max_fov_rad/2)),
            self.pos.y + self.radar_range_m*np.sin(self.azmith_angle_rad+(self.max_fov_rad/2))
        )

        self.lat_fov_low_pos = PositionVector(
            self.pos.x + self.radar_range_m*np.cos(self.azmith_angle_rad-(self.max_fov_rad/2)),
            self.pos.y + self.radar_range_m*np.sin(self.azmith_angle_rad-(self.max_fov_rad/2))
        )

        self.lat_fov_upp_rad = self.azmith_angle_rad + (self.max_fov_rad/2)
        self.lat_fov_low_rad = self.azmith_angle_rad - (self.max_fov_rad/2)

    def compute_vert_max_fox(self):
        """computes the vertical bounds of the radar fov"""
        self.vert_fov_upp_pos = PositionVector(
            self.pos.x + self.radar_range_m*np.cos(self.elevation_angle_rad+(self.vert_max_fov_rad/2)),
            self.pos.y + self.radar_range_m*np.sin(self.elevation_angle_rad+(self.vert_max_fov_rad/2)),
            self.pos.z + self.radar_range_m*np.cos(self.elevation_angle_rad+(self.vert_max_fov_rad/2))
        )

        self.vert_fov_low_pos = PositionVector(
            self.pos.x + self.radar_range_m*np.cos(self.elevation_angle_rad-(self.vert_max_fov_rad/2)),
            self.pos.y + self.radar_range_m*np.sin(self.elevation_angle_rad-(self.vert_max_fov_rad/2)),
            self.pos.z + self.radar_range_m*np.cos(self.elevation_angle_rad-(self.vert_max_fov_rad/2))
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

            r_max_x = self.pos.x + self.radar_range_m*np.cos(np.deg2rad(bearing))
            r_max_y = self.pos.y + self.radar_range_m*np.sin(np.deg2rad(bearing))
            bearing_rays = fast_voxel_algo(self.pos.x , self.pos.y, 
                                        r_max_x, r_max_y, obs_list)
            detection_voxels.extend(bearing_rays)

        return detection_voxels

    def compute_fov_cells_3d(self, obs_list=[]) -> list:
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

        azmith_bearing_dgs = np.arange(min_lat_dg, max_lat_dg+1)
        elevation_bearing_dgs = np.arange(min_vert_dg, max_vert_dg+1)

        max_radar_val = 0
        for bearing in azmith_bearing_dgs:
            for elevation in elevation_bearing_dgs:

                r_max_x = self.pos.x + (self.radar_range_m*np.cos(np.deg2rad(bearing)) * \
                      np.sin(np.deg2rad(elevation)))
                
                r_max_y = self.pos.y + (self.radar_range_m*np.sin(np.deg2rad(bearing)) * \
                        np.sin(np.deg2rad(elevation)))
                
                r_max_z = self.pos.z + self.radar_range_m*np.cos(np.deg2rad(elevation))

                #round to nearest whole number
                r_max_x = round(r_max_x)
                r_max_y = round(r_max_y)
                r_max_z = round(r_max_z)
                
                bearing_rays = another_fast_voxel(self.pos.x , self.pos.y, self.pos.z,
                                            r_max_x, r_max_y, r_max_z, obs_list)
                
                for br in bearing_rays:
                    pos = PositionVector(int(br[0]), int(br[1]), int(br[2]))
                    pos_idx = self.grid.convert_position_to_index(pos)
                    if pos_idx not in self.detection_info:
                        dist = np.linalg.norm(pos.vec - self.pos.vec)
                        detect_val = self.compute_preliminary_prob_inst_detect(dist)
                        self.detection_info[pos_idx] = (detect_val, pos)
                        if detect_val > max_radar_val:
                            max_radar_val = detect_val

        #normalize radar values
        for k,v in self.detection_info.items():
            self.detection_info[k] = (v[0]/max_radar_val, v[1])

        #print size of detection info
        
        return self.detection_info


    def compute_preliminary_prob_inst_detect(self, dist:float) -> float:
        """
        computes the preliminary probability of 
        instantaneous detection without consideration of RCS
        """
        #return 1/(1 +(self.c2*dist**4)**self.c1) 
        # return 1 - (dist/self.radar_range_m)
        return 1 
    
    def compute_prob_detect(self, dist, rcs_val:float, is_linear_db:bool=False) -> float:
        """
        Computes the probability of detection for the radar
        """
        if is_linear_db == True:
            linear_db = rcs_val
        else:
            linear_db = 10**(rcs_val/10) 

        radar_prob_detection = 1/(1 +(self.c2* np.power(dist,4) / linear_db)**self.c1)
        probability_detection = 1- pow(radar_prob_detection , self.radar_fq_hz)

        return probability_detection

if __name__ == "__main__":
    pass