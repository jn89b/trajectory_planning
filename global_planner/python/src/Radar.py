
import numpy as np

from src.PositionVector import PositionVector, rotation_z, \
    rotation_x, rotation_y

from src.Config.radar_config import radar_inputs
from src.Raytrace import fast_voxel_algo, fast_voxel_algo3D, another_fast_voxel


class Radar():
    def __init__(self, radar_params:dict) -> None:
        
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

        self.azmith_angle_rad = np.deg2rad(self.azmith_angle_dg)
        self.elevation_angle_rad = np.deg2rad(self.elevation_angle_dg)

        self.compute_lat_max_fov()
        self.compute_vert_max_fox()

        self.detection_voxels = []
        self.detection_positions = []


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
            self.pos.x + self.radar_range_m*np.cos(self.elevation_angle_rad+(self.max_fov_rad/2)),
            self.pos.y + self.radar_range_m*np.sin(self.elevation_angle_rad+(self.max_fov_rad/2)),
            self.pos.z + self.radar_range_m*np.cos(self.elevation_angle_rad+(self.max_fov_rad/2))
        )

        self.vert_fov_low_pos = PositionVector(
            self.pos.x + self.radar_range_m*np.cos(self.elevation_angle_rad-(self.max_fov_rad/2)),
            self.pos.y + self.radar_range_m*np.sin(self.elevation_angle_rad-(self.max_fov_rad/2)),
            self.pos.z + self.radar_range_m*np.cos(self.elevation_angle_rad-(self.max_fov_rad/2))
        )

        self.vert_fov_upp_rad = self.elevation_angle_rad + (self.max_fov_rad/2)
        self.vert_fov_low_rad = self.elevation_angle_rad - (self.max_fov_rad/2)

    def get_obs_within_fov(self) -> list:
        """returns obstacles within fov"""
        return []
    
    def compute_fov_cells_2d(self, obs_list=[]) -> list:
        """
        returns the cells that are within the radar fov
        in 2d scale
        """
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
            self.detection_voxels.extend(bearing_rays)

        return self.detection_voxels

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
        print("min azmith", min_lat_dg)
        print("max azmith", max_lat_dg)
        print("min elevation", min_vert_dg)
        print("max elevation", max_vert_dg)

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

                self.detection_positions.append((r_max_x, r_max_y, r_max_z))
                
                bearing_rays = another_fast_voxel(self.pos.x , self.pos.y, self.pos.z,
                                            r_max_x, r_max_y, r_max_z, obs_list)
                
                self.detection_voxels.extend(bearing_rays)

        print(len(azmith_bearing_dgs)*len(elevation_bearing_dgs))
        print("total length" , len(self.detection_voxels))

        return self.detection_voxels


    def compute_preliminary_prob_inst_detect(self) -> float:
        """computes the preliminary probability of 
        instantaneous detection without consideration of RCS"""




if __name__ == "__main__":
    pass