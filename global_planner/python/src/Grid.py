import numpy as np
import math as m
from src.PositionVector import PositionVector
from src.Obstacle import Obstacle


class FWAgent():
    def __init__(self, position:PositionVector, 
                 theta_dg:float=0, psi_dg:float=0, leg_m:float=50) -> None:
        self.position = position
        self.theta_dg = theta_dg #pitch anglemoves.append([next_x, next_y, next_z])
        self.psi_dg = psi_dg # this is azmith heading
        self.radius_m = 5 #radius of aircraft meters
        self.leg_m = leg_m #leg length in meters

    def set_current_state(self, position:PositionVector, 
                  theta_dg:float=0, psi_dg:float=0) -> None:
        """
        update state of aircraft
        """
        self.position = position
        self.theta_dg = theta_dg
        self.psi_dg = psi_dg

    def set_goal_state(self, position:PositionVector) -> None:
        self.goal_position = position

    def vehicle_constraints(self, horizontal_min_radius_m:float=35, 
                            max_climb_angle_dg:float=10, 
                            max_psi_turn_dg:float=45) -> None:
        """
        horizontal_min_turn = v^2/ (g * tan(phi)) where theta is bank angle
        """
        self.horizontal_min_radius_m = horizontal_min_radius_m
        self.max_climb_angle_dg = max_climb_angle_dg
        self.max_psi_turn_dg = max_psi_turn_dg


    def get_moves(self, position:PositionVector, curr_psi_dg:float,
                  step_psi=5) -> list:
        """
        based on current position and heading get all 
        possible forward moves
        """
        
        moves = []
        ac_max_psi_dg = self.max_psi_turn_dg

        for i in range(0,ac_max_psi_dg+step_psi, step_psi):
            next_psi_dg = curr_psi_dg + i
            if next_psi_dg > 360:
                next_psi_dg -= 360
            if next_psi_dg < 0:
                next_psi_dg += 360

            psi_rad = np.deg2rad(next_psi_dg)
            next_x = position.x + round(self.leg_m*(np.cos(psi_rad)))
            next_y = position.y + round(self.leg_m*(np.sin(psi_rad)))
            for z in range(-1,2,1):
                next_z = position.z + z
                moves.append([next_x, next_y, next_z])

        for i in range(0,ac_max_psi_dg+step_psi, step_psi):
            next_psi_dg = curr_psi_dg - i
            if next_psi_dg > 360:
                next_psi_dg -= 360
            if next_psi_dg < 0:
                next_psi_dg += 360        

            psi_rad = np.deg2rad(next_psi_dg)
            next_x = position.x + round(self.leg_m*(np.cos(psi_rad)))
            next_y = position.y + round(self.leg_m*(np.sin(psi_rad)))
            for z in range(-1,2,1):
                next_z = position.z + z
                moves.append([next_x, next_y, next_z])
        return moves
    

class Grid():
    """
    Set grid size based on agent constraints
    For now consider in units of meters 
    """
    def __init__(self, 
                 agent:FWAgent, 
                 x_max_m:float=1000, 
                 y_max_m:float=1000,
                 z_max_m:float=1000,
                 x_min_m:float=0,
                 y_min_m:float=0,
                 z_min_m:float=0,
                 offset_x:float=0,
                 offset_y:float=0,
                 offset_z:float=0) -> None:
        
        self.agent = agent
        self.x_min_m = x_min_m
        self.y_min_m = y_min_m
        self.z_min_m = z_min_m

        self.x_max_m = x_max_m
        self.y_max_m = y_max_m
        self.z_max_m = z_max_m
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.offset_z = offset_z

        self.set_grid_size()
        self.obstacles = []

    def get_grid_size(self) -> tuple:
        return (self.sx, self.sy, self.sz)

    def insert_obstacles(self, obstacle:Obstacle) -> None:
        self.obstacles.append(obstacle)

    def map_position_to_grid(self, position:PositionVector, 
                             direction_vector:PositionVector) -> PositionVector:
        """
        sets the position up or down based on set_high
        Note based on unit vector from start to goal 
        
        Check modulus if 0 if so then we are on the grid 
        and snap to grid
        
        if i direction is positive then get ceiling 
        if i direction is negative then get floor

        """
        if position.x % self.sx == 0 and position.y % self.sy == 0 and \
            position.z % self.sz == 0:
            return position
        
        if direction_vector.x > 0:
            x_round = self.sx * m.ceil(position.x/self.sx)
        else:
            x_round = self.sx * m.floor(position.x/self.        sx)
            
        if direction_vector.y > 0:
            y_round = self.sy * m.ceil(position.y/self.sy)
        else:
            y_round = self.sy * m.floor(position.y/self.sy)
        
        if direction_vector.z > 0:
            z_round = self.sz * m.ceil(position.z/self.sz)
        else:
            z_round = self.sz * m.floor(position.z/self.sz)
        
        rounded_position = PositionVector(x_round, y_round, z_round)
        rounded_position.set_position(x_round, y_round, z_round)
        return rounded_position

    def set_grid_size(self) -> None:
        """
        From paper set grid size based on agent constraints
        sx = size of grid in x direction
        sy = size of grid in y direction
        sz = size of grid in z direction
        """
        #round up 
        # self.sx = m.ceil(2/3 * self.agent.horizontal_min_radius_m)
        # self.sy = self.sx
        # self.sz = m.ceil(self.sx * 
        #                 np.tan(np.deg2rad(self.agent.max_climb_angle_dg)))
        self.sx = self.x_max_m - self.x_min_m
        self.sy = self.y_max_m - self.y_min_m
        self.sz = self.z_max_m - self.z_min_m


    def is_out_bounds(self, position:PositionVector) -> bool:
        """
        Check if position is out of bounds 
        """
        if position.x < self.x_min_m or position.x > self.x_max_m:
            return True
        if position.y < self.y_min_m or position.y > self.y_max_m:
            return True
        if position.z < self.z_min_m or position.z > self.z_max_m:
            return True
        
        return False

    def is_in_obstacle(self, position:PositionVector) -> bool:
        """
        Check if position is in obstacle
        """
        for obs in self.obstacles:
            if obs.is_inside2D(position, self.agent.radius_m):
                return True

    def convert_position_to_index(self, position:PositionVector) -> int:
        # """returns 1D index of position"""
        tuple_position = (int(position.x), int(position.y), int(position.z))
        return "_".join(map(str, tuple_position))
        # if position.z == 0:
        #     return int(position.x + position.y * self.sx)
        # else:
        #     index = position.x + (position.y * self.sx) + \
        #         (position.z * self.sx * self.sy)
        #     return int(index)
        
    def convert_index_to_position(self, index:int) -> PositionVector:
        """returns position from 1D index"""

        x = index % self.sx 
        index /= self.sx 
        y = index % self.sy
        z = index / self.sy 

        return PositionVector(int(x), int(y), int(z))
    
        