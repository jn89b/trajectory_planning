import numpy as np
import math as m
from queue import PriorityQueue

from src.PositionVector import PositionVector
from src.Grid import Grid

import time

def round_to_nearest_even(number:int):
    rounded_number = round(number)
    if rounded_number % 2 == 1:  # Check if the rounded number is odd
        return rounded_number + 1  # If odd, add 1 to make it even
    else:
        return rounded_number  # If even, return it as is


class Node(object):
    """
    parent = parent of current node
    position = position of node right now it will be x,y coordinates
    g = cost from start to current to node
    h = heuristic 
    f = is total cost
    """
    def __init__(self, parent, 
                 position:PositionVector, 
                 velocity_m:float=15,
                 prev_psi_dg:float=0,
                 theta_dg:float=0, 
                 psi_dg:float=0):
                 
        self.parent = parent 
        self.position = position # x,y,z coordinates
        if parent is not None:
            self.direction_vector = np.array([self.position.x - self.parent.position.x, 
                                            self.position.y - self.parent.position.y, 
                                            self.position.z - self.parent.position.z])
            self.theta_dg = np.arctan2(self.direction_vector[2], 
                                    np.linalg.norm(self.direction_vector[0:2]))
            self.theta_dg = np.rad2deg(self.theta_dg)
            self.psi_dg = np.arctan2(self.direction_vector[1], self.direction_vector[0])
            self.psi_dg = np.rad2deg(self.psi_dg)
            delta_psi_rad = np.deg2rad(self.psi_dg - prev_psi_dg)
            phi_rad = np.arctan((delta_psi_rad*velocity_m)/9.81)
            self.phi_dg = np.rad2deg(phi_rad)

        else:
            self.direction_vector = np.array([self.position.x,
                                              self.position.y,
                                              self.position.z])
            self.theta_dg = theta_dg
            self.psi_dg = psi_dg
            delta_psi_rad = np.deg2rad(self.psi_dg - prev_psi_dg)
            phi_rad = np.arctan((delta_psi_rad*velocity_m)/9.81)
            self.phi_dg = np.rad2deg(phi_rad)

        
        self.g = 0
        self.h = 0
        self.radar_cost = 0
        self.radar_detection = 0
        self.f = 0
        self.total_distance = 0
        self.total_time = 0
        self.rcs_value = 0

        
    def get_direction_vector(self) -> np.array:
        return self.direction_vector

    def __lt__(self, other):
        return self.f < other.f
    
    # Compare nodes
    def __eq__(self, other):
        return self.position == other.position 
    # Print node
    def __repr__(self):
        return ('({0},{1})'.format(self.position, self.f))



class SparseAstar():
    def __init__(self, grid:Grid, 
                 velocity=5,
                 use_radar:bool=False,
                 rcs_hash:dict={}, 
                 radar_info:list=[],
                 radar_weight:float=0, 
                 max_rcs:float=1) -> None:        
        self.open_set = PriorityQueue()
        self.closed_set = {}

        self.grid = grid
        self.agent = grid.agent
        self.start_node = None
        self.goal_node = None
        self.velocity = velocity

        self.use_radar = use_radar
        self.radars = radar_info
        self.radar_weight = radar_weight
        self.rcs_hash = rcs_hash
        self.max_rcs = max_rcs

    def clear_sets(self):
        self.open_set = PriorityQueue()
        self.closed_set = {}

    def update_radar_weight(self, radar_weight:float):
        self.radar_weight = radar_weight
    
    def init_nodes(self):
        # Snap the start and end position to the grid
        direction = self.agent.position.vec - self.agent.goal_position.vec
        direction_vector = PositionVector(direction[0], direction[1], direction[2])
        direction_vector.set_position(direction[0], direction[1], direction[2])
        # rounded_start_position = self.grid.map_position_to_grid(
        #     self.agent.position, direction_vector)
        
        self.start_node = Node(None, self.agent.position, 
                               self.velocity, 0,
                               self.agent.theta_dg, self.agent.psi_dg)
        
        radar_cost, rcs_val = self.get_radar_cost(self.start_node)
        self.start_node.radar_cost = radar_cost
        self.start_node.rcs_value = rcs_val
        self.start_node.radar_detection = radar_cost


        self.start_node.g = self.start_node.h = self.start_node.f = 0
        self.open_set.put((self.start_node.f, self.start_node))

        # self.goal_node = Node(None, rounded_goal_position)
        self.goal_node = Node(None, self.agent.goal_position, 
                              self.velocity, 0,
                              self.agent.theta_dg, self.agent.psi_dg)
        self.goal_node.g = self.goal_node.h = self.goal_node.f = 0

    def is_valid_position(self, position:PositionVector) -> bool:
        """Checks if position is valid based on grid constraints"""
        if self.grid.is_out_bounds(position):
            return False
        if self.grid.is_in_obstacle(position):
            return False
        
        return True
        
    def get_legal_moves(self, current_node:Node, psi_dg:float)-> list:
        """Get legal moves based on agent constraints"""
        moves = self.agent.get_moves(current_node.position, psi_dg)
        legal_moves = []

        # need to scale moves based on grid size
        for move in moves:
            scaled_move = [move[0], move[1], move[2]]
            scaled_position = PositionVector(move[0], move[1], move[2])
            scaled_position.set_position(
                scaled_move[0], scaled_move[1], scaled_move[2])
            if self.is_valid_position(scaled_position):
                legal_moves.append(scaled_position)

        return legal_moves
    
    def compute_cost(self) -> float:
        """
        pass
        """

    def compute_distance(self, node1:Node, node2:Node) -> float:
        """
        Compute distance between two nodes
        """
        cost = np.linalg.norm(node1.position.vec - node2.position.vec)
        return cost

    def get_rcs_key(self, roll:int,pitch:int,yaw:int) -> str:
        """returns the rcs key based on roll pitch yaw"""
        return f"{roll}_{pitch}_{yaw}"


    def get_key(self, azimith_dg:int, elevation_dg:int) -> str:
        """returns the rcs key based on roll pitch yaw"""
        return f"{azimith_dg}_{elevation_dg}"


    def return_path(self,current_node):
        path = []
        current = current_node
        
        while current is not None:
            states = [current.position.x, current.position.y, current.position.z]
            states.append(current.theta_dg)
            states.append(current.phi_dg)
            states.append(current.psi_dg)
            states.append(current.rcs_value)
            states.append(current.radar_detection)
            path.append(states)
            current = current.parent
        # Return reversed path as we need to show from start to end path
        path = path[::-1]
        
        waypoints = []
        for points in path:
            waypoints.append(points)
            
        return waypoints


    def get_radar_cost(self, node:Node)->tuple:
        """
        pass
        """
        #compute heuristic based on radar 
        if self.use_radar:
            radar_cost = 0
            rcs_val = -100
            radar_probs = []
            ## Need to reclass this to radar system
            for radar in self.radars:
                idx_pos = self.grid.convert_position_to_index(node.position)
                #get radar value
                #wrap psi_dg between 0 and 360
                wrapped_psi_dg = node.psi_dg

                rel_phi_dg = node.phi_dg
                rel_theta_dg = node.theta_dg - (90 - radar.elevation_angle_dg)
                # rel_psi_dg = wrapped_psi_dg - radar.azmith_angle_dg
                # rel_psi_dg = radar.azmith_angle_dg - wrapped_psi_dg

                dy = node.position.y - radar.pos.y
                dx = node.position.x - radar.pos.x
                rel_psi_dg = np.arctan2(dy, dx) * 180 / np.pi
                rel_psi_dg = rel_psi_dg - node.psi_dg + 180

                if rel_psi_dg > 360:
                    rel_psi_dg -= 360
                if rel_psi_dg < 0:
                    rel_psi_dg += 360

                if idx_pos in radar.detection_info:
                    #even_psi = round_to_nearest_even(rel_psi_dg)
                    rcs_key = self.get_key(
                        int(rel_psi_dg),
                        int(0)
                    )
                    
                    dist_radar = np.linalg.norm(
                        radar.pos.vec - node.position.vec
                    )

                    if rcs_key in self.rcs_hash:
                        radar_cost = radar.compute_prob_detect(
                            dist_radar, 
                            self.rcs_hash[rcs_key])
                        
                        rcs_val = self.rcs_hash[rcs_key]
                        radar_probs.append(radar_cost)

                else:


                    #Need to fix voxel detections this is a hacky way to fix it
                    z_vals = [0, 1, -1, 2, -2]
                    y_vals = [0, 1, -1, 2, -2]
                    x_vals = [0, 1, -1, 2, -2]
                    for z in z_vals:
                        next_z = node.position.z + z
                        new_pos = PositionVector(node.position.x,
                                                    node.position.y,
                                                    next_z)
                        idx_pos = self.grid.convert_position_to_index(new_pos)
                        if idx_pos in radar.detection_info:
                            rcs_key = self.get_key(
                                int(rel_psi_dg),
                                int(0)
                            )
                            dist_radar = np.linalg.norm(
                                radar.pos.vec - new_pos.vec
                            )
                            if rcs_key in self.rcs_hash:

                                radar_cost = radar.compute_prob_detect(
                                    dist_radar, 
                                    self.rcs_hash[rcs_key])
                                
                                rcs_val = self.rcs_hash[rcs_key]
                                radar_probs.append(radar_cost)

                                break

                    for x,y in zip(x_vals, y_vals):
                        next_x = node.position.x + x
                        next_y = node.position.y + y
                        new_pos = PositionVector(next_x,
                                                    next_y,
                                                    node.position.z)
                        idx_pos = self.grid.convert_position_to_index(new_pos)
                        if idx_pos in radar.detection_info:
                            rcs_key = self.get_key(
                                int(rel_psi_dg),
                                int(0)
                            )
                            dist_radar = np.linalg.norm(
                                radar.pos.vec - new_pos.vec
                            )
                            if rcs_key in self.rcs_hash:
                                radar_cost = radar.compute_prob_detect(
                                    dist_radar, 
                                    self.rcs_hash[rcs_key])
                                rcs_val = self.rcs_hash[rcs_key]
                    
                                radar_probs.append(radar_cost)
                                break
        
        if len(radar_probs) > 1:
            radar_cost = (1 - np.prod(1 - np.array(radar_probs))) #*np.sqrt(len(radar_probs))
            # continue
        elif len(radar_probs) == 1:
            radar_cost = radar_probs[0]
        else:
            radar_cost = 0

        return radar_cost, rcs_val
        

    def search(self):
        
        max_iterations = 10000
        iterations = 0
        
        start_time = time.time()
        max_time = 10 #seconds

        while (not self.open_set.empty() and iterations < max_iterations):

            iterations += 1
            cost,current_node = self.open_set.get()
            # print("current node", current_node.position.vec)
            # print("current node psi", current_node.psi_dg)
                
            self.closed_set[str(list(current_node.position.vec))] = current_node

            current_time = time.time() - start_time
            # print("position", current_node.position.vec)

            if current_time > max_time:
                print("reached time limit", current_time)
                return self.return_path(current_node)

            if current_node.position == self.goal_node.position:
                print("time", current_time)
                print("found goal", current_node.position)
                return self.return_path(current_node)
            
            if iterations >= max_iterations:
                print("iterations", iterations)
                return self.return_path(current_node)
                # break

            if self.compute_distance(current_node, self.goal_node) < self.agent.leg_m:
                print("time", current_time)
                print("found goal", current_node.position)
                return self.return_path(current_node)

            expanded_moves = self.get_legal_moves(
                current_node, current_node.psi_dg)

            if not expanded_moves:
                continue

            for move in expanded_moves:
                if str(list(move.vec)) in self.closed_set:
                    continue
                
                if move == current_node.position:
                    continue

                neighbor = Node(current_node, move, self.velocity, 
                                current_node.psi_dg)

                #compute heuristic based on radar 
                if self.use_radar:
                    radar_cost = 0
                    rcs_val = -100
                    radar_probs = []
                    ## Need to reclass this to radar system
                    for radar in self.radars:
                        idx_pos = self.grid.convert_position_to_index(neighbor.position)
                        #get radar value
                        #wrap psi_dg between 0 and 360
                        wrapped_psi_dg = neighbor.psi_dg

                        rel_phi_dg = neighbor.phi_dg
                        rel_theta_dg = neighbor.theta_dg - (90 - radar.elevation_angle_dg)
                        # rel_psi_dg = wrapped_psi_dg - radar.azmith_angle_dg
                        # rel_psi_dg = radar.azmith_angle_dg - wrapped_psi_dg

                        dy = neighbor.position.y - radar.pos.y
                        dx = neighbor.position.x - radar.pos.x
                        rel_psi_dg = np.arctan2(dy, dx) * 180 / np.pi
                        rel_psi_dg = rel_psi_dg - neighbor.psi_dg + 180

                        if rel_psi_dg > 360:
                            rel_psi_dg -= 360
                        if rel_psi_dg < 0:
                            rel_psi_dg += 360

                        if idx_pos in radar.detection_info:
                            #even_psi = round_to_nearest_even(rel_psi_dg)
                            rcs_key = self.get_key(
                                int(rel_psi_dg),
                                int(0)
                            )
                            
                            dist_radar = np.linalg.norm(
                                radar.pos.vec - neighbor.position.vec
                            )

                            if rcs_key in self.rcs_hash:
                                radar_cost = radar.compute_prob_detect(
                                    dist_radar, 
                                    self.rcs_hash[rcs_key])
                                
                                rcs_val = self.rcs_hash[rcs_key]
                                radar_probs.append(radar_cost)

                        else:


                            #Need to fix voxel detections this is a hacky way to fix it
                            z_vals = [0, 1, -1, 2, -2]
                            y_vals = [0, 1, -1, 2, -2]
                            x_vals = [0, 1, -1, 2, -2]
                            for z in z_vals:
                                next_z = neighbor.position.z + z
                                new_pos = PositionVector(neighbor.position.x,
                                                            neighbor.position.y,
                                                            next_z)
                                idx_pos = self.grid.convert_position_to_index(new_pos)
                                if idx_pos in radar.detection_info:
                                    rcs_key = self.get_key(
                                        int(rel_psi_dg),
                                        int(0)
                                    )
                                    dist_radar = np.linalg.norm(
                                        radar.pos.vec - new_pos.vec
                                    )
                                    if rcs_key in self.rcs_hash:

                                        radar_cost = radar.compute_prob_detect(
                                            dist_radar, 
                                            self.rcs_hash[rcs_key])
                                        
                                        rcs_val = self.rcs_hash[rcs_key]
                                        radar_probs.append(radar_cost)

                                        break

                            for x,y in zip(x_vals, y_vals):
                                next_x = neighbor.position.x + x
                                next_y = neighbor.position.y + y
                                new_pos = PositionVector(next_x,
                                                            next_y,
                                                            neighbor.position.z)
                                idx_pos = self.grid.convert_position_to_index(new_pos)
                                if idx_pos in radar.detection_info:
                                    rcs_key = self.get_key(
                                        int(rel_psi_dg),
                                        int(0)
                                    )
                                    dist_radar = np.linalg.norm(
                                        radar.pos.vec - new_pos.vec
                                    )
                                    if rcs_key in self.rcs_hash:
                                        radar_cost = radar.compute_prob_detect(
                                            dist_radar, 
                                            self.rcs_hash[rcs_key])
                                        rcs_val = self.rcs_hash[rcs_key]
                            
                                        radar_probs.append(radar_cost)
                                        break
                
                if len(radar_probs) > 1:
                    radar_cost = (1 - np.prod(1 - np.array(radar_probs))) #*np.sqrt(len(radar_probs))
                    # continue
                elif len(radar_probs) == 1:
                    radar_cost = radar_probs[0]
                else:
                    radar_cost = 0

                neighbor.g = current_node.g + 1
                neighbor.rcs_value = rcs_val
                neighbor.radar_detection = radar_cost
                neighbor.radar_cost = self.radar_weight*radar_cost
                neighbor.h = (self.compute_distance(neighbor, self.goal_node))
                neighbor.f = neighbor.g +  neighbor.h + neighbor.radar_cost
                self.open_set.put((neighbor.f, neighbor))

        return self.return_path(current_node)