import numpy as np
import math as m
from queue import PriorityQueue

from src.PositionVector import PositionVector
from src.Grid import Grid

import time



class Node(object):
    """
    parent = parent of current node
    position = position of node right now it will be x,y coordinates
    g = cost from start to current to node
    h = heuristic 
    f = is total cost
    """
    def __init__(self, parent, position:PositionVector, 
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

        else:
            self.direction_vector = np.array([self.position.x,
                                              self.position.y,
                                              self.position.z])
            self.theta_dg = theta_dg
            self.psi_dg = psi_dg
            

        self.g = 0
        self.h = 0
        self.f = 0
        self.total_distance = 0
        self.total_time = 0

        
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
    def __init__(self, grid:Grid, use_radar:bool=False, 
                 radar_info:list=[],
                 radar_weight:float=0) -> None:        
        self.open_set = PriorityQueue()
        self.closed_set = {}

        self.grid = grid
        self.agent = grid.agent
        self.start_node = None
        self.goal_node = None
        self.use_radar = use_radar
        self.radars = radar_info
        self.radar_weight = radar_weight
        
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
                               self.agent.theta_dg, self.agent.psi_dg)
        self.start_node.g = self.start_node.h = self.start_node.f = 0
        self.open_set.put((self.start_node.f, self.start_node))

        # self.goal_node = Node(None, rounded_goal_position)
        self.goal_node = Node(None, self.agent.goal_position, 
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

    def return_path(self,current_node):
        path = []
        current = current_node
        
        while current is not None:
            states = [current.position.x, current.position.y, current.position.z]
            states.append(current.theta_dg)
            states.append(current.psi_dg)
            path.append(states)
            current = current.parent
        # Return reversed path as we need to show from start to end path
        path = path[::-1]
        start_value = 0
        waypoints = []
        for points in path:
            waypoints.append(points)
            
        return waypoints

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
                return self.closed_set, self.open_set

            for move in expanded_moves:
                if str(list(move.vec)) in self.closed_set:
                    continue
                
                if move == current_node.position:
                    continue

                neighbor = Node(current_node, move)
                neighbor.g = current_node.g + 1#self.compute_distance(current_node, neighbor)
                radar_cost = 0
                #compute heuristic based on radar 
                if self.use_radar:
                    for radar in self.radars:
                        idx_pos = self.grid.convert_position_to_index(neighbor.position)
                        if idx_pos in radar.detection_info:
                            #get radar value
                            radar_cost = radar.detection_info[idx_pos][0]

                neighbor.h = (self.compute_distance(neighbor, self.goal_node)/2) + \
                    (self.radar_weight*radar_cost)
                
                neighbor.f = neighbor.g + 1*neighbor.h
                self.open_set.put((neighbor.f, neighbor))

            
        # return self.closed_set, self.open_set
        return self.return_path(current_node)