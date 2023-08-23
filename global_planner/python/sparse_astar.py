import numpy as np
import math as m
from queue import PriorityQueue

import matplotlib.pyplot as plt
import pandas as pd
import random as rand
"""
Paper Reference:
Efficient Two-phase 3D Motion Planning for Small Fixed-wing UAV

The Coarse planner algorithm from the paper is implemented here


Need to snap start position like I did in the goal position
Model vehicle constraints based on velocity

Define max turn angles for aircraft
Define leg points or grid spacing

Expansion of nodes:
    - Based on current heading and max angle turns:

"""

class PositionVector():
    def __init__(self) -> None:
        self.x = 0
        self.y = 0
        self.z = 0
        self.vector = np.array([self.x, self.y, self.z])

    def set_position(self, x:float=0, y:float=0, z:float=0) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.vector = np.array([self.x, self.y, self.z])

    # Compare position
    def __eq__(self, other):
        return list(self.vector) == list(other.vector)

class Obstacle():
    def __init__(self, position:PositionVector, radius:float) -> None:
        self.position = position
        self.radius = radius

    def is_inside2D(self, position:PositionVector, agent_radius:float) -> bool:
        """
        Check if position is inside obstacle
        """
        total_radius = self.radius + agent_radius

        dist = np.sqrt((position.x - self.position.x)**2 +
                          (position.y - self.position.y)**2)
        
        if dist <= total_radius:
            return True
        
        return False
    

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
        return self.position == other.position and \
            self.theta_dg == other.theta_dg

    # Print node
    def __repr__(self):
        return ('({0},{1})'.format(self.position, self.f))


class FWAgent():
    def __init__(self, position:PositionVector, 
                 theta_dg:float=0, psi_dg:float=0, leg_m:float=20) -> None:
        self.position = position
        self.theta_dg = theta_dg #pitch anglemoves.append([next_x, next_y, next_z])
        self.psi_dg = psi_dg # this is azmith heading
        self.radius_m = 10 #radius of aircraft meters
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
                            max_psi_turn_dg:float=30) -> None:
        """
        horizontal_min_turn = v^2/ (g * tan(phi)) where theta is bank angle
        """
        self.horizontal_min_radius_m = horizontal_min_radius_m
        self.max_climb_angle_dg = max_climb_angle_dg
        self.max_psi_turn_dg = max_psi_turn_dg


    def get_moves(self, position:PositionVector, curr_psi_dg:float,
                  step_psi=10) -> list:
        """
        based on current position and heading get all 
        possible forward moves
        """
        
        leg = self.leg_m
        moves = []
        ac_max_psi_dg = self.max_psi_turn_dg

        for i in range(0,ac_max_psi_dg, step_psi):
            next_psi_dg = curr_psi_dg + i
            if next_psi_dg > 360:
                next_psi_dg -= 360
            if next_psi_dg < 0:
                next_psi_dg += 360

            psi_rad = np.deg2rad(next_psi_dg)
            next_x = position.x + round(leg*(np.cos(psi_rad)))
            next_y = position.y + round(leg*(np.sin(psi_rad)))
            for z in range(-1,2,1):
                next_z = position.z + z
                moves.append([next_x, next_y, next_z])

        for i in range(0,ac_max_psi_dg, step_psi):
            next_psi_dg = curr_psi_dg - i
            if next_psi_dg > 360:
                next_psi_dg -= 360
            if next_psi_dg < 0:
                next_psi_dg += 360        

            psi_rad = np.deg2rad(next_psi_dg)
            next_x = position.x + round(leg*(np.cos(psi_rad)))
            next_y = position.y + round(leg*(np.sin(psi_rad)))
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
                 offset_x:float=0,
                 offset_y:float=0,
                 offset_z:float=0) -> None:
        
        self.agent = agent
        self.x_min_m = -500
        self.y_min_m = -500
        self.z_min_m = 0

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
            x_round = self.sx * m.floor(position.x/self.sx)
            
        if direction_vector.y > 0:
            y_round = self.sy * m.ceil(position.y/self.sy)
        else:
            y_round = self.sy * m.floor(position.y/self.sy)
        
        if direction_vector.z > 0:
            z_round = self.sz * m.ceil(position.z/self.sz)
        else:
            z_round = self.sz * m.floor(position.z/self.sz)
        
        rounded_position = PositionVector()
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
        self.sx = m.ceil(2/3 * self.agent.horizontal_min_radius_m)
        self.sy = self.sx
        self.sz = m.ceil(self.sx * 
                        np.tan(np.deg2rad(self.agent.max_climb_angle_dg)))

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
        for obstacle in self.obstacles:
            if obstacle.is_inside2D(position, self.agent.radius_m):
                return True

class HighLevelAstar():
    def __init__(self, grid:Grid) -> None:        
        self.open_set = PriorityQueue()
        self.closed_set = {}

        self.grid = grid
        self.agent = grid.agent
        self.start_node = None
        self.goal_node = None
        
    def init_nodes(self):
        # Snap the start and end position to the grid
        direction = self.agent.position.vector - self.agent.goal_position.vector
        direction_vector = PositionVector()
        direction_vector.set_position(direction[0], direction[1], direction[2])
        rounded_start_position = self.grid.map_position_to_grid(
            self.agent.position, direction_vector)
        
        self.start_node = Node(None, rounded_start_position, 
                               self.agent.theta_dg, self.agent.psi_dg)
        self.start_node.g = self.start_node.h = self.start_node.f = 0
        self.open_set.put((self.start_node.f, self.start_node))

        #snap the goal to the grid 
        direction = self.agent.goal_position.vector - self.agent.position.vector
        direction_vector = PositionVector()
        direction_vector.set_position(direction[0], direction[1], direction[2])
        rounded_goal_position = self.grid.map_position_to_grid(
            self.agent.goal_position, direction_vector)
        
        print("rounded goal position", rounded_goal_position.vector)
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
            scaled_position = PositionVector()
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
        cost = np.linalg.norm(node1.position.vector - node2.position.vector)
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

        while (not self.open_set.empty() and iterations < max_iterations):
            
            cost,current_node = self.open_set.get()
            # print("current node", current_node.position.vector)
            self.closed_set[str(list(current_node.position.vector))] = current_node

            if current_node.position == self.goal_node.position:
                print("found goal", current_node.position)
                return self.return_path(current_node)


            if self.compute_distance(current_node, self.goal_node) < self.agent.leg_m:
                print("found goal", current_node.position)
                return self.return_path(current_node)

            expanded_moves = self.get_legal_moves(
                current_node, current_node.psi_dg)

            # print("current position", current_node.position.vector)

            if not expanded_moves:
                # print("no expanded moves")
                # print("current node", current_node.position.vector)
                continue
                return self.closed_set, self.open_set

            for move in expanded_moves:
                if str(list(move.vector)) in self.closed_set:
                    continue
                
                if move == current_node.position:
                    continue

                neighbor = Node(current_node, move)
                neighbor.g = current_node.g + 1 #self.compute_distance(current_node, neighbor)
                neighbor.h = self.compute_distance(neighbor, self.goal_node)
                neighbor.f = neighbor.g + 1*neighbor.h
                self.open_set.put((neighbor.f, neighbor))
            
            iterations += 1

        print("no path found")
        return self.closed_set, self.open_set


def test_crap():
    start_position = PositionVector()
    start_position.set_position(0,0,0)
    fw_agent = FWAgent(start_position,0, 0)
    fw_agent.vehicle_constraints()
    move_list = fw_agent.get_moves()

    print("moves", move_list)
    #create a 3D plot   
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    #plot the points
    ax.scatter(start_position.x, start_position.y, start_position.z, c='r', marker='o', s=100)
    for move in move_list:
        ax.scatter(start_position.x + move[0], 
                   start_position.y + move[1], 
                   start_position.z + move[2], c='b', marker='o')
    plt.show()

def get_diff_paths(agent:FWAgent, horizon_min_radius_m:float, 
                   max_climb_angle_dg:float, seed_num:int=0) -> tuple:
    
    fw_agent.vehicle_constraints(
        horizontal_min_radius_m=horizon_min_radius_m, 
        max_climb_angle_dg=max_climb_angle_dg)
    
    grid = Grid(fw_agent)
    
    #add an obstacle
    obstacle_position = PositionVector()
    obstacle_position.set_position(400,300,0)
    obstacle = Obstacle(obstacle_position, 50)
    grid.insert_obstacles(obstacle)

    n_obstacles = 100
    #set seed 
    rand.seed(seed_num)
    for i in range(n_obstacles):

        obstacle_position = PositionVector()
        obstacle_position.set_position(rand.randint(grid.x_min_m + 150, grid.x_max_m - 300),
                                       rand.randint(grid.y_min_m + 150 ,grid.y_max_m - 300), 0)
        obstacle = Obstacle(obstacle_position, rand.randint(10,30))

        if obstacle.is_inside2D(agent.position, agent.radius_m + 10):
            continue

        if obstacle.is_inside2D(agent.goal_position, agent.radius_m + 10):
            continue
        
        grid.insert_obstacles(obstacle)

    high_level_astar = HighLevelAstar(grid)
    high_level_astar.init_nodes()
    wp_path = high_level_astar.search() 

    return grid,wp_path

def print_fan():
    """
    Sanity check to make sure the fan concept works from SAS
    """
    curr_psi_dg = 350
    test_moves = fw_agent.get_moves(
        start_position, curr_psi_dg, aircraft_max_turn_dg)
        
    #create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    for move in test_moves:
        ax.scatter(move[0], 
                   move[1], 
                   move[2], c='b', marker='o')
    
    ax.scatter(start_position.x, start_position.y, start_position.z, c='r', marker='o', s=100)
    #direction vector of start position
    x_dir = np.cos(np.deg2rad(curr_psi_dg))
    y_dir = np.sin(np.deg2rad(curr_psi_dg))
    z_dir = start_position.z
    print("x_dir", x_dir)
    print("y_dir", y_dir)
    print("z_dir", z_dir)


    ax.quiver(start_position.x, start_position.y, start_position.z,
                x_dir, y_dir, z_dir, length=5, normalize=True)
    # ax.scatter(x_dir, y_dir, z_dir, c='g', marker='o', s=100)

    plt.show()


if __name__ == '__main__':
    plt.close('all')
    start_position = PositionVector()
    # I need to snap the start position to the grid
    start_position.set_position(0,0,0)
    fw_agent = FWAgent(start_position,0, 0)
    fw_agent.vehicle_constraints(horizontal_min_radius_m=60, 
                                 max_climb_angle_dg=5)
    goal = PositionVector()
    goal.set_position(775,500,25)
    fw_agent.set_goal_state(goal)

    aircraft_speeds_ms = [15]
    aircraft_max_roll_dg = 45
    aircraft_max_pitch_dg = 5
    aircraft_max_turn_dg = 20


    grids = []
    wp_paths = []
    for speed_ms in aircraft_speeds_ms:
        r = speed_ms**2 / (9.81 * np.tan(np.deg2rad(aircraft_max_roll_dg)))
        max_radius_m = m.ceil(r)
        grid,wp_path = get_diff_paths(
            fw_agent, max_radius_m, aircraft_max_pitch_dg, seed_num=2)
        wp_paths.append(wp_path)


    fig4 = plt.figure()
    ax4 = fig4.add_subplot(111)
    for speed,wp_path in zip(aircraft_speeds_ms,wp_paths):
        # if wp_path is None:
        #     continue
        
        # if len(wp_path) > 1:
        #     continue

        x_path = [wp[0] for wp in wp_path]
        y_path = [wp[1] for wp in wp_path]
        ax4.plot(x_path, y_path, '-o', label=str(speed) + ' m/s')

    # plot obstacles
    for obstacle in grid.obstacles:
        circle = plt.Circle((obstacle.position.x, obstacle.position.y), 
                            obstacle.radius, color='r', fill=False)
        ax4.add_artist(circle)


    plt.show()

    #key dictionary
    failed_set = wp_paths[0][0]
    print(failed_set)
    open_set = list(wp_paths[0][1].queue)
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)

    for k,v in failed_set.items():
        print(v.psi_dg)
        print(v.position.vector)
        positon = v.position.vector
        ax1.scatter(positon[0], positon[1], c='r', marker='o', s=10)

    #convert open set to list
    for info in open_set:
        node = info[1] 
        ax1.scatter(node.position.x, node.position.y, 
                    c='b', marker='o', s=10)

    for obstacle in grid.obstacles:
        circle = plt.Circle((obstacle.position.x, obstacle.position.y), 
                            obstacle.radius, color='r', fill=False)
        ax1.add_artist(circle)


    plt.show()
        

