import numpy as np
import math as m
from queue import PriorityQueue

import matplotlib.pyplot as plt
import pandas as pd

"""
Paper Reference:
Efficient Two-phase 3D Motion Planning for Small Fixed-wing UAV

The Coarse planner algorithm from the paper is implemented here


Need to snap start position like I did in the goal position

Model vehicle constraints based on velocity

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
    def __init__(self) -> None:
        pass

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
                 theta_dg:float=0, psi_dg:float=0) -> None:
        self.position = position
        self.theta_dg = theta_dg #pitch anglemoves.append([next_x, next_y, next_z])
        self.psi_dg = psi_dg # this is azmith heading

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
                            max_climb_angle_dg:float=10) -> None:
        """
        horizontal_min_turn = v^2/ (g * tan(phi)) where theta is bank angle
        """
        self.horizontal_min_radius_m = horizontal_min_radius_m
        self.max_climb_angle_dg = max_climb_angle_dg


    def get_moves(self, psi_dg:float) -> list:
        """
        based on current position and heading get all 
        possible forward moves
        """
        next_x = round(np.cos(np.deg2rad(psi_dg)))
        next_y = round(np.sin(np.deg2rad(psi_dg)))
        next_z = 0
        moves = []
        # #if moves are diagonal 
        if (next_x != 0 and next_y != 0):
            moves.append([next_x-next_x, next_y, next_z])
            moves.append([next_x-next_x, next_y, next_z+1])
            moves.append([next_x-next_z, next_y, next_z-1])
            moves.append([next_x, next_y-next_y, next_z])
            moves.append([next_x, next_y-next_y, next_z+1])
            moves.append([next_x, next_y-next_y, next_z-1])
            moves.append([next_x, next_y, next_z])
            moves.append([next_x, next_y, next_z + 1])
            moves.append([next_x, next_y, next_z - 1])
            return moves
        # moves going in x direction
        elif(next_x != 0 and next_y == 0):
            moves.append([next_x, next_y, next_z])
            moves.append([next_x, next_y, next_z + 1])
            moves.append([next_x, next_y, next_z - 1])
            moves.append([next_x, next_y+1, next_z])
            moves.append([next_x, next_y+1, next_z+1])
            moves.append([next_x, next_y+1, next_z-1])
            moves.append([next_x, next_y-1, next_z])
            moves.append([next_x, next_y-1, next_z+1])
            moves.append([next_x, next_y-1, next_z-1])
            return moves
        # moves going in y direction
        else:
            moves.append([next_x, next_y, next_z])
            moves.append([next_x, next_y, next_z + 1])
            moves.append([next_x, next_y, next_z - 1])
            moves.append([next_x+1, next_y, next_z])
            moves.append([next_x+1, next_y, next_z+1])
            moves.append([next_x+1, next_y, next_z-1])
            moves.append([next_x-1, next_y, next_z])
            moves.append([next_x-1, next_y, next_z+1])
            moves.append([next_x-1, next_y, next_z-1])
            return moves

        # so loop around all surrounding nodes from the next_x
        #based on this next position get surrounding nodes
        
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
        self.x_min_m = -100
        self.y_min_m = -100
        self.z_min_m = -100

        self.x_max_m = x_max_m
        self.y_max_m = y_max_m
        self.z_max_m = z_max_m
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.offset_z = offset_z

        self.set_grid_size()

    def get_grid_size(self) -> tuple:
        return (self.sx, self.sy, self.sz)

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
        return False

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
        self.goal_node = Node(None, rounded_goal_position)
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
        moves = self.agent.get_moves(psi_dg)
        legal_moves = []

        # need to scale moves based on grid size
        for move in moves:
            scaled_move = [move[0] * self.grid.sx, 
                           move[1] * self.grid.sy, 
                           move[2] * self.grid.sz]
            scaled_position = PositionVector()
            scaled_position.set_position(current_node.position.x + scaled_move[0],
                                         current_node.position.y + scaled_move[1],
                                         current_node.position.z + scaled_move[2])
            if self.is_valid_position(scaled_position):
                legal_moves.append(scaled_position)

        return legal_moves
    
    def compute_cost(self) -> float:
        pass

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
        
        max_iterations = 1E100
        iterations = 0

        while (not self.open_set.empty() and iterations < max_iterations):
            
            cost,current_node = self.open_set.get()
            # print("current node", current_node.position.vector)
            self.closed_set[str(list(current_node.position.vector))] = current_node

            if current_node.position == self.goal_node.position:
                print("found goal", current_node.position)
                return self.return_path(current_node)
                # return self.get_path(current_node)

            expanded_moves = self.get_legal_moves(
                current_node, current_node.psi_dg)

            if not expanded_moves:
                print("no expanded moves")
                return None

            for move in expanded_moves:

                if str(list(move.vector)) in self.closed_set:
                    continue
                
                if move == current_node.position:
                    continue

                neighbor = Node(current_node, move)

                neighbor.g = current_node.g + 1 #self.compute_distance(current_node, neighbor)
                neighbor.h = self.compute_distance(neighbor, self.goal_node)
                neighbor.f = neighbor.g + neighbor.h
                self.open_set.put((neighbor.f, neighbor))
            
            iterations += 1

        print("no path found")

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

if __name__ == '__main__':
    start_position = PositionVector()
    # I need to snap the start position to the grid
    start_position.set_position(10,10,0)
    fw_agent = FWAgent(start_position,0, 270)
    fw_agent.vehicle_constraints(horizontal_min_radius_m=25, max_climb_angle_dg=10)
    goal = PositionVector()
    goal.set_position(777,500,25)
    fw_agent.set_goal_state(goal)

    grid = Grid(fw_agent)
    print(grid.sx, grid.sy, grid.sz)
    
    high_level_astar = HighLevelAstar(grid)
    high_level_astar.init_nodes()
    wp_path = high_level_astar.search() 

    #plot path
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    # ax.set_zlabel('Z')
    x = [wp[0] for wp in wp_path]
    y = [wp[1] for wp in wp_path]
    z = [wp[2] for wp in wp_path]
    theta_dg = [wp[3] for wp in wp_path]
    psi_dg = [wp[4] for wp in wp_path]
    ax.plot(x, y, '-o')
    plt.show()

    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111, projection='3d')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')

    #plot the points
    ax2.plot(x, y, z, '-o')
    plt.show()

    #save trajectory to csv
    df = pd.DataFrame(wp_path, columns=['x', 'y', 'z', 'theta_dg', 'psi_dg'])
    df.to_csv('trajectory.csv', index=False)


    # compare_position = PositionVector()
    # compare_position.set_position(24,5,16)

    # if (goal == compare_position):
    #     print("equal")

    # node = Node(start_position, goal)
    # print("node direction vector", node.get_direction_vector())
    # print("node theta", node.theta_dg)
    # print("node psi", node.psi_dg)

    # closed_set = {}

    # str_position = str(node.position.vector)
    # closed_set[str(list(node.position.vector))] = node
    # print("closed set", closed_set)
    
    # another_node = Node(start_position, goal)
    # if (str(list(another_node.position.vector)) in closed_set):
    #     print("in closed set")

    # if (node == another_node):
    #     print("node equal")
        
    # goal = PositionVector()
    # goal.set_position(24,5,16)
    # unit_vector = PositionVector()
    # unit_vector.set_position(-1,1,-1)
    # rounded_position = grid.map_position_to_grid(goal, unit_vector)
    # print(rounded_position.vector)


