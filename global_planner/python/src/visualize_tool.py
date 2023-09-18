import plotly.graph_objects as go
import matplotlib.pyplot as plt
import numpy as np

# plot 3D axis of roll pitch yaw of aircraft 

class State():
    def __init__(self, 
                 x:float, y:float, z:float,
                 roll_dg:float, pitch_dg:float,
                 yaw_dg:float) -> None:
        
        self.x = x
        self.y = y
        self.z = z

        self.roll_dg = roll_dg
        self.pitch_dg = pitch_dg
        self.yaw_dg = yaw_dg


def create_transformation_matrix(roll_rad:float, 
                                 pitch_rad:float, 
                                 yaw_rad:float, 
                                 x:float, y:float, z:float):
    # Create individual rotation matrices for roll, pitch, and yaw
    R_roll = np.array([[1, 0, 0],
                       [0, np.cos(roll_rad), -np.sin(roll_rad)],
                       [0, np.sin(roll_rad), np.cos(roll_rad)]])
    
    R_pitch = np.array([[np.cos(pitch_rad), 0, np.sin(pitch_rad)],
                        [0, 1, 0],
                        [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]])
    
    R_yaw = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0],
                      [np.sin(yaw_rad), np.cos(yaw_rad), 0],
                      [0, 0, 1]])
    
    # Combine the rotation matrices
    rotation_matrix = R_yaw.dot(R_pitch).dot(R_roll)
    
    # Create the transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = [x, y, z]
    
    return transformation_matrix

if __name__ == "__main__":
    

    state = State(0,0,10,0,0,0)

