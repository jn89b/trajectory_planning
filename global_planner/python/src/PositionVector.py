import numpy as np

class PositionVector():
    def __init__(self, x:float, y:float, z:float=0) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.vec = np.array([x, y, z])

    def set_position(self, x:float=0, y:float=0, z:float=0) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.vec= np.array([self.x, self.y, self.z])

    # Compare position
    def __eq__(self, other):
        return list(self.vec) == list(other.vec)
    
def rotation_z(psi_rad:float) -> np.ndarray:
    """returns a 2D rotation matrix for a given angle psi_rad"""
    return np.array([[np.cos(psi_rad), -np.sin(psi_rad), 0],
                     [np.sin(psi_rad), np.cos(psi_rad), 0],
                     [0, 0, 1]])

def rotation_x(theta_rad:float) -> np.ndarray:
    """returns a 2D rotation matrix for a given angle theta_rad"""
    return np.array([[1, 0, 0],
                     [0, np.cos(theta_rad), -np.sin(theta_rad)],
                     [0, np.sin(theta_rad), np.cos(theta_rad)]])

def rotation_y(phi_rad:float) -> np.ndarray:
    """returns a 2D rotation matrix for a given angle phi_rad"""
    return np.array([[np.cos(phi_rad), 0, np.sin(phi_rad)],
                     [0, 1, 0],
                     [-np.sin(phi_rad), 0, np.cos(phi_rad)]])

