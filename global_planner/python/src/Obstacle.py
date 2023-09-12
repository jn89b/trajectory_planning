import numpy as np

from src.PositionVector import PositionVector

class Obstacle():
    def __init__(self, position:PositionVector, radius_m:float) -> None:
        self.position = position
        self.radius_m = radius_m

    def is_inside2D(self, position:PositionVector, agent_radius:float=0.0) -> bool:
        """
        Check if position is inside obstacle
        """
        total_radius = self.radius_m + agent_radius

        dist = np.sqrt((position.x - self.position.x)**2 +
                          (position.y - self.position.y)**2)
        if dist <= total_radius:
            return True
        
        return False

