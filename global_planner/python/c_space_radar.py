import numpy as np
from src.PositionVector import PositionVector
from src.Grid import Grid, FWAgent

some_pos = PositionVector(15, 20, 5) 
fw_agent = FWAgent(some_pos, 0, 0, 5)
grid = Grid(fw_agent)

some_idx = grid.convert_position_to_index(some_pos)

test_position = grid.convert_index_to_position(some_idx)

print("some_idx", some_idx)
print("test_position", test_position.x, test_position.y, test_position.z)

# if some_pos.vec in position_dict.keys():
#     print("yes in dictionary")