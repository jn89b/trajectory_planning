import pickle as pkl
import numpy as np

import plotly.graph_objects as go


fig = go.Figure([])
n = 1
# SET RANGE TO EQUAL 1 GREATER THAN THE TOTAL NUMBER OF RADARS IN NETWORK
for n in range(2):
    with open('radar_data'+str(n)+'.pickle', 'rb') as file:
        loaded_data = pkl.load(file)
        fig.add_trace(loaded_data)
    n = n + 1


# print("data", loaded_data)
# fig.add_trace(loaded_data)
fig.show()
fig.write_html("path_traj.html")