"""
Parse out the information 

Columns are relative azmith positions:
from 0 to 360 in 2 degree increments

Rows are relative elevation positions:
from -80 to 80 in 10 degree increments

"""

import pandas as pd
import numpy as np
from scipy import interpolate
# from src.Config.radar_config import RADAR_AIRCRAFT_HASH_FILE


info_dir = 'info/'
# filename = 'plane_sig' #90 deg
# filename = 'plane_90s_plane_sig'
# filename = 'plane_45front_90rear_plane_sig'
filename = 'plane_45s_plane_sig'
info_dir = info_dir + filename + '.csv'

df = pd.read_csv(info_dir, header=None)

yaw_step = 2

# max roll
max_roll = 80
min_roll = -80

# max pitch
max_pitch = 80
min_pitch = -80

elevation_step = 10


elevation_list = np.arange(min_pitch, max_pitch+1, elevation_step)
azimuth_list = np.arange(0, 360+1, yaw_step)

#convert df to 2d array
rcs_array = df.values

# loop through 
# Create a 2D interpolation function
interp_func = interpolate.interp2d(azimuth_list, elevation_list, 
                                    rcs_array, kind='linear')


elevations = np.arange(min_pitch, max_pitch+1, 1)
azimuths = np.arange(0, 360+1, 1)

rcs_vals = {}
casadi_table = []
for az in azimuths:
    az_row = []
    rcs_val = interp_func(az, 0)
    for ele in elevations:
        az_row.append(rcs_val[0])
        key = f"{az}_{ele}"
        rcs_vals[key] = rcs_val[0]
        print(key, rcs_vals[key])

    casadi_table.append(az_row)


rcs_table = pd.DataFrame(casadi_table)
rcs_table.to_csv('info/' + filename + '_mpc_table.csv')

#export this to a csv file
rcs_hash = pd.DataFrame.from_dict(rcs_vals, orient='index')
#save to csv
rcs_hash.to_csv('info/hash/' + filename + '_hash.csv')




