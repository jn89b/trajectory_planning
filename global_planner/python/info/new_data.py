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


file_name = 'info/plane_sig.csv' #90 deg

df = pd.read_csv(file_name, header=None)

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
for az in azimuths:
    for ele in elevations:
        key = f"{az}_{ele}"
        rcs_vals[key] = interp_func(az, ele)
        print(key, rcs_vals[key])


#export this to a csv file
rcs_hash = pd.DataFrame.from_dict(rcs_vals, orient='index')
#save to csv
rcs_hash.to_csv('info/plane_sig_hash.csv')




