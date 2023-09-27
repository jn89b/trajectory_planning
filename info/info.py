import pandas as pd
from scipy import interpolate
import numpy as np
#interpolate the data to get the rcs values for each angle

"""
Notes
- Every 9 columns is a single yaw starting from 0 
- Within each of these 9 indicies represents a roll angle from -60 to 60 deg 
- Rows are -60 to 60 deg pitch


a Float is 4 bytes 

Should only be about 3mb 
"""

# enter parameters here
file_name = 'info/sig_mat_45front_90rear_noseONzero.csv' #90 deg
#no colunm names
df = pd.read_csv(file_name, header=None)

# get every 9 columns and store in a list
# loop through columns
every_col_idx = 9
yaw_step = 2 

# max roll 
max_roll = 60
min_roll = -60

#max pitch 
max_pitch = 60
min_pitch = -60

pitch_step = 15
roll_step = 15


yaw_dict = {}
for i in range(0, len(df.columns), every_col_idx):
    if i == 0:
        yaw_key = int(0)
        yaw_dict[yaw_key] = df.iloc[:, i:i+every_col_idx].values
    
    yaw_key = int((i/every_col_idx)*yaw_step)
    print(yaw_key)
    yaw_dict[yaw_key] = df.iloc[:, i:i+every_col_idx].values

# each of these yaw values contains a 2d array 
# where each row represents the pitch and the columns represent the roll
rcs_angles = {}

pitch_list = np.arange(min_pitch, max_pitch+1, pitch_step)
roll_list = np.arange(min_roll, max_roll+1, roll_step)


pitch_loop = np.arange(min_pitch, max_pitch+1, 1)
roll_loop = np.arange(min_roll, max_roll+1, 1)



for yaw,pitch_roll_array in yaw_dict.items():

    # Create a 2D interpolation function
    interp_func = interpolate.interp2d(pitch_list, roll_list, 
                                       pitch_roll_array, kind='linear')

    print("interpolating")
    for pitch in pitch_loop:
        for roll in roll_loop:
            # print(pitch, roll)
            roll = int(roll)
            pitch = int(pitch)
            yaw = int(yaw)
            key = f"{roll}_{pitch}_{yaw}"
            
            if roll == -26 and yaw == 75:
                print("found", roll, pitch, yaw, 
                      key)

            rcs_angles[key] = interp_func(pitch, roll)


#export this to a csv file
rcs_hash = pd.DataFrame.from_dict(rcs_angles, orient='index')
#save to csv
rcs_hash.to_csv('info/sig_mat_45front_90rear_noseONzero_hash.csv')

# goal is to generate a hash table for each yaw angle 
# something like this 
euler_angles = {}

# Define the roll, pitch, and yaw values
roll = int(30.0)
pitch = int(45.0)
yaw = int(60.0)

# Create a key by concatenating the values
key = f"{roll}_{pitch}_{yaw}"

# Store the values in the dictionary with the concatenated key
euler_angles[key] = {'roll': roll, 'pitch': pitch, 'yaw': yaw}

# Access and print values from the dictionary using the concatenated key
print(f"Roll, Pitch, and Yaw for key '{key}':", euler_angles[key])





