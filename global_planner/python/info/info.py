import pandas as pd
"""
Notes
- Every 9 columns is a single yaw starting from 0 
- Within each of these 9 indicies represents a roll angle from -60 to 60 deg 
- Rows are -60 to 60 deg pitch


a Float is 4 bytes 

Should only be about 3mb 
"""

# enter parameters here
file_name = 'sig_mat.csv' #90 deg
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
for yaw,pitch_roll_array in yaw_dict.items():
    #loop though yaw keys and get array
    # print(pitch_roll_array.shape)
    #loop through array and get pitch and roll values
    pitch_step = (max_pitch - min_pitch)/pitch_roll_array.shape[0]
    roll_step = (max_roll - min_roll)/pitch_roll_array.shape[1]
    
    for i in range(pitch_roll_array.shape[0]):
        pitch_idx = int((i) * pitch_step) + int(min_pitch)
        print(pitch_idx)
        for j in range(pitch_roll_array.shape[1]):
            
            roll_idx = int((j) * roll_step) + int(min_roll)
            key = f"{roll_idx}_{pitch_idx}_{int(yaw)}"
            rcs_angles[key] = pitch_roll_array[i,j]

# goal is to generate a hash table for each yaw angle 
# something like this 
euler_angles = {}

# Define the roll, pitch, and yaw values
roll = 30.0
pitch = 45.0
yaw = 60.0

# Create a key by concatenating the values
key = f"{roll}_{pitch}_{yaw}"

# Store the values in the dictionary with the concatenated key
euler_angles[key] = {'roll': roll, 'pitch': pitch, 'yaw': yaw}

# Access and print values from the dictionary using the concatenated key
print(f"Roll, Pitch, and Yaw for key '{key}':", euler_angles[key])





