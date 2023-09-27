import pandas as pd
import sys

df = pd.read_csv('info/rcs_hash.csv', header=None)
#get first column
rpy_keys = df.iloc[:, 0]
rcs_vals = df.iloc[:, 1]

#convert to dictionary
rcs_hash = dict(zip(rpy_keys, rcs_vals))
print(sys.getsizeof(rcs_hash))

#%% 

def get_rcs(roll,pitch,yaw):
    return f"{roll}_{pitch}_{yaw}"

# get the first row of the dataframe
test = df.iloc[1, :]

#Test query
roll =  -26
pitch = 0
yaw =   75

#rcs_val = rcs_hash[query(int(roll), int(pitch), int(yaw))]
#key = f"{int(roll)}_{int(pitch)}_{int(yaw)}"
key = get_rcs(roll,pitch,yaw)

vals = rcs_hash[key]

#%%
keys = rcs_hash.keys()
# Define the roll, pitch, and yaw values
roll = int(30.0)
pitch = int(45.0)
yaw = int(60.0)

# Create a key by concatenating the values
key = f"{roll}_{pitch}_{yaw}"

euler_angles = {}
# Store the values in the dictionary with the concatenated key
euler_angles[key] = {'roll': roll, 'pitch': pitch, 'yaw': yaw}

# if key in euler_angles:
#     print("yes")
# else: