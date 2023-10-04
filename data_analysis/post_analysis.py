import pandas as pd
import os
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np
import pickle as pkl

def load_pickle():
    """
    pass
    """
    with open('radar_params_obs.pickle', 'rb') as file:
        loaded_data = pkl.load(file)

    return loaded_data

#get all csvs in local directory
def get_csvs():
    """
    pass
    """
    csvs = []
    for file in os.listdir():
        if file.endswith(".csv"):
            csvs.append(file)
    return csvs


def get_csvs_based_on_weight(csv_filenames:list, weight:int):
    desired_csvs = []
    for name in csv_filenames:
        split_string = name.split("weight_")
        if len(split_string) > 1:
            print(split_string[1])
            if split_string[1] == str(int(weight))+".csv":
                desired_csvs.append(name)

    return desired_csvs

seaborn_palette = sns.color_palette("colorblind")

weights = [0, 5, 10, 15] #only weights available
desired_weight = weights[-1]
csvs = get_csvs()
searched_filenames = get_csvs_based_on_weight(csvs, desired_weight)

pickle_stuff = load_pickle()

# get all the csvs that have the same weight
dfs_list = []
for filename in searched_filenames:
    df = pd.read_csv(filename)
    dfs_list.append(df)

# plot x, y trajectories of the various different configurations
fig,ax = plt.subplots()
ax.set_xlim(0, 110)
ax.set_ylim(0, 110)

prob_detection_means = []
prob_detection_std = []
for i, df in enumerate(dfs_list):
    name = searched_filenames[i]
    x_pos = df['x pos']
    y_pos = df['y pos']
    z_pos = df['z pos']
    
    pd_mean = np.mean(df['pd'])
    pd_std = np.std(df['pd'])
    prob_detection_means.append(pd_mean)
    prob_detection_std.append(pd_std)

    ax.plot(x_pos, y_pos, '-o', label=name, 
            color=seaborn_palette[i])
ax.legend()

plt.show()



