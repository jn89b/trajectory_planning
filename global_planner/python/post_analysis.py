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
    #go to a directory
    os.chdir('data_analysis')
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

names = ['Skywalker', '90s', '45s', '45s90']
weights = [0, 5, 10, 15] #only weights available
desired_weight = weights[-1]
csvs = get_csvs()
searched_filenames = get_csvs_based_on_weight(csvs, desired_weight)
radars = load_pickle()

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
distance_traversed = []
for i, df in enumerate(dfs_list):
    name = searched_filenames[i]
    x_pos = df['x pos']
    y_pos = df['y pos']
    z_pos = df['z pos']
    
    #compute sum of distances
    dist = 0
    for j in range(len(x_pos)-1):
        dist += np.sqrt((x_pos[j+1]-x_pos[j])**2 + (y_pos[j+1]-y_pos[j])**2)

    distance_traversed.append(dist)

    pd_mean = np.mean(df['pd'])
    pd_std = np.std(df['pd'])
    prob_detection_means.append(pd_mean)
    prob_detection_std.append(pd_std)

    ax.plot(x_pos, y_pos, '-o', label=names[i], 
            color=seaborn_palette[i])


for i, ra in enumerate(radars):

    # plot radar fov
    colors = ['r', 'gray']
    radar_image = plt.Circle((ra.pos.x, ra.pos.y), 
                            ra.radar_range_m, 
                            color=colors[i], fill=False)
    
    ax.plot(ra.pos.x, ra.pos.y, 'ro', label='radar')

    #plot radar pixels
    radar_values = []
    voxel_positions = set()
    for k,v in ra.detection_info.items():
        value = v[0]
        pos = v[1]
        if (pos.x, pos.y) in voxel_positions:
            continue

        voxel_positions.add((pos.x, pos.y))
        radar_values.append(value)
        
    #set color map
    for pos in voxel_positions:
        voxel_image = plt.Rectangle((pos[0], pos[1]),
                                    1, 1, 
                                    color=colors[i], fill=False,
                                    alpha=0.1)
        ax.add_artist(voxel_image)

ax.add_artist(radar_image)
ax.legend()
plt.show()

fig2, ax2 = plt.subplots()
ax2.set_ylim(0, 1)
#plot mean and standard deviation of probability of detection vs 
# distance traversed
for i, (dist, prob) in enumerate(zip(distance_traversed, prob_detection_means)):
    print(searched_filenames[i])
    ax2.annotate(str(round(prob, 2)), (x_pos[i], y_pos[i]))

    #if standard deviation is hits below 0, then set it to 0
    ax2.errorbar(dist, prob, yerr=prob_detection_std[i], fmt='o',
                 label=names[i], color=seaborn_palette[i])    

ax2.legend()    
ax2.set_xlabel('Distance Traversed')
ax2.set_ylabel('Probability of Detection %')
plt.show()

#save figure 
save_fig_dir = '/figures/'
fig2.savefig('prob_detection_vs_dist_traversed.png')
fig2.savefig('prob_detection_vs_dist_traversed.svg')


