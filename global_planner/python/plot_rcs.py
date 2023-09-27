import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

#import animation
from matplotlib.animation import FuncAnimation

def format_spider_plot(locations,categories):
    fig, axis = plt.subplots(subplot_kw={'projection': 'polar'})
    # axis.set_xticks(locations, categories, color='black', size=10)
    
    return fig,axis

df = pd.read_csv('info/plane_90s_plane_sig.csv', header=None)

# get 8th row
rcs_vals = df.iloc[8, :]
angles = np.arange(0, 360+1, 2)

# Create a polar plot
plt.figure(figsize=(8, 6))
fig = plt.figure()
ax = plt.subplot(111, polar=True)

# Plot the data as a spider plot    
ax.plot(angles * np.pi / 180, rcs_vals, 'b', linewidth=1)
ax.fill(angles * np.pi / 180, rcs_vals, 'b', alpha=0.1)
ax.set_title('90s Plane RCS', size=11, y=1.1)

#set north to 0 degrees
ax.set_theta_zero_location("N")

# add dots for animation
def update(frame):
    ax.plot(angles[frame] * np.pi / 180, rcs_vals[frame], 'bo', markersize=5)
    return ax

#animate
anim = FuncAnimation(fig, update, frames=np.arange(0,360,1), interval=30)


# label y axis
ax.set_ylabel('RCS (dBsm)', labelpad=30)
# move yaxis label to the left
ax.yaxis.set_label_position("left")

#tight layout
plt.tight_layout()
# Show the plot
plt.show()

# title_name = '90s_plane_rcs'

# folder_dir = 'data_analysis/'
# save = True
# if save == True:
#     plt.savefig(folder_dir+title_name+'.svg')
#     plt.savefig(folder_dir+title_name+'.png')


