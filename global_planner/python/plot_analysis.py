import pickle as pkl
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd

from matplotlib.animation import FuncAnimation
from src.DataContainer import SimDataContainer

#interpolate 1d data

#set seaborn style
#sns.set_theme(style="darkgrid")
sns.set_palette("colorblind")


class PlotSim():
    def __init__(self) -> None:
        pass

    def save_plot(self, name_space:str) -> None:
        """
        Save plots in a directory
        """

    def set_light_grid(self, ax:plt.Axes) -> None:
        """
        Set light grid for the plot
        """
        ax.grid(color='lightgray', linestyle='--', linewidth=0.5)

    
    def animate3d(self,sim_data:dict, title_name:str,
                    save:bool=False, fps:int=5) -> None:
        
        fig = plt.figure()
        # set figsize
        fig = plt.figure(figsize=(8, 8))
        # fig.tight_layout()
        ax = fig.add_subplot(111, projection='3d')

        # ax.set_axis_off() # You don't actually need this line as the saved figure will not include the labels, ticks, etc, but I like to include it
        fig.subplots_adjust(left=0, bottom=0, right=0.95, top=0.95)
        #set azmith and elevation        
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_xlim(0, 90)
        ax.set_ylim(0, 90)
        ax.set_zlim(0, 10)

        #plot radar position and direction
        for radar in sim_data['radars']:
            ax.scatter(radar.pos.vec[0], radar.pos.vec[1], radar.pos.vec[2], 
                       c='g', marker='x', label='radar')
            radar_lat_fov_upp_pos = radar.lat_fov_upp_pos.vec
            radar_lat_fov_low_pos = radar.lat_fov_low_pos.vec
            radar_vert_fov_upp_pos = radar.vert_fov_upp_pos.vec
            radar_vert_fov_low_pos = radar.vert_fov_low_pos.vec

            # make quiver smaller
            arrow_length_ratio = 0.1
            length = 0.5
            color = 'g'
            alpha_radar = 0.5
            ax.quiver3D(radar.pos.vec[0], radar.pos.vec[1], radar.pos.vec[2],
                        radar_lat_fov_upp_pos[0],
                        radar_lat_fov_upp_pos[1],
                        radar_lat_fov_upp_pos[2],
                        color=color, label='radar fov', 
                        length=length, 
                        arrow_length_ratio=arrow_length_ratio,
                        alpha=alpha_radar)
            
            ax.quiver3D(radar.pos.vec[0], radar.pos.vec[1], radar.pos.vec[2],
                        radar_lat_fov_low_pos[0],
                        radar_lat_fov_low_pos[1],
                        radar_lat_fov_low_pos[2],
                        color=color,
                        length=length, 
                        arrow_length_ratio=arrow_length_ratio,
                        alpha=alpha_radar)
            
            ax.quiver3D(radar.pos.vec[0], radar.pos.vec[1], radar.pos.vec[2],
                        radar_vert_fov_upp_pos[0],
                        radar_vert_fov_upp_pos[1],
                        radar_vert_fov_upp_pos[2],
                        color=color, 
                        length=length,
                        arrow_length_ratio=arrow_length_ratio,
                        alpha=alpha_radar)
            
            ax.quiver3D(radar.pos.vec[0], radar.pos.vec[1], radar.pos.vec[2],
                        radar_vert_fov_low_pos[0],
                        radar_vert_fov_low_pos[1],
                        radar_vert_fov_low_pos[2],
                        color=color, 
                        length=length,
                        arrow_length_ratio=arrow_length_ratio,
                        alpha=alpha_radar)
                    
        # plot start and goal position
        ax.scatter(sim_data['start_position'][0], 
                   sim_data['start_position'][1], 
                   sim_data['start_position'][2], c='r', marker='x', label='start')

        ax.scatter(sim_data['goal_position'][0],
                     sim_data['goal_position'][1],
                     sim_data['goal_position'][2], c='b', marker='x', label='goal')
        
        num_points = 50
        height = 5
        # plot obstacles as cylinders
        for obstacle in sim_data['obstacles']:
            radius = obstacle.radius_m
            position = obstacle.position.vec
            theta = np.linspace(0, 2 * np.pi, num_points)  # Create points around the circumference
            z = np.linspace(0, height, num_points)         # Create points along the height
            Z, Theta = np.meshgrid(z, theta)               # Create a grid of (z, theta) points
            X = radius * np.cos(Theta) + position[0]                                       
            Y = radius * np.sin(Theta) + position[1]                     
            ax.plot_surface(X, Y, Z, color='gray', alpha=0.2)

        # animate the path
        # Initialize an empty plot for the animation
        line, = ax.plot([], [], [], '-o', fillstyle='none')

        # Function to initialize the animation
        def init():
            line.set_data([], [])
            line.set_3d_properties([])
            return line,

        # Function to update the animation for each frame
        def update(frame):
            self.title = None
            path = sim_data['paths'][frame]
            x_wp = [x[0] for x in path]
            y_wp = [x[1] for x in path]
            z_wp = [x[2] for x in path]
            line.set_data(x_wp, y_wp)
            line.set_3d_properties(z_wp)

            #set title in upper left corner
            ax.set_title('RCS Weight ' + str(sim_data['weights'][frame]))            
            # set title in lower left corner
            # ax.set_title('RCS Weight ' + str(sim_data['weights'][frame]), loc='left')
            return line,
            
        # Create the animation
        # play slower by increasing interval
        ani = FuncAnimation(fig, update, frames=len(sim_data['paths']), 
                            init_func=init, blit=False, interval=1000)
        
        ax.set_box_aspect((1, 1, 1))
        #set aspect ratio equal
        ax.view_init(azim=45, elev=45)
        
        #save as gif
        if save == True:
            ani.save('data_analysis/45s.gif', writer='imagemagick', fps=fps)

        # Show the animation
        plt.show()
        
    def plot2d(self, sim_data:dict, title_name:str,
               save:bool=False) -> None:
        fig = plt.figure()
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")

        for radar in sim_data['radars']:
            ax.scatter(radar.pos.x, radar.pos.y, c='g', marker='x', label='radar')
            radar_lat_fov_upp_pos = radar.lat_fov_upp_pos.vec
            radar_lat_fov_low_pos = radar.lat_fov_low_pos.vec
            radar_vert_fov_upp_pos = radar.vert_fov_upp_pos.vec
            radar_vert_fov_low_pos = radar.vert_fov_low_pos.vec

            # make quiver smaller
            arrow_length_ratio = 0.1
            length = 0.01
            color = 'g'
            ax.quiver(radar.pos.x, radar.pos.y,
                        radar_lat_fov_upp_pos[0],
                        radar_lat_fov_upp_pos[1],
                        color=color, label='radar fov', scale=length)
            
            ax.quiver(radar.pos.x, radar.pos.y,
                        radar_lat_fov_low_pos[0],
                        radar_lat_fov_low_pos[1],
                        color=color, scale=length)        #plot radar pixels
            
        # plot start and goal position
        ax.scatter(sim_data['start_position'][0],
                     sim_data['start_position'][1],
                        c='r', marker='x', label='start')
        
        ax.scatter(sim_data['goal_position'][0],
                        sim_data['goal_position'][1],
                        c='b', marker='x', label='goal')
        
        # plot obstacles as circles
        for obstacle in sim_data['obstacles']:
            ax.add_artist(plt.Circle((obstacle.position.x, obstacle.position.y),
                                     obstacle.radius_m, color='gray', alpha=0.2))
            
        # plot the path
        for i, path in enumerate(sim_data['paths']):
            x_wp = [x[0] for x in path] 
            y_wp = [x[1] for x in path]
            ax.plot(x_wp, y_wp, '-o', fillstyle='none',
                    label='RCS Weight ' + str(sim_data['weights'][i]))
            
        full_title = title_name + " 2D Trajectory"
        ax.set_title(full_title) 
        ax.legend(loc='upper right')

        save = True

        if save == True:
            fig.savefig('data_analysis/' + title_name + '_2d.png')
            #save as svg
            fig.savefig('data_analysis/' + title_name + '_2d.svg')
            

        return ax
        

    def plot3d(self, sim_data:dict, title_name:str,
               save:bool=False) -> None:
        
        fig = plt.figure()
        #set aspect ratio equal

        ax = fig.add_subplot(111, projection='3d')
        #set azmith and elevation
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        #plot radar position and direction
        for radar in sim_data['radars']:
            ax.scatter(radar.pos.vec[0], radar.pos.vec[1], radar.pos.vec[2], 
                       c='g', marker='x', label='radar')
            radar_lat_fov_upp_pos = radar.lat_fov_upp_pos.vec
            radar_lat_fov_low_pos = radar.lat_fov_low_pos.vec
            radar_vert_fov_upp_pos = radar.vert_fov_upp_pos.vec
            radar_vert_fov_low_pos = radar.vert_fov_low_pos.vec

            # make quiver smaller
            arrow_length_ratio = 0.1
            length = 0.5
            color = 'g'
            alpha_radar = 0.5
            ax.quiver3D(radar.pos.vec[0], radar.pos.vec[1], radar.pos.vec[2],
                        radar_lat_fov_upp_pos[0],
                        radar_lat_fov_upp_pos[1],
                        radar_lat_fov_upp_pos[2],
                        color=color, label='radar fov', 
                        length=length, 
                        arrow_length_ratio=arrow_length_ratio,
                        alpha=alpha_radar)
            
            ax.quiver3D(radar.pos.vec[0], radar.pos.vec[1], radar.pos.vec[2],
                        radar_lat_fov_low_pos[0],
                        radar_lat_fov_low_pos[1],
                        radar_lat_fov_low_pos[2],
                        color=color,
                        length=length, 
                        arrow_length_ratio=arrow_length_ratio,
                        alpha=alpha_radar)
            
            ax.quiver3D(radar.pos.vec[0], radar.pos.vec[1], radar.pos.vec[2],
                        radar_vert_fov_upp_pos[0],
                        radar_vert_fov_upp_pos[1],
                        radar_vert_fov_upp_pos[2],
                        color=color, 
                        length=length,
                        arrow_length_ratio=arrow_length_ratio,
                        alpha=alpha_radar)
            
            ax.quiver3D(radar.pos.vec[0], radar.pos.vec[1], radar.pos.vec[2],
                        radar_vert_fov_low_pos[0],
                        radar_vert_fov_low_pos[1],
                        radar_vert_fov_low_pos[2],
                        color=color, 
                        length=length,
                        arrow_length_ratio=arrow_length_ratio,
                        alpha=alpha_radar)
                    
        # plot start and goal position
        ax.scatter(sim_data['start_position'][0], 
                   sim_data['start_position'][1], 
                   sim_data['start_position'][2], c='r', marker='x', label='start')

        ax.scatter(sim_data['goal_position'][0],
                     sim_data['goal_position'][1],
                     sim_data['goal_position'][2], c='b', marker='x', label='goal')
        
        num_points = 50
        height = 5
        # plot obstacles as cylinders
        for obstacle in sim_data['obstacles']:
            radius = obstacle.radius_m
            position = obstacle.position.vec
            theta = np.linspace(0, 2 * np.pi, num_points)  # Create points around the circumference
            z = np.linspace(0, height, num_points)         # Create points along the height
            Z, Theta = np.meshgrid(z, theta)               # Create a grid of (z, theta) points
            X = radius * np.cos(Theta) + position[0]                                       
            Y = radius * np.sin(Theta) + position[1]                     
            ax.plot_surface(X, Y, Z, color='gray', alpha=0.2)
        
        # plot the path
        for i, path in enumerate(sim_data['paths']):
            x_wp = [x[0] for x in path] 
            y_wp = [x[1] for x in path]
            z_wp = [x[2] for x in path]
            ax.plot(x_wp, y_wp, z_wp, '-o', fillstyle='none',
                    label='RCS Weight ' + str(sim_data['weights'][i]))

        full_title = title_name + " 3D Trajectory"
        ax.set_title(full_title)
        #move labels to the side
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0.)

        #set aspect ratio equal
        ax.aspect = 'equal'
        ax.view_init(azim=50, elev=-148)
        
        if save == True:
            fig.savefig('data_analysis/' + title_name + '_3d.png')
            #save as svg
            fig.savefig('data_analysis/' + title_name + '_3d.svg')

        plt.show()

        return ax
    
    def animate_single_trajectory(self, sim_data:dict, title_name:str,
                                  selected_idx:int, save:bool=False,
                                  fps:int=5) -> None:
        """
        Animate one trajectory in 2d 
        """
        fig = plt.figure()
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")

        for radar in sim_data['radars']:
            ax.scatter(radar.pos.x, radar.pos.y, c='g', marker='x', label='radar')
            radar_lat_fov_upp_pos = radar.lat_fov_upp_pos.vec
            radar_lat_fov_low_pos = radar.lat_fov_low_pos.vec
            radar_vert_fov_upp_pos = radar.vert_fov_upp_pos.vec
            radar_vert_fov_low_pos = radar.vert_fov_low_pos.vec

            # make quiver smaller
            arrow_length_ratio = 0.1
            length = 0.01
            color = 'g'
            ax.quiver(radar.pos.x, radar.pos.y,
                        radar_lat_fov_upp_pos[0],
                        radar_lat_fov_upp_pos[1],
                        color=color, label='radar fov', scale=length)
            
            ax.quiver(radar.pos.x, radar.pos.y,
                        radar_lat_fov_low_pos[0],
                        radar_lat_fov_low_pos[1],
                        color=color, scale=length)        #plot radar pixels
            
        # plot start and goal position
        ax.scatter(sim_data['start_position'][0],
                     sim_data['start_position'][1],
                        c='r', marker='x', label='start')
        
        ax.scatter(sim_data['goal_position'][0],
                        sim_data['goal_position'][1],
                        c='b', marker='x', label='goal')
        
        # plot obstacles as circles
        for obstacle in sim_data['obstacles']:
            ax.add_artist(plt.Circle((obstacle.position.x, obstacle.position.y),
                                     obstacle.radius_m, color='gray', alpha=0.2))
            
        path = sim_data['paths'][selected_idx]

        # plot the path
        x_wp = [x[0] for x in path]
        y_wp = [x[1] for x in path]

        def init():
            line.set_data([], [])
            return line,

        def update(frame):
            line.set_data(x_wp[:frame], y_wp[:frame])
            return line,

        line, = ax.plot([], [], '-o', fillstyle='none',
                    label='RCS Weight ' + str(sim_data['weights'][selected_idx]))
        
        full_title = title_name + " 2D Trajectory"
        ax.set_title(full_title)
        ax.legend(loc='upper right')

        ani = FuncAnimation(fig, update, frames=len(x_wp),
                            init_func=init, blit=True, interval=50)
        
        if save == True:
            ani.save('data_analysis/'+full_title+'.gif', writer='imagemagick', fps=fps)
            print("saving gif")


    def plot_rcs(self, sim_data:dict, title_name:str, 
                 save:bool=False) -> None:
        ax = plt.subplot(111)
        ax.set_ylabel("RCS")
        # set color as gradient
        for i,rcs_val in enumerate(sim_data['rcs_vals']):
            ax.plot(rcs_val, '-o', label=str(sim_data['weights'][i]))


        full_title = title_name + " RCS"
        ax.set_title(full_title)
        ax.legend() 

        return ax        

    def plot_sum_rcs(self, sim_data:dict, title_name:str,
                     save:bool=False) -> None:
        """
        plot as a bar chart
        """
        ax = plt.subplot(111)
        ax.set_ylabel("RCS")
        
        sum_rcs = compute_sum_rcs(sim_data)

        for i,rcs_val in enumerate(sum_rcs):
            ax.bar(i, rcs_val, label=str(sim_data['weights'][i]))

        #set x ticks as weights
        ax.set_xticks(range(len(sim_data['weights'])))
        ax.set_xticklabels(sim_data['weights'])

        full_title = title_name + " Sum RCS"
        ax.set_title(full_title)
        # ax.legend()

        return ax 

    def plot_prob_detection(self, sim_data:dict, title_name:str,
                            save:bool=False) -> None:
        
        ax = plt.subplot(111)
        ax.set_ylabel("Probability of Detection")
        # set color as gradient
        for i,rcs_val in enumerate(sim_data['rcs_probs']):
            ax.plot(rcs_val, '-o', label=str(sim_data['weights'][i]))

        full_title = title_name + " Probability of Detection"
        ax.set_title(full_title)
        ax.legend()

        return ax

    def plot_relationship_path_rcs(self, sim_data:dict, title_name:str,
                                   save:bool=False) -> None:
        
        ax = plt.subplot(111)
        ax.set_ylabel("RCS")
        ax.set_xlabel("Distance")

        sum_rcs = compute_sum_rcs(sim_data)
        sum_distance = compute_sum_distance(sim_data)

        for i,rcs_val in enumerate(sum_rcs):
            ax.plot(sum_distance[i], rcs_val, '-o', label=str(sim_data['weights'][i]))

        full_title = title_name + " Relationship Path RCS"
        ax.set_title(full_title)
        ax.legend()

        return ax


    def plot_relationship_path_pdetection(self, sim_data:dict, title_name:str,
                                          save:bool=False) -> None:
        """
        Plot mean and std of probability of detection
        """
        fig = plt.figure()
        ax = plt.subplot(111)
        
        ax.set_ylabel("Probability of Detection %")
        ax.set_xlabel("Distance")

        mean_prob_detection, std_prob_detection = compute_mean_std_prob_detection(sim_data)
        sum_distance = compute_sum_distance(sim_data)

        for i,rcs_val in enumerate(mean_prob_detection):
            #ax.plot(sum_distance[i], rcs_val, '-o', label=str(sim_data['weights'][i]))
            ax.errorbar(sum_distance[i], mean_prob_detection[i],
                        yerr=std_prob_detection[i], 
                        label='RCS Weight ' + str(sim_data['weights'][i]), 
                        linestyle='None', marker='o')
        
        #set light grid
        self.set_light_grid(ax)
        full_title = title_name + " Relationship Path Probability of Detection"
        ax.set_title(full_title)
        ax.legend()

        if save == True:
            fig.savefig('data_analysis/' + full_title + '.png')
            #save as svg
            fig.savefig('data_analysis/' + full_title + '.svg')

        return ax
    
    def animate_rcs_plot(self, df_rcs:pd.DataFrame, sim_data:dict, 
                         title_name:str, weight_idx:int=0,
                         save:bool=False, fps:int=5) -> None:
        
        """
        Animate the RCS plot 
        Get the position of the aircraft 
        Get the location of the radar
        Compute theta 
        With the theta get the RCS value
        animate the RCS value 
        """

        rcs_vals = df_rcs.iloc[8, :]
        angles = np.arange(0, 360+1, 2)

        #interpolate the data from 2 degrees to 1 degree
        angles_interp = np.arange(0, 360+1, 1)
        rcs_vals_interp = np.interp(angles_interp, angles, rcs_vals)

        # Create a polar plot
        plt.figure(figsize=(8, 6))
        fig = plt.figure()
        ax = plt.subplot(111, polar=True)

        # Plot the data as a spider plot    
        ax.plot(angles_interp * np.pi / 180, rcs_vals_interp, 'b', linewidth=1)
        ax.fill(angles_interp * np.pi / 180, rcs_vals_interp, 'b', alpha=0.1)
        
        full_title = title_name + " RCS"
        ax.set_title(full_title, size=11, y=1.1)

        print("plotting rcs")
        #set north to 0 degrees
        ax.set_theta_zero_location("N")

        # Initialize an empty plot for the animation
        line, = ax.plot([], [], 'bo', markersize=5)

        for radar in sim_data['radars']:
            radar_pos = radar.pos.vec

        # need to find the rcs values for each angle
        rcs_sim_vals = []
        rcs_theta_vals = []
        paths = sim_data['paths'][weight_idx]
        dx_wp = [x[0] - radar_pos[0] for x in paths]
        dy_wp = [x[1] - radar_pos[1] for x in paths]
        rel_psi_dg = np.arctan2(dy_wp, dx_wp) * 180 / np.pi
        rel_psi_dg = rel_psi_dg + 90 
        

        for dg in rel_psi_dg:
            if dg > 360:
                dg = dg - 360
            if dg < 0:
                dg = dg + 360

            rcs_sim_vals.append(rcs_vals_interp[int(dg)])
            rcs_theta_vals.append(dg)
    
        # Function to initialize the animation
        def init():
            line.set_data([], [])
            return line,
    
        def update(frame):
            #get the position of the aircraft
            #get the position of the radar
            #compute theta
            #get the rcs value
            #plot the rcs value
            # line.set_data(angles[frame] * np.pi / 180, rcs_vals[frame])
            line.set_data(rcs_theta_vals[frame] * np.pi / 180, rcs_sim_vals[frame])
            return line,
    
        #animate the plot
        anim = FuncAnimation(fig, update, frames=len(rcs_sim_vals), interval=100,
                             init_func=init, blit=True)

        #save as gif
        if save == True:
            anim.save('data_analysis/' + title_name + '_rcs.gif', writer='imagemagick', 
                      fps=fps)
            print("saving gif")
        plt.show()

def get_spider_plot(df_rcs:pd.DataFrame, title_name:str, save:bool=False):
    # get 8th row
    rcs_vals = df_rcs.iloc[8, :]
    angles = np.arange(0, 360+1, 2)

    # Create a polar plot
    plt.figure(figsize=(8, 6))
    fig = plt.figure()
    ax = plt.subplot(111, polar=True)
    ax.set_ylim(-100, 0)
    # Plot the data as a spider plot    
    ax.plot(angles * np.pi / 180, rcs_vals, 'b', linewidth=1)
    ax.fill(angles * np.pi / 180, rcs_vals, 'b', alpha=0.1)
    
    full_title = title_name + " RCS"
    ax.set_title(full_title, size=11, y=1.1)
    ax.set_rlim(-100, 10)

    #set north to 0 degrees
    ax.set_theta_zero_location("N")

    if save == True:
        print("saving to directory")
        fig.savefig('figures/' + title_name + '_rcs.png')
        #save as svg
        fig.savefig('figures/' + title_name + '_rcs.svg')
        
    
    return fig, ax

def compute_sum_rcs(sim_data:dict):
    sum_rcs = []
    for i,rcs_val in enumerate(sim_data['rcs_vals']):
        sum_rcs.append(sum(rcs_val))
    return sum_rcs

def compute_sum_distance(sim_data:dict):
    sum_distance = []
    for i,path in enumerate(sim_data['paths']):
        #compute total distance traveled
        x_wp = [x[0] for x in path]
        y_wp = [x[1] for x in path]
        z_wp = [x[2] for x in path]

        dist = 0
        for j in range(len(x_wp)-1):
            dx = x_wp[j+1] - x_wp[j]
            dy = y_wp[j+1] - y_wp[j]
            dz = z_wp[j+1] - z_wp[j]
            dist += np.sqrt(dx**2 + dy**2 + dz**2)

        sum_distance.append(dist)

    print("sum distance", sum_distance)
    return sum_distance

def compute_mean_std_prob_detection(sim_data:dict):
    mean_prob_detection = []
    std_prob_detection = []
    for i,rcs_val in enumerate(sim_data['rcs_probs']):
        mean_prob_detection.append(np.mean(rcs_val))
        std_prob_detection.append(np.std(rcs_val))
    return mean_prob_detection, std_prob_detection


if __name__ == '__main__':
    sim_data = pkl.load(open('data_analysis/plane_45s_plane_sig_hash.pkl', 'rb'))

#%% Plot the trajectories and results
    plotter = PlotSim()
    # rcs_plot = plotter.plot_rcs(sim_data, "45s")
    # rcs_detection_plot = plotter.plot_relationship_path_pdetection(
    #     sim_data, "45s", True)
    # trajectory_plot_3d = plotter.plot3d(sim_data, "45s", False)
    # trajectory_plot_2d = plotter.plot2d(sim_data, "45s", False)
    
    # plt.show()

    # plotter.animate3d(sim_data, "45s", False)
    # plotter.plot_sum_rcs(sim_data, "45s")
    # plotter.plot_prob_detection(sim_data, "45s")
    # plotter.plot_relationship_path_rcs(sim_data, "45s")

    # single_traj_2d = plotter.animate_single_trajectory(sim_data, "45s", 0, False)

    # ### PLOT RCS
    # df_45 = pd.read_csv('info/plane_45s_plane_sig.csv', header=None)
    # fig, ax = get_spider_plot(df_45, "45s", True)
    # # # plt.show()
    # plotter.animate_rcs_plot(df_45, sim_data, "45s",-1, True)
    plt.show()

#%% Animate the closed set stuff

    # with open('data_analysis/closed_set.pkl', 'rb') as f:
    #     closed_set = pkl.load(f)

    # closed_positions = []
    # for k,v in closed_set.items():
    #     print("position is", v.position.vec)
    #     closed_positions.append(v.position.vec)
    
    # #plot closed set
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.set_xlabel("X")
    # ax.plot3D([x[0] for x in closed_positions],
    #             [x[1] for x in closed_positions],
    #             [x[2] for x in closed_positions], 'o')
    
    # plt.show()
