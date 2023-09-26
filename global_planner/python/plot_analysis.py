import pickle as pkl
import numpy as np
from src.DataContainer import SimDataContainer
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


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
                    save:bool=False) -> None:
        
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
            ani.save('data_analysis/45s.gif', writer='imagemagick', fps=1)

        # Show the animation
        plt.show()
        

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

        plt.show()

        return ax

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

        ax = plt.subplot(111)
        ax.set_ylabel("Probability of Detection")
        ax.set_xlabel("Distance")

        mean_prob_detection, std_prob_detection = compute_mean_std_prob_detection(sim_data)
        sum_distance = compute_sum_distance(sim_data)

        for i,rcs_val in enumerate(mean_prob_detection):
            #ax.plot(sum_distance[i], rcs_val, '-o', label=str(sim_data['weights'][i]))
            ax.errorbar(sum_distance[i], mean_prob_detection[i],
                        yerr=std_prob_detection[i], 
                        label=str(sim_data['weights'][i]), linestyle='None', marker='o')
        
        #set light grid
        self.set_light_grid(ax)
        full_title = title_name + " Relationship Path Probability of Detection"
        ax.set_title(full_title)
        ax.legend()

        return ax


def compute_sum_rcs(sim_data:dict):
    sum_rcs = []
    for i,rcs_val in enumerate(sim_data['rcs_vals']):
        sum_rcs.append(sum(rcs_val))
    return sum_rcs

def compute_sum_distance(sim_data:dict):
    sum_distance = []
    for i,path in enumerate(sim_data['paths']):
        x_wp = [x[0] - sim_data['start_position'][0] for x in path] 
        y_wp = [x[1] - sim_data['start_position'][1] for x in path]
        z_wp = [x[2] - sim_data['start_position'][2]for x in path]
        sum_distance.append(sum(x_wp) + sum(y_wp) + sum(z_wp))

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

#%% 

    plotter = PlotSim()
    # rcs_plot = plotter.plot_rcs(sim_data, "45s")
    # rcs_detection_plot = plotter.plot_relationship_path_pdetection(sim_data, "45s")
    # trajectory_plot = plotter.plot3d(sim_data, "45s")
    # plt.show()

    plotter.animate3d(sim_data, "45s", True)
    # plotter.plot_sum_rcs(sim_data, "45s")
    # plotter.plot_prob_detection(sim_data, "45s")
    # plotter.plot_relationship_path_rcs(sim_data, "45s")

