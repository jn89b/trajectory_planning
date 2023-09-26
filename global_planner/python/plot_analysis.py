import pickle as pkl
import numpy as np
from src.DataContainer import SimDataContainer
import matplotlib.pyplot as plt

class PlotSim():
    def __init__(self) -> None:
        pass


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
        plt.show()

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
        plt.show()

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
        plt.show()


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

        plt.show()

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

            # ax.fill_between(sum_distance[i], mean_prob_detection[i] - std_prob_detection[i],
            #                 sum_distance[i], mean_prob_detection[i] + std_prob_detection[i],
            #                 alpha=0.2)
            
        full_title = title_name + " Relationship Path Probability of Detection"
        ax.set_title(full_title)
        ax.legend()

        plt.show()


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
    plotter.plot_rcs(sim_data, "45s")

    plotter.plot_relationship_path_pdetection(sim_data, "45s")
    # plotter.plot_sum_rcs(sim_data, "45s")
    # plotter.plot_prob_detection(sim_data, "45s")
    # plotter.plot_relationship_path_rcs(sim_data, "45s")

