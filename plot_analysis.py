import pickle as pkl
from src.DataContainer import SimDataContainer
import matplotlib.pyplot as plt

class PlotSim():
    def __init__(self) -> None:
        pass


    def plot_rcs(self, sim_data:dict) -> None:
        ax = plt.subplot(111)
        ax.set_ylabel("RCS")
        
        for i,rcs_val in enumerate(sim_data['rcs_vals']):
            ax.plot(rcs_val, '-o', label=str(sim_data['weights'][i]))

        ax.legend()
        plt.show()


if __name__ == '__main__':
    sim_data = pkl.load(open('data_analysis/plane_45s_plane_sig_hash.pkl', 'rb'))

#%% 

    plotter = PlotSim()
    plotter.plot_rcs(sim_data)




