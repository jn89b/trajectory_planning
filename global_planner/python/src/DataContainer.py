"""
Stores data for the global planner
"""
import pickle as pkl

class SimDataContainer() :
    def __init__(self) -> None:
        self.sim_results = {
            'paths': None,   
            'weights': None,
            'radars': None,
            'rcs_vals': None,
            'rcs_probs': None,
            'obstacles': None,
            'start_position': None,
            'goal_position': None,
            'grid': None,
        }

    def pickle_data(self, dirname:str) -> None:
        pkl.dump(self.sim_results, open(dirname, 'wb'))
        print("Data saved to", dirname)

    def load_data(self, dirname:str) -> None:
        self.sim_results = pkl.load(open(dirname, 'rb'))
        print("Data loaded from", dirname)

