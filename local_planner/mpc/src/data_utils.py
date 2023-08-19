import numpy as np

def get_state_control_info(solution):
    """get actual position and control info"""
    control_info = [np.asarray(control[0]) for control in solution]

    state_info = [np.asarray(state[1]) for state in solution]

    return control_info, state_info 

def get_info_history(info: list, n_info: int) -> list:
    """get actual position history"""
    info_history = []

    for i in range(n_info):
        states = []
        for state in info:
            states.append(state[i,0])
        info_history.append(states)

    return info_history

def get_info_horizon(info:list, n_info:int) -> list:
    """get actual position history"""
    info_horizon = []

    for i in range(n_info):
        states = []
        for state in info:
            states.append(state[i, 1:])
        info_horizon.append(states)

    return info_horizon
