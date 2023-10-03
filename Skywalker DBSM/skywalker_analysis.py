import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

import seaborn as sns


if __name__ == '__main__':
    df = pd.read_csv('Skywalker DBSM/skywalker_sig_dbsm.csv',index_col=0)
    
    #take mean of each column
    max_pitch = 80
    min_pitch = -80
    df_mean = df.mean(axis=0)
    
    info_dir = 'info/'
    filename = 'skywalker_sig'
    info_dir = info_dir + filename + '.csv'

    azmiuth_list = np.arange(0,360,1)
    elevation_list = np.arange(min_pitch, max_pitch+1, 1)

    casadi_table = []

    rcs_vals = {}
    for i, az in enumerate(azmiuth_list):
        print(i)
        az_row = []

        rcs_val = df_mean[i] 

        for ele in elevation_list:
            az_row.append(rcs_val)
            key = f"{az}_{ele}"
            az_row.append(rcs_val)
            rcs_vals[key] = rcs_val
            print(az, rcs_val)
            #print(key, rcs_vals[key])

    # rcs_vals = {}
    # for ele in elevation_list:
    #     ele_row = []
    #     for i, az in enumerate(azmiuth_list):
    #         rcs_val = df_mean[i] 
    #         ele_row.append(rcs_val)
    #         key = f"{az}_{ele}"
    #         ele_row.append(rcs_val)
    #         rcs_vals[key] = rcs_val
    #         print(key, rcs_vals[key])

    rcs_hash = pd.DataFrame.from_dict(rcs_vals, orient='index')
    #save to csv
    rcs_hash.to_csv('info/hash/' + filename + '_hash.csv')        