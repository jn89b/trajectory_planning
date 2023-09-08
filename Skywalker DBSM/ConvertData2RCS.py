# -*- coding: utf-8 -*-
"""
Created on Wed Feb 22 14:01:33 2023

@author: CUAV
"""
import pandas as pd
import numpy as np


def ConvertData2RCS(std_diameter,std_filename,tgt_filename):
    ''' Load data '''
    std_data = pd.read_csv(std_filename,index_col=0)
    tgt_data = pd.read_csv(tgt_filename,index_col=0)
    
    
    ''' Determine Number of Angles '''
    std_length = np.shape(std_data)
    tgt_length = np.shape(tgt_data)
    if std_length[0] != tgt_length[0] or std_length[1] != tgt_length[1]:
        print("Data is incompatible! Fix or re-test")
    angle = 360/(std_length[1]-1)
    angles = np.arange(0,360+angle,int(angle))
    
    ''' Convert From df to Array '''
    std_data = std_data.to_numpy()
    tgt_data = tgt_data.to_numpy()
    
        
    ''' Plot the Data '''
    # convert data to RCS
    # Calculate Theoretical calibration sphere RCS
    std_theo_rcs_m2 = np.pi*((std_diameter/2)**2)
    std_theo_rcs_dB = 10*np.log10(std_theo_rcs_m2)
    
    tgt_rcs_m2 = []
    tgt_rcs_dB = []
    
    for i in range(std_length[1]):
        # Calculate Experimental RCS
        tgt_rcs_m2.append((np.abs(np.mean(tgt_data[:,i])/np.mean(std_data[:,i]))**2)*std_theo_rcs_m2)
        tgt_rcs_dB.append(10*np.log10(tgt_rcs_m2[i]))
    return angles, tgt_rcs_m2, tgt_rcs_dB