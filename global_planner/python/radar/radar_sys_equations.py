# -*- coding: utf-8 -*-
"""
Created on Fri Sep 22 15:55:47 2023

@author: mjb7tf
"""

import numpy as np 

def compute_cummaltive_pd(radars_pd:list) -> float:
    pdj_array = np.array(radars_pd)
    cumm_pdj_array = np.prod(pdj_array)
    return cumm_pdj_array 

def compute_other_cummalitive_pd(radars_pd:list) -> float:
    other_cumm_pdj = []
    for pd in radars_pd:
        difference = (1- pd)
        other_cumm_pdj.append(difference)
    other_cumm_array = np.prod(np.array(other_cumm_pdj))
    return other_cumm_array
    
R = 1

radars_Pd = [0.7, 0.8, 0.9]
radars_decision = [1,0,1,1]

if R == 1:
    cumm_prob = compute_cummaltive_pd(radars_Pd) * np.sqrt(len(radars_Pd))
else:
    other_cumm_prop = compute_other_cummalitive_pd(radars_Pd)


