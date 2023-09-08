# -*- coding: utf-8 -*-
"""
Created on Thu Jun 16 08:55:23 2022

@author: abc8cb
"""
import os
import matplotlib.pyplot as plt
from matplotlib import cycler
# from texttable import Texttable
# import latextable
colors = cycler('color',['tab:blue','darkred','#9988DD','#EECC55','#88BB44','#FFBBBB'])
plt.rc('axes', facecolor='white', edgecolor='black', axisbelow=True, grid=True, prop_cycle=colors)
plt.rc('grid', color='silver', linestyle='solid')
plt.rc('xtick', direction='out', color='black')
plt.rc('ytick', direction='out', color='black')
plt.rc('patch', edgecolor='white')
plt.rc('lines', linewidth=1)
plt.rcParams.update({'font.family':'Times New Roman'})
import numpy as np
import math as m
from ConvertData2RCS import *


''' Options '''
# Define Filenames
folder = 'Final Data/'

std_filename = 's25_090823_1deg.csv'
tgt1_filename = 's25_090823 roll right 10d Skywalker_1deg.csv'
tgt2_filename = 's25_090823 roll left 10d Skywalker_1deg.csv'
tgt3_filename = 's25_090823 pitch up 10d Skywalker_1deg.csv'



# tgt2_filename = 's8_baseline_empty_shell_1deg.csv'
# tgt3_filename = 's8_foam_spar_big_servos_1deg.csv'
# tgt4_filename = 's8_foam_only_1deg.csv'

# tgt1_filename = 'High-Res Data/skywalker_x8_no_electronics_1deg.csv'
# tgt2_filename = 'High-Res Data/skywalker_x8_with_electronics_1deg.csv'
# tgt3_filename = 'High-Res Data/skywalker_x8_cone_equipped_1deg.csv'


# tgt1_filename = 's8_mod_wood_t_1deg.csv'
# tgt2_filename = 's8_mod_wood_servos_t_1deg.csv'

# tgt1_filename = 'rebel_1deg.csv'
# tgt2_filename = 'rebel_t_1deg.csv'

# tgt1_filename = 'm2s_u_1deg.csv'
# tgt2_filename = 'm2s_t_1deg.csv'

# tgt1_filename = 's8_edf_+5deg_1deg.csv'
# tgt2_filename = 's8_edf_0deg_1deg.csv'
# tgt3_filename = 's8_edf_-5deg_1deg.csv'

# Save Radar Plots?
# [1 == yes, 0 == no]
save_plots = 1

# Target Name (also name for photo)
# tgt1_name = 'X8, Baseline Measurment'
# tgt2_name = 'X8, No Electronics'
# tgt3_name = 'X8, No Elec. and No Secondary Spars'
# tgt4_name = 'X8, No Servos, No Elec. and No Primary Spars'

tgt1_name = 'Roll right'
tgt2_name = 'Roll left'
tgt3_name = 'Pitch up'


# tgt1_name = 'Baseline'
# # tgt2_name = 'Modified with Nosecone, Wood Primaries'
# tgt3_name = 'Modified with Nosecone, Spars, and Plastic Servos'

# tgt1_name = "RCS [dBsm] at +5\xb0 pitch"
# tgt2_name = "RCS [dBsm] at 0\xb0 pitch"
# tgt3_name = "RCS [dBsm] at -5\xb0 pitch"

# Calibration Sphere Size
size = 0.25 # diameter in meters

''' Convert Data 2 RCS Data '''
angles, tgt1_rcs_m2, tgt1_rcs_dB = ConvertData2RCS(size, folder+std_filename, folder+tgt1_filename)
angles, tgt2_rcs_m2, tgt2_rcs_dB = ConvertData2RCS(size, folder+std_filename, folder+tgt2_filename)
angles, tgt3_rcs_m2, tgt3_rcs_dB = ConvertData2RCS(size, folder+std_filename, folder+tgt3_filename)
# angles, tgt4_rcs_m2, tgt4_rcs_dB = ConvertData2RCS(size, folder+std_filename, folder+tgt4_filename)



fig, ax = plt.subplots(subplot_kw={'projection': 'polar'},dpi=200)
ax.plot(np.deg2rad(angles), tgt1_rcs_m2, label=tgt1_name)
ax.plot(np.deg2rad(angles), tgt2_rcs_m2, label=tgt2_name)
ax.plot(np.deg2rad(angles), tgt3_rcs_m2, label=tgt3_name)
# ax.plot(np.deg2rad(angles), tgt4_rcs_m2, label=tgt4_name)
ax.set_theta_zero_location("N")
ax.set_title('X-Band [8-12 GHz] Average RCS in m$^2$', va='bottom')
plt.legend()
plt.show()

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'},dpi=200)
ax.plot(np.deg2rad(angles), tgt1_rcs_dB, label=tgt1_name)
ax.plot(np.deg2rad(angles), tgt2_rcs_dB, label=tgt2_name)
ax.plot(np.deg2rad(angles), tgt3_rcs_dB, label=tgt3_name)
# ax.plot(np.deg2rad(angles), tgt4_rcs_dB, label=tgt4_name)
ax.set_theta_zero_location("N")
ax.set_ylim(-30,0)
ax.set_title('Average RCS [dBsm]', va='bottom')
ax.legend(loc='lower right',bbox_to_anchor=(1, 0))
# plt.legend()
plt.show()


# fig = plt.figure(dpi=200)
# # # Import image to plot over
# # im = plt.imread("Target Photos/"+name+".jpg")
# #create axes in the background to show cartesian image
# ax0 = fig.add_subplot(111)
# # ax0.imshow(im)
# ax0.axis("off")
# # create polar axes in the foreground and remove its background
# # to see through
# ax = fig.add_subplot(111, polar=True, label="polar")
# ax.set_facecolor("None")
# ax.set_theta_zero_location('N')
# ax.set_theta_direction(-1)
# # ax.set_title('Average RCS (m$^2$) for ' + name + plot_name)
# ax.set_title('Average RCS [m$^2$] for 0.25 m sphere')
# ax.plot(np.deg2rad(angles), tgt_rcs_m2, label='X-Band [8-12 GHz]')
# ax.grid()
# ax.legend(loc=3)
# # if save_plots == 1:
# #     figure = plt.gcf()
# #     figure.set_size_inches(4, 4.5)
# #     plt.savefig(results_dir + name  + plot_name + '.png', dpi = 300)
# plt.show()
