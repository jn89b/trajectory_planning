# -*- coding: utf-8 -*-
"""
Created on Thu Jun 16 08:55:23 2022

@author: abc8cb
"""
import os
import matplotlib.pyplot as plt
import numpy as np
import math as m
import pandas as pd
import scipy as sp
from tabulate import tabulate
import array
import matplotlib.pyplot as plt
# plt.style.use('tableau-colorblind10')
# plt.rcParams.update({'font.family':'Times New Roman'})
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


# Define y-axis functions for secondary axis
def RCS_dB2m2(RCS_data_in_dB):
    return 10**(RCS_data_in_dB/10)
def RCS_m22dB(RCS_data_in_m2):
    return 10*(np.log10(RCS_data_in_m2))

''' Options '''
# Define Filenames
# background_filename = 'background_20deg.csv'
# folder = 'High-Res Data/'
folder = 'Final Data/'
# std_filename = 's25_wide_gate_1deg.csv'
std_filename = 's25_1deg.csv'

#---------- BASELINES --------------
tgt_filename = 'ANTX_hic_prim_secs_1deg.csv'
# tgt_filename = 's8_mod_wood_t_1deg.csv'
# tgt_filename = 's8_edf_0deg_1deg.csv'
# tgt_filename = 's8_baseline_1deg.csv'
# tgt_filename = 's8_edf_cone_wood_servos_1deg.csv'
# tgt_filename = 's8_edf_cone_wood_servos_1deg.csv'
# tgt_filename = 'rebel_1deg.csv'
# tgt_filename = 'm2s_u_1deg.csv'

# tgt_filename = 's8_with_electronics_1deg.csv'
# tgt_filename = 'Rebel_untreated_1deg.csv'
# tgt_filename = 'Mavic_Air_2s_untreated_1deg.csv'
# std_filename = 'High-Res Data/s15_1deg.csv'
# tgt_filename = 'High-Res Data/s25_1deg.csv'
# std_filename = 'Final Data/s15_1deg.csv'
# tgt_filename = 'Final Data/s25_1deg.csv'

# Save Radar Plots?
# [1 == yes, 0 == no]
save_plots = 0

# Target Name (also name for photo)
name = 'P4'


# Calibration Sphere Size
cal_std_1_dia = 0.25 # diameter in meters

''' Load data '''
std_data = pd.read_csv(folder+std_filename,index_col=0)
tgt_data = pd.read_csv(folder+tgt_filename,index_col=0)


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
std_1_theo_rcs_m2 = np.pi*((cal_std_1_dia/2)**2)
std_1_theo_rcs_dB = 10*np.log10(std_1_theo_rcs_m2)

cal_std_2_dia = 0.25 # diameter in meters

std_2_theo_rcs_m2 = np.pi*((cal_std_2_dia/2)**2)
std_2_theo_rcs_dB = 10*np.log10(std_2_theo_rcs_m2)

tgt_rcs_m2 = []
tgt_rcs_dB = []
cal_diff_matrix = []

for i in range(std_length[1]):
    # Calculate Experimental RCS
    tgt_rcs_m2.append((np.abs(np.mean(tgt_data[:,i])/np.mean(std_data[:,i]))**2)*std_1_theo_rcs_m2)
    cal_diff_matrix.append(tgt_rcs_m2[i]/std_2_theo_rcs_m2)
    tgt_rcs_dB.append(10*np.log10(tgt_rcs_m2[i]))
    
# fig, ax = plt.subplots(subplot_kw={'projection': 'polar'},dpi=200)
# ax.plot(np.deg2rad(angles), tgt_rcs_m2, label='X-Band [8-12 GHz]')
# ax.set_theta_zero_location("N")
# ax.grid(True)
# # ax.set_title('Average RCS [m$^2$] for 0.25 m sphere', va='bottom')
# plt.show()

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'},dpi=200)
ax.set_ylim(-30,0)
ax.plot(np.deg2rad(angles-5), tgt_rcs_dB)#, label="RCS [dBsm]\n at 0\xb0 pitch")
ax.set_theta_zero_location("N")
# labels = array.array(np.linspace(0,360, 5))+array.array(np.linspace(-180,-45, 4))
# # labels = np.array(np.linspace(0,360, 5))+np.array(np.linspace(-180,-45, 4))
# ax.set_xticks(np.deg2rad(np.linspace(0,360, 9)),labels=labels)

ax.grid(True)
ax.legend(loc='lower right')
# ax.set_title('Average RCS [dBsm] for 0.25 m sphere', va='bottom')
plt.show()

# Define New Figure
fig = plt.figure(103,dpi=200)
ax = fig.add_subplot(1,1,1)
# Plot data in time domain
ax.plot(angles-180,tgt_rcs_m2, label='Measured')
# ax.axhline(std_2_theo_rcs_m2, label='Theory', color='darkred')
ax.set_xlim(min(angles-180),max(angles-180))
ax.set_ylim(min(tgt_rcs_m2)-.01,max(tgt_rcs_m2)+.01)
ax.set_title('Test Setup Background RCS Measurement')
ax.set_xlabel('Incident Angle [degrees]')
ax.set_ylabel('RCS [sm]')
ax.set_xticks(np.linspace(-180, 180, 13))
# ax.grid()
ax.legend()

second_ylabel = ax.secondary_yaxis('right', functions=(RCS_m22dB, RCS_dB2m2))
second_ylabel.set_ylabel('RCS [dBsm]')
plt.show()

       

# Save data

RCS_m2_front_pm_30 = np.mean(tgt_rcs_m2[0:30]+tgt_rcs_m2[330:360])
RCS_dB_front_pm_30 = np.mean(tgt_rcs_dB[0:30]+tgt_rcs_dB[330:360])
RCS_front_pm_30_std_lin = np.std(tgt_rcs_m2[0:30]+tgt_rcs_m2[330:360])
RCS_front_pm_30_std_dB = np.std(tgt_rcs_dB[0:30]+tgt_rcs_dB[330:360])

RCS_m2_back_pm_30 = np.mean(tgt_rcs_m2[150:210])
RCS_dB_back_pm_30 = np.mean(tgt_rcs_dB[150:210])
RCS_back_pm_30_std_lin = np.std(tgt_rcs_m2[150:210])
RCS_back_pm_30_std_dB = np.std(tgt_rcs_dB[150:210])

RCS_m2_left_pm_60 = np.mean(tgt_rcs_m2[31:149])
RCS_dB_left_pm_60 = np.mean(tgt_rcs_dB[31:149])
RCS_left_pm_60_std_lin = np.std(tgt_rcs_m2[31:149])
RCS_left_pm_60_std_dB = np.std(tgt_rcs_dB[31:149])

RCS_m2_right_pm_60 = np.mean(tgt_rcs_m2[211:329])
RCS_dB_right_pm_60 = np.mean(tgt_rcs_dB[211:329])
RCS_right_pm_60_std_lin = np.std(tgt_rcs_m2[211:329])
RCS_right_pm_60_std_dB = np.std(tgt_rcs_dB[211:329])

# Create data fields
# header = ['Front \(\pm 30\degree\)','Back \(\pm 30\degree\)','Left \(\pm 60\degree\)','Right \(\pm 60\degree\)']
# index = ['Average RCS (dBsm)','Average RCS (\(m^{2}\))', 'Standard Deviation. (dBsm)', 'Standard Deviation (\(m^{2}\))']
header = ['Front +/- 30 deg','Back +/- 30 deg','Left +/- 60 deg','Right +/- 60 deg']
index = ['Average RCS (dBsm)', 'Standard Deviation (dBsm)','Average RCS (m2)', 'Standard Deviation (m2)']
data = np.array([[RCS_dB_front_pm_30, RCS_dB_back_pm_30, RCS_dB_left_pm_60, RCS_dB_right_pm_60],
                 [RCS_front_pm_30_std_dB, RCS_back_pm_30_std_dB, RCS_left_pm_60_std_dB, RCS_right_pm_60_std_dB],
                 [RCS_m2_front_pm_30, RCS_m2_back_pm_30, RCS_m2_left_pm_60, RCS_m2_right_pm_60],
                 [RCS_front_pm_30_std_lin, RCS_back_pm_30_std_lin, RCS_left_pm_60_std_lin, RCS_right_pm_60_std_lin]])
# data_dB = np.array([[RCS_dB_front_pm_30, RCS_dB_back_pm_30, RCS_dB_left_pm_60, RCS_dB_right_pm_60],        [RCS_front_pm_30_std_dB, RCS_back_pm_30_std_dB, RCS_left_pm_60_std_dB, RCS_right_pm_60_std_dB],
#         [RCS_front_pm_30_std_dB, RCS_back_pm_30_std_dB, RCS_left_pm_60_std_dB, RCS_right_pm_60_std_dB]])

# data_m2 = np.array([[RCS_m2_front_pm_30, RCS_m2_back_pm_30, RCS_m2_left_pm_60, RCS_m2_right_pm_60],
#         [RCS_front_pm_30_std_lin, RCS_back_pm_30_std_lin, RCS_left_pm_60_std_lin, RCS_right_pm_60_std_lin]])


data = np.around(np.transpose(data),decimals=4)

# Put it all in a dictionary
dict = {'label':index,
        header[0]:data[0][0:],
        header[1]:data[1][0:],
        header[2]:data[2][0:],
        header[3]:data[3][0:],}

rcs_data = pd.DataFrame(dict)
print(tabulate(rcs_data, headers = 'keys', tablefmt = 'psql'),'\n \n')

print(rcs_data.to_latex(index=False,caption='RCS Baseline Measurement in dBsm of a Skywalker X8 fixed-wing UAV'),'\n \n')  

folder_path = folder+'Avg Data/'
# save data to file
# rcs_data.to_csv(folder_path+'avg_data_'+tgt_filename)

# d = {'Incident Angle [deg]': angles,'RCS [dBsm]': tgt_rcs_dB}
# df = pd.DataFrame(data=d)

# df.to_csv(folder_path+tgt_filename)




# table_1 = texttable()
# table_1.set_cols_align(["l", "r", "c"])
# table_1.set_cols_valign(["t", "m", "b"])
# table_1.add_rows([["Name", "Age", "Nickname"],
#                  ["Mr\nXavier\nHuon", 32, "Xav'"],
#                  ["Mr\nBaptiste\nClement", 1, "Baby"],
#                  ["Mme\nLouise\nBourgeau", 28, "Lou\n \nLoue"]])
# print('-- Example 1: Basic --')
# print('Texttable Output:')
# print(table_1.draw())
# print('\nLatextable Output:')

# print(latextable.draw_latex(table1, caption="An example table.", label="table:example_table"))

                            
# # Define second subplot
# ax = fig.add_subplot(1,1,1)
# # Plot data in time domain
# ax.plot(angles,tgt_rcs_m2)
# ax.plot(angles,tgt_rcs_dB)

# # ax2.set_xlim(min(time_xaxis),max(time_xaxis))
# # use the peak index to plot red dot
# # ax2.plot(time_xaxis[peak_idx],s12_td_dB[peak_idx],'ro',markersize='4')
# # Plot the window
# # ax2.plot(time_xaxis,s12_td_tg_dB[int(len(freq_swp)/2)+1:len(freq_swp)])
# # apply a hanning window to the time domain data
# # hannwnd = np.hanning()
# # ax2.plot(time_xaxis,hann)
# # ax2.set_title('Time and Distance Domain', y=1.2)
# # ax2.set_xlabel('Time [ns]')
# # Add second axis to second subplot
# second_ylabel = ax.secondary_yaxis('right', functions=(RCS_m22dB, RCS_dB2m2))
# second_ylabel.set_ylabel('RCS [dBsm]')
# plt.show()


# plt.figure(dpi=200)
# plt.plot(angles, tgt_rcs_m2, label='X-Band [8-12 GHz]')

# plt.figure(dpi=200)
# plt.plot(angles, tgt_rcs_dB, label='X-Band [8-12 GHz]')


# fig = plt.figure(dpi=200)
# # # Import image to plot over
# im = plt.imread("Target Photos/"+'Mavic Air 2s'+".jpg")
# #create axes in the background to show cartesian image
# ax0 = fig.add_subplot(111)
# ax0.imshow(im)
# ax0.axis("off")
# # create polar axes in the foreground and remove its background
# # to see through
# ax = fig.add_subplot(111, polar=True, label="polar")
# ax.set_facecolor("None")
# ax.set_theta_zero_location('N')
# ax.set_theta_direction(-1)
# # ax.set_title('Average RCS (m$^2$) for ' + name + plot_name)
# ax.set_title('Average RCS [m$^2$] for 0.25 m sphere')
# ax.plot(np.deg2rad(angles), tgt_rcs_dB, label='X-Band [8-12 GHz]')
# ax.grid()
# # ax.legend(loc=3)
# # if save_plots == 1:
# #     figure = plt.gcf()
# #     figure.set_size_inches(4, 4.5)
# #     plt.savefig(results_dir + name  + plot_name + '.png', dpi = 300)
# plt.show()
