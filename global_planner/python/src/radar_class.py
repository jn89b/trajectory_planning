# -*- coding: utf-8 -*-
"""
Created on Tue Aug 29 16:40:05 2023

@author: mjb7tf
"""
import cmath
import matplotlib.pyplot as plt
import numpy as np

class Plane:
    def __init__(self, sigma, x, y, z):   # Average linear RCS is roughly 0.07 for skywalker
        self.sigma = sigma
        self.x = x
        self.y = y
        self.z = z

class Radar:
    degree_FOV = 120
    degree_elevation = 80
    hz = 10000
    def __init__(self, x, y, c1, c2, degree_azimuth, hz):
        # c1 is the max range given in radar documentation
        # c2 is a contanst per radar system that lets the equation work
        self.x = x
        self.y = y
        self.c1 = c1
        self.c2 = c2
        self.degree_azimuth = degree_azimuth
        self.hz = hz
        
    def compute_instant_detection(self, airplane):
        distance = cmath.sqrt(pow(airplane.x, 2) + pow(airplane.y, 2) + pow(airplane.z, 2))
        distance_squared = pow(distance, 4)
        instant_detection = ( 1 / (1 + (pow(((self.c2 * distance_squared)/ airplane.sigma)  , self.c1))))
        
        return instant_detection
    
    def compute_probability_detection(self, instant_detection_prob):
        probability_detection = 1- pow(instant_detection_prob , self.hz)
        
        return probability_detection
        

        
def compute():
    Radar1 = Radar(0, 0, -.3, 1200, 0, 10000)
    p_detection1 = []
    sky_walker1_pos = []
    plt.figure()
    for x in range(2000):
        Skywalker1 = Plane(0.07, x, 0, 30)
        distance = cmath.sqrt(pow(Skywalker1.x, 2) + pow(Skywalker1.y, 2) + pow(Skywalker1.z, 2))
        distance_squared = pow(distance, 4)
        instant_detection = ( 1 / (1 + (pow(((Radar1.c2 * distance_squared)/ Skywalker1.sigma)  , Radar1.c1))))
        probability_detection = 1- pow(instant_detection  , Radar1.hz)
        p_detection1.append(probability_detection)
        sky_walker1_pos.append(Skywalker1.x)
        
    # print(p_detection1[-1])
        
    plt.plot(sky_walker1_pos, p_detection1, '-', color = 'r', label='X-8 Skywalker')
    plt.legend()
    plt.xlabel('Range (m)')
    plt.ylabel('Chance of Detection')
    plt.title('Probability of Detecting Aircraft')

def changing_c1():
     Skywalker1 = Plane(0.000278, 0, 0, 30)
     
     
     c1_array = np.arange(-0.3, -0.2, 0.01)
     
     
     overall_p_detection = []
     
     for c1_val in c1_array:
         Radar1 = Radar(x=0, y=0, c1=c1_val, c2=1200, degree_azimuth=0, hz=10000)
         x_position = 0
         p_detection = []
         sky_walker_position = []
         Skywalker1.x = x_position
         for x in range(150):
             Skywalker1.x = x
             instant_detection = Radar1.compute_instant_detection(Skywalker1)
             p_detection.append(Radar1.compute_probability_detection(instant_detection))
             sky_walker_position.append(Skywalker1.x)    
         overall_p_detection.append(p_detection)
         
         
     fig1, ax1 = plt.subplots()
     for c1_val, p_detection_vals in zip(c1_array,overall_p_detection):
         ax1.plot(sky_walker_position, p_detection_vals, label=str(c1_val))
         plt.xlabel('Range (m)')
         plt.ylabel('Percent chance of detection')
         plt.title('EchodyneCR Detection Probability W/ Changing C1 Value')
     
     ax1.legend()
     
def changing_rcs():
     
     
     rcs_array = np.arange(0.1, 0.07, 0.01)
     Radar1 = Radar(0, 0, -.3, 1200, 0, 10000)
     
     overall_p_detection = []
     
     for sig in rcs_array:
         Skywalker1 = Plane(sig, 0, 0, 30)
         x_position = 0
         p_detection = []
         sky_walker_position = []
         Skywalker1.x = x_position
         for x in range(2000):
             Skywalker1.x = x
             instant_detection = Radar1.compute_instant_detection(Skywalker1)
             p_detection.append(Radar1.compute_probability_detection(instant_detection))
             sky_walker_position.append(Skywalker1.x)    
         overall_p_detection.append(p_detection)
         
         
     fig1, ax1 = plt.subplots()
     for sig, p_detection_vals in zip(sig,overall_p_detection):
         ax1.plot(sky_walker_position, p_detection_vals, label=str(sig))
         plt.xlabel('Range (m)')
         plt.ylabel('Percent chance of detection')
         plt.title('EchodyneCR Detection Probability W/ Changing C1 Value')
     
     ax1.legend()

if __name__=='__main__':
    

    # compute()
    changing_c1()
    plt.show()
    # changing_rcs()
    

    
        
        
    
    


