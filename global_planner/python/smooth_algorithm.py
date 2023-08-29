"""
- Given list of wps 
- Given list of obstacles


- Check if:
    - p1 -> p2 has obstacle in its way (visbility check)
    - if so -> run MPC/Optimal Trajectory to find intermediate points
    - Update the list of wps with the intermediate points
    - Repeat until no obstacles in the ways

- Update MPC for error tolerance based on intermediate waypoints

To do:
    - Implement smooth algorithm by using MPC if visibility check fails
    - Smooth algorithm will be one shot Trajectory Optimization 

"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def is_in_obstacle(x:float,y:float, 
                   obs_x:float, obs_y:float, obs_radius:float) -> bool:
    
    dist = np.sqrt((x-obs_x)**2 + (y-obs_y)**2)
    if dist < obs_radius:
        return True
    return False


if __name__ == '__main__':
    
    obs_df = pd.read_csv('obstacles.csv')
    obs_x = obs_df['x'].values
    obs_y = obs_df['y'].values
    obs_radius = obs_df['radius'].values

    obstacles = np.array([obs_x,obs_y,obs_radius]).T

    speed = 40
    waypoints = pd.read_csv('trajectory.csv')

    x_steps = []
    y_steps = []

    # traverse through and detect any visibility issues 
    for i in range(1,len(waypoints)):
        current_x = waypoints['x'].values[i-1]
        current_y = waypoints['y'].values[i-1]
        current_z = waypoints['z'].values[i-1]

        next_x = waypoints['x'].values[i]
        next_y = waypoints['y'].values[i]
        next_z = waypoints['z'].values[i]

        # this is for 2D intermediate points 
        x_points = np.linspace(current_x,next_x,50)
        y_points = np.linspace(current_y,next_y,50)
        
        for x,y in zip(x_points,y_points):

            for obs in obstacles:
                flag_obstacle = False
                if is_in_obstacle(x,y, obs[0], obs[1], obs[2]):
                    print('obstacle in the way at wp', current_x, current_y)
                    print('next wp', next_x, next_y)
                    print('obstacle', obs[0], obs[1], obs[2])
                    flag_obstacle = True
                    break
            
            if flag_obstacle:
                break

        # Check if there is an obstacle in between
        # if so -> add new wp in between
        # if not -> continue

    plt.plot(waypoints['x'].values,waypoints['y'].values, '-o', label='waypoints')
    plt.plot(x_steps,y_steps, '-o', label='intermediate points')

    #plot circle
    for i in range(len(obstacles)):
        circle = plt.Circle((obs_x[i], obs_y[i]), obs_radius[i], color='r', fill=False)
        plt.gcf().gca().add_artist(circle)

    plt.show()
