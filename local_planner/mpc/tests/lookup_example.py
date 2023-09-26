"""
Easier formulation of MPC without the crazy stuff
"""
from time import time 
import casadi as ca
import numpy as np
import pandas as pd


"""
Load csv hash file
Link the hash file to the lookup table

In MPC:
    init the rcs_sum
    for loop:
        look up rcs value
        append to rcs_sum

    In cost function look to minimize rcs_sum

"""


class ToyCar():
    """
    Toy Car Example 
    
    3 States: 
    [x, y, psi]
     
     2 Inputs:
     [v, psi_rate]
    
    """
    def __init__(self):
        self.define_states()
        self.define_controls()
        
    def define_states(self):
        self.x = ca.SX.sym('x')
        self.y = ca.SX.sym('y')
        self.psi = ca.SX.sym('psi')
        
        self.states = ca.vertcat(
            self.x,
            self.y,
            self.psi,
        )
        #column vector of 3 x 1
        self.n_states = self.states.size()[0] #is a column vector 
        
    def define_controls(self):
        self.v_cmd = ca.SX.sym('v_cmd')
        self.psi_cmd = ca.SX.sym('psi_cmd')
        
        self.controls = ca.vertcat(
            self.v_cmd,
            self.psi_cmd
        )
        #column vector of 2 x 1
        self.n_controls = self.controls.size()[0] 
        
    def set_state_space(self):
        #this is where I do the dynamics for state space
        self.x_dot = self.v_cmd * ca.cos(self.psi)
        self.y_dot = self.v_cmd * ca.sin(self.psi)
        self.psi_dot = self.psi_cmd 

        
        self.z_dot = ca.vertcat(
            self.x_dot, self.y_dot, self.psi_dot    
        )
        
        #ODE right hand side function
        self.function = ca.Function('f', 
                        [self.states, self.controls],
                        [self.z_dot]
                        ) 
        
        return self.function
    

def compute_cost(obstacle_x, obstacle_y, opti):
    """
    REFACTOR THIS
    The problem right now is it keeps adding more constraints 
    for each iteration, need to reinitialize or empty the constraints
    
    """
    #margin cost function for obstacle avoidance
    # safe_cost = opti.variable(N)
    
    #might do an inflation
    obstacle_distance = ca.sqrt((obstacle_x - x_pos)**2 + \
        (obstacle_y - y_pos)**2)
    
    # #inequality constraint for obstacle avoidance
    opti.subject_to(obstacle_distance[0:-1].T \
        >= safe_cost + radius_obstacle+safe_margin)
            
    # cost function
    e_x = (goal_x - x_pos)
    e_y = (goal_y - y_pos)
    e_z = (goal_psi - psi)
    
    weights = [1, 1, 1]

    cost_value = (weights[0]*ca.sumsqr(e_x)) + (weights[1]*ca.sumsqr(e_y)) + \
        (weights[2]*ca.sumsqr(e_z))  + \
            lambda_obst*ca.sumsqr(safe_cost) +  ca.sumsqr(U)    

    opti.minimize(cost_value)    
    
    return opti

    
if __name__ == '__main__':
    
    #limits 
    v_max = 20
    v_min = 0
    
    psi_rate_max = np.deg2rad(45)
    psi_rate_min = -np.deg2rad(45)
    
    #init locations
    init_x = 0.0
    init_y = 0.0
    init_psi = 0.0
    init_vector = [init_x, init_y, init_psi]
    
    obstacle_x = 40
    obstacle_y = 40
    
    #goal locations
    goal_x = 80
    goal_y = 80
    goal_psi = np.deg2rad(20)
    goal_vector = [goal_x, goal_y , goal_psi]
    
    #state model
    toy_car = ToyCar()
    f = toy_car.set_state_space()
    
    """
    Data is formatted as follows:
    rows are azimuth from 0 to 360 
    columns are elevation from -80 to 80
    80th column is elevation at 0 degrees
    """
    mpc_table = pd.read_csv('plane_sig_mpc_table.csv', header=None)
    #convert to numpy array
    mpc_table = mpc_table.values
    #remove first row and first column
    mpc_table = mpc_table[1:, 1:]

    mpc_array = mpc_table[:,79]

    #convert to numpy array
    
    # mpc_array = np.asarray(mpc_table, dtype=np.float64)
    
    mpc_array = np.asarray(mpc_array, dtype=np.float64)
    #flatten the array
    # mpc_array = mpc_array.flatten()

    # N_refpath = 10
    # ref_path_s = np.linspace(0, 1, N_refpath)
    # p = ca.MX.sym('p', N_refpath, 1)
    # x = ca.MX.sym('x', 1, 1)

    elev_values_dg = np.arange(-80, 81, 1)
    az_values_dg = np.arange(0, 361, 1)    

    # azmith_angle = ca.MX.sym('azmith_angle', len(az_values_dg), 1)
    # elevation_angle = ca.MX.sym('elevation_angle', len(elev_values_dg), 1)
    azmith_angle = ca.MX.sym('azmith_angle', 1)
    elevation_angle = ca.MX.sym('elevation_angle', 1)

    interpol_path_x = ca.interpolant("name", "bspline", [az_values_dg], mpc_array)
    inter_exp = interpol_path_x(azmith_angle)
    inter_fun = ca.Function('inter_fun', [azmith_angle], [inter_exp])
    print(inter_fun(45))

    radar_pos_x = 50
    radar_pos_y = 50

    #%% Optimizatoin 
    #horizon time and dt value, can make basis on how fast I'm localizing
    N = 25
    dt_val = 0.1
    opti = ca.Opti()
    dt_val = 0.1 
    t_init = 0.0
    
    radius_obstacle = 2.0
    safe_margin = 0.5
    lambda_obst = 25
    
    #decision variables to send to NLP
    X = opti.variable(toy_car.n_states, N+1) #allow one more iteration for final
    x_pos = X[0,:]
    y_pos = X[1,:]
    psi = X[2,:]
    
    U = opti.variable(toy_car.n_controls, N) # U decision variables
    u_vel = U[0,:]
    u_psi_rate = U[1,:]
    
    ## initial parameters 
    #Initiate State parameter
    x0 = opti.parameter(toy_car.n_states)    
    xF = opti.parameter(toy_car.n_states)

    #set initial value
    opti.set_value(x0, [init_x, init_y, init_psi])
    opti.set_value(xF, [goal_x, goal_y, goal_psi])
        
    #intial and terminal constraints 
    opti.subject_to(X[:,0] == x0)
    opti.subject_to(opti.bounded(psi_rate_min, u_psi_rate, psi_rate_max))
    opti.subject_to(opti.bounded(v_min, u_vel, v_max))
    
    #g constraints dynamic constraints
    rcs = opti.variable(N)
    for k in range(N):
        states = X[:, k]
        controls = U[:, k]
        state_next =X[:, k+1]

        dy = states[1] - radar_pos_y
        dx = states[0] - radar_pos_x

        c2 = 0.0001
        c1 = 1.5
        radar_fq_hz = 1000
        dist = ca.sqrt(dy**2 + dx**2)

        rel_psi_dg = ca.atan2(dy, dx) * 180/np.pi
        rcs_val = inter_fun(rel_psi_dg)
    
        linear_db = 10**(rcs_val/10) 
        radar_prob_detection = 1/(1 +(c2* np.power(dist,4) / linear_db)**c1)
        probability_detection = 1- pow(radar_prob_detection , radar_fq_hz)

        rcs[k] = probability_detection

        k1 = f(states, controls)
        k2 = f(states + dt_val/2*k1, controls)
        k3 = f(states + dt_val/2*k2, controls)
        k4 = f(states + dt_val * k3, controls)
        state_next_RK4 = states + (dt_val / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
        opti.subject_to(X[:, k+1] == state_next_RK4)
        
    #margin cost function for obstacle avoidance
    safe_cost = opti.variable(N)
    
    #might do an inflation
    obstacle_distance = ca.sqrt((obstacle_x - x_pos)**2 + \
        (obstacle_y - y_pos)**2)
    
    #inequality constraint for obstacle avoidance
    opti.subject_to(obstacle_distance[0:-1].T \
        >= safe_cost + radius_obstacle+safe_margin)
                
    # cost function
    e_x = (goal_x - x_pos)
    e_y = (goal_y - y_pos)
    e_z = (goal_psi - psi)
    
    weights = [1, 1, 1]

    cost_value = (weights[0]*ca.sumsqr(e_x)) + (weights[1]*ca.sumsqr(e_y)) + \
        (weights[2]*ca.sumsqr(e_z))  + \
            lambda_obst*ca.sumsqr(safe_cost) +  ca.sumsqr(U) + (0*ca.sumsqr(rcs))

    opti.minimize(cost_value)
    opts = {
        'ipopt': {
            'max_iter': 2000,
            'print_level': 1,
            'acceptable_tol': 1e-2,
            'acceptable_obj_change_tol': 1e-2,
        },
        'print_time': 1
    }
    opti.solver('ipopt', opts)#, {'ipopt': {'print_level': 0}})

    #Initial Trajectory    
    sol = opti.solve()

    t_simulate = np.arange(t_init, N*dt_val, dt_val)
    x_traj = sol.value(X)
    u_traj = sol.value(U)
    
    #%% MPC with the opti stuff
    def shift_time_step(step_horizon, t_init, state_init, u,f):
        # u_init = u[:,0]
        f_value = f(state_init, u[:,0]) #calls out the runge kutta
        next_state = ca.DM.full(state_init + (step_horizon * f_value))

        #shift time and controls
        next_t = t_init + step_horizon
        return next_t, next_state
    
    tolerance = 0.1
    t_init = 0.0
    sim_time = 5.0 #seconds
    solution_list = []
    times = np.array([[0]]) 
    main_loop = time()  # return time in sec
    
    time_history = [t_init]
    #time_history = [self.t0]

    ### Begin MPC ### 
    while (np.linalg.norm(np.array(init_vector) - np.array(goal_vector)) > tolerance) \
        and (t_init < sim_time):


        t1 = time()
        opti = compute_cost(obstacle_x, obstacle_y, opti)
        sol = opti.solve()


        main_loop_time = time()
        t2 = time()
        
        #reinitialize here
        t_init, init_vector = shift_time_step(dt_val, t_init, init_vector, u_traj, f)
        x_traj = sol.value(X)
        u_traj = sol.value(U)

        solution_list.append((u_traj, x_traj))

        opti.set_value(x0, init_vector)
        time_history.append(t_init)
        print("sim time ", t_init)
        
    #print('Current time: ', main_loop_time - main_loop)
    ss_error = np.linalg.norm(np.array(init_vector) - np.array(goal_vector))
    print('final error: ', ss_error)
    print('Total time: ', main_loop_time - main_loop)

    
#%% Animate
    
    import matplotlib.pyplot as plt
    import seaborn as sns
    from matplotlib import animation

    plt.close('all')


    #for control_info each index is length [control x N]
    #for state_info each index is length [state x N+1]
    control_info = [control[0] for control in solution_list]
    state_info = [state[1] for state in solution_list]

    actual_x = []
    actual_y = []
    actual_psi = []

    horizon_x = []
    horizon_y = []
    horizon_psi = []

    for state in state_info:
        state = np.asarray(state)
        
        actual_x.append(state[0,0])
        actual_y.append(state[1,0])
        actual_psi.append(state[2,0])
        
        
        horizon_x.append(state[0,1:])
        horizon_y.append(state[1,1:])
        horizon_psi.append(state[2,1:])
    
    overall_horizon_x = []
    overall_horizon_y = []
    overall_horizon_z = []
    
    for x,y,z in zip(horizon_x, horizon_y, horizon_psi):
        overall_horizon_x.extend(x)
        overall_horizon_y.extend(y)        
        overall_horizon_z.extend(z)

    obstacle = plt.Circle( (obstacle_x, obstacle_y ),
                                        radius_obstacle ,
                                        fill = True )

    fig1, ax1 = plt.subplots(figsize=(8,8))
    ax1.plot(actual_x, actual_y)
    ax1.plot(goal_x, goal_y, 'x')
    

    """format position"""
    actual_pos_array = np.array([actual_x, actual_y])
    horizon_pos_array = np.array([overall_horizon_x, overall_horizon_y])
    
    buffer = 2
    min_x = min(actual_x)
    max_x = max(actual_x)
    
    min_y = min(actual_y)
    max_y = max(actual_y)
    
    TIME_SPAN = 5
    position_data = [actual_pos_array, horizon_pos_array]
    labels = ['Actual Position', 'Horizon']



    fig2, ax2 = plt.subplots(figsize=(6, 6))
    ax2.add_patch(obstacle)
    
    #ax2.plot(obstacle_x, obstacle_y, 'o', markersize=10, label='obstacle')
    # ax2.plot(actual_x, actual_y)
    
    ax2.set_xlim([min_x-buffer, max_x+buffer])
    ax2.set_ylim([min_y-buffer, max_y+buffer])
    
    ax2.plot(init_x, init_y, 'x', markersize=10, label='start')
    ax2.plot(goal_x, goal_y, 'x', markersize=10,label='end')
    
    lines = [ax2.plot([], [], [])[0] for _ in range(len(position_data))] 
    # line2, = ax2.plot(horizon_pos_array[0,0], 
    #                   horizon_pos_array[1,0],
    #                   horizon_pos_array[2,0])
    
    color_list = sns.color_palette("hls", len(position_data))
    # ax.scatter(5,5,1, s=100, c='g')
    for i, line in enumerate(lines):
        line._color = color_list[i]
        #line._color = color_map[uav_list[i]]
        line._linewidth = 2.0
        line.set_label(labels[i])
        
    patches = lines

    def init():
        #lines = [ax2.plot(uav[0, 0:1], uav[1, 0:1], uav[2, 0:1])[0] for uav in position_data]
        lines = [ax2.plot(uav[0, 0:1], uav[1, 0:1])[0] for uav in position_data]

        #scatters = [ax.scatter3D(uav[0, 0:1], uav[1, 0:1], uav[2, 0:1])[0] for uav in data]

        return patches
    
    def update_lines(num, dataLines, lines):
        count = 0 
        for i, (line, data) in enumerate(zip(lines, dataLines)):
            #NOTE: there is no .set_data() for 3 dim data...
            time_span = TIME_SPAN
            if num < time_span:
                interval = 0
            else:
                interval = num - time_span
                
            if i == 1:
                line.set_data(data[:2, N*num:N*num+N])
                # line.set_3d_properties(data[2, N*num:N*num+N])
            else:
                
                line.set_data(data[:2, interval:num])
                # line.set_3d_properties(data[2, interval:num])
            
            count +=1
        
        return patches
    



