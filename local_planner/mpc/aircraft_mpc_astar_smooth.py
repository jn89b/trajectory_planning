import casadi as ca
import numpy as np
from src import MPC, Config, data_utils
from time import time
import pandas as pd
"""

Path-following control for small fixed-wing unmanned aerial vehicles under wind disturbances

Included Zurich's equation for psi command 

"""

class AirplaneSimpleModel():
    def __init__(self):
        self.define_states()
        self.define_controls()

    def define_states(self):
        """define the states of your system"""
        #positions ofrom world
        self.x_f = ca.SX.sym('x_f')
        self.y_f = ca.SX.sym('y_f')
        self.z_f = ca.SX.sym('z_f')

        #attitude
        self.phi_f = ca.SX.sym('phi_f')
        self.theta_f = ca.SX.sym('theta_f')
        self.psi_f = ca.SX.sym('psi_f')
        self.u_phi = ca.SX.sym('u_phi')

        self.states = ca.vertcat(
            self.x_f,
            self.y_f,
            self.z_f,
            self.phi_f,
            self.theta_f,
            self.psi_f,
            self.u_phi
        )

        self.n_states = self.states.size()[0] #is a column vector 

    def define_controls(self):
        """controls for your system"""
        self.du_phi = ca.SX.sym('du_phi')
        self.u_theta = ca.SX.sym('u_theta')
        self.u_psi = ca.SX.sym('u_psi')
        self.v_cmd = ca.SX.sym('v_cmd')

        self.controls = ca.vertcat(
            self.du_phi,
            self.u_theta,
            self.u_psi,
            self.v_cmd
        )
        self.n_controls = self.controls.size()[0] 

    def set_state_space(self):
        """define the state space of your system"""
        self.g = 9.81 #m/s^2
        #body to inertia frame
        self.x_fdot = self.v_cmd * ca.cos(self.theta_f) * ca.cos(self.psi_f) 
        self.y_fdot = self.v_cmd * ca.cos(self.theta_f) * ca.sin(self.psi_f)
        self.z_fdot = -self.v_cmd * ca.sin(self.theta_f)
        self.phi_fdot = self.u_phi + self.du_phi
        

        self.du_phi_fdot = self.du_phi    
        self.theta_fdot = self.u_theta
        self.psi_fdot = self.u_psi + (self.g * (ca.tan(self.phi_f) / self.v_cmd))

        self.z_dot = ca.vertcat(
            self.x_fdot,
            self.y_fdot,
            self.z_fdot,
            self.phi_fdot,
            self.du_phi_fdot,
            self.theta_fdot,
            self.psi_fdot)

        #ODE function
        self.function = ca.Function('f', 
            [self.states, self.controls], 
            [self.z_dot])
        
class AirplaneSimpleModelMPC(MPC.MPC):
    def __init__(self, model, dt_val:float, N:int, 
        Q:ca.diagcat, R:ca.diagcat, airplane_params):
        super().__init__(model, dt_val, N, Q, R)

        self.airplane_params = airplane_params

    def add_additional_constraints(self):
        """add additional constraints to the MPC problem"""
        #add control constraints
        self.lbx['U'][0,:] = self.airplane_params['du_phi_min']
        self.ubx['U'][0,:] = self.airplane_params['du_phi_max']

        self.lbx['U'][1,:] = self.airplane_params['u_theta_min']
        self.ubx['U'][1,:] = self.airplane_params['u_theta_max']

        self.lbx['U'][2,:] = self.airplane_params['u_psi_min']
        self.ubx['U'][2,:] = self.airplane_params['u_psi_max']

        self.lbx['U'][3,:] = self.airplane_params['v_cmd_min']
        self.ubx['U'][3,:] = self.airplane_params['v_cmd_max']

        self.lbx['X'][2,:] = self.airplane_params['z_min']
        self.ubx['X'][2,:] = self.airplane_params['z_max']

        self.lbx['X'][3,:] = self.airplane_params['phi_min']
        self.ubx['X'][3,:] = self.airplane_params['phi_max']

        self.lbx['X'][4,:] = self.airplane_params['theta_min']
        self.ubx['X'][4,:] = self.airplane_params['theta_max']

        self.lbx['X'][6,:] = self.airplane_params['u_phi_min']
        self.ubx['X'][6,:] = self.airplane_params['u_phi_max']

    def solve_mpc(self, start, goal, t0=0, sim_time = 5, 
                  set_control=False, init_controls=None):
        """main loop to solve for MPC"""
        #Packing the solution into a single row vector
        print("Solving MPC")
        main_loop = time()  # return time in sec
        n_states = self.n_states
        n_controls = self.n_controls
        
        solution_list = []
    
        self.state_init = ca.DM(start)        # initial state
        self.state_target = ca.DM(goal)  # target state
        self.t0 = t0
        if set_control == False:
            self.u0 = ca.DM.zeros((self.n_controls, self.N))  # initial control
        else:
            controls = ca.DM(init_controls)
            self.u0 = ca.repmat(controls, 1, self.N)

        self.X0 = ca.repmat(self.state_init, 1, self.N+1)         # initial state full
        times = np.array([[0]]) 
        time_history = [self.t0]
        mpc_iter = 0
        obstacle_history = []

        # I need to have this tuned, but it's a good starting point
        # This is good enough
        error_pos = 15

        if Config.OBSTACLE_AVOID:
            obs_x = Config.OBSTACLE_X
            obs_y = Config.OBSTACLE_Y

        while (ca.norm_2(self.state_init[:2] - self.state_target[:2]) >=error_pos) \
            and (self.t0 < sim_time):
            #initial time reference
            t1 = time()

            if Config.MOVING_OBSTACLE:
                obs_x, obs_y = self.move_obstacle(obs_x, obs_y)
                obstacle_history.append((obs_x, obs_y))        

            # self.init_solver()
            # self.compute_cost()

            if Config.OBSTACLE_AVOID:
                """NEEED TO ADD OBSTACLES IN THE LBG AND UBG"""
                # constraints lower bound added 
                lbg =  ca.DM.zeros((self.n_states*(self.N+1)+self.N, 1))
                # -infinity to minimum marign value for obs avoidance  
                lbg[self.n_states*self.N+n_states:] = -ca.inf 
                
                # constraints upper bound
                ubg  =  ca.DM.zeros((self.n_states*(self.N+1)+self.N, 1))
                #rob_diam/2 + obs_diam/2 #adding inequality constraints at the end 
                ubg[self.n_states*self.N+n_states:] = 0 

            elif Config.MULTIPLE_OBSTACLE_AVOID:
                """NEEED TO ADD OBSTACLES IN THE LBG AND UBG"""
                # constraints lower bound added 
                num_constraints = Config.N_OBSTACLES * self.N
                lbg =  ca.DM.zeros((self.n_states*(self.N+1)+num_constraints, 1))
                # -infinity to minimum marign value for obs avoidance  
                lbg[self.n_states*self.N+n_states:] = -ca.inf 
                
                # constraints upper bound
                ubg  =  ca.DM.zeros((self.n_states*(self.N+1)+num_constraints, 1))
                #rob_diam/2 + obs_diam/2 #adding inequality constraints at the end 
                ubg[self.n_states*self.N+n_states:] = 0

            else:
                lbg = ca.DM.zeros((self.n_states*(self.N+1), 1))
                ubg  =  ca.DM.zeros((self.n_states*(self.N+1), 1))

            args = {
                'lbg': lbg,  # constraints lower bound
                'ubg': ubg,  # constraints upper bound
                'lbx': self.pack_variables_fn(**self.lbx)['flat'],
                'ubx': self.pack_variables_fn(**self.ubx)['flat'],
            }
                        
            #this is where you can update the target location
            args['p'] = ca.vertcat(
                self.state_init,    # current state
                self.state_target   # target state
            )
            
            # optimization variable current state
            args['x0'] = ca.vertcat(
                ca.reshape(self.X0, n_states*(self.N+1), 1),
                ca.reshape(self.u0, n_controls*self.N, 1)
            )

            #this is where we solve
            sol = self.solver(
                x0=args['x0'],
                lbx=args['lbx'],
                ubx=args['ubx'],
                lbg=args['lbg'],
                ubg=args['ubg'],
                p=args['p']
            )
            
            t2 = time()

            #unpack as a matrix
            self.u = ca.reshape(sol['x'][self.n_states * (self.N + 1):], 
                                self.n_controls, self.N)
            
            self.X0 = ca.reshape(sol['x'][: n_states * (self.N+1)], 
                                 self.n_states, self.N+1)


            #this is where we shift the time step
            self.t0, self.state_init, self.u0 = self.shift_timestep(
                self.dt_val, self.t0, self.state_init, self.u, self.f)
            
            # self.target = [goal[0], goal[1], self.state_init[2][0]]

            #shift forward the X0 vector
            self.X0 = ca.horzcat(
                self.X0[:, 1:],
                ca.reshape(self.X0[:, -1], -1, 1)
            )
            
            #won't need this for real time system
            solution_list.append((self.u, self.X0))
            
            # xx ...
            times = np.vstack((
                times,
                t2-t1
            ))
            time_history.append(t0+self.t0)

            mpc_iter = mpc_iter + 1

        main_loop_time = time()
        ss_error = ca.norm_2(self.state_init - self.state_target)

        print('\n\n')
        print('Total time: ', main_loop_time - main_loop)
        print('avg iteration time: ', np.array(times).mean() * 1000, 'ms')
        print('final error: ', ss_error)
         
        return time_history, solution_list, obstacle_history          
## start main code
if __name__ == "__main__":

    airplane = AirplaneSimpleModel()
    airplane.set_state_space()

    airplane_params = {
        'u_psi_min': np.deg2rad(-60),
        'u_psi_max': np.deg2rad(60),
        'u_phi_min': np.deg2rad(-60),
        'u_phi_max': np.deg2rad(60),
        'u_theta_min': np.deg2rad(-20),
        'u_theta_max': np.deg2rad(20),
        'z_min': 0.0,
        'z_max': 30.0,
        'v_cmd_min': 15,
        'v_cmd_max': 20,
        'theta_min': np.deg2rad(-10),
        'theta_max': np.deg2rad(10),
        'phi_min': np.deg2rad(-60), 
        'phi_max': np.deg2rad(60),
        'du_phi_min': np.deg2rad(-45),
        'du_phi_max': np.deg2rad(45)
    }  

    Q = ca.diag([1.0, 1.0, 1.0, 0.75, 0.75, 0.75, 0.5])
    R = ca.diag([2.0, 3.0, 1.0, 3.0])


    mpc_airplane = AirplaneSimpleModelMPC(
        model=airplane,
        N=10,
        dt_val=0.1,
        Q=Q,
        R=R,
        airplane_params=airplane_params
    )

    df = pd.read_csv('trajectory.csv')
    x_start = 0
    y_start = 0
    z_start = 0
    theta_start = np.deg2rad(df['theta_dg'].values[0])
    psi_start = np.deg2rad(df['psi_dg'].values[0])

    idx = 1
    x_wps = df['x'].values
    y_wps = df['y'].values
    z_wps = df['z'].values
    theta_wps = np.deg2rad(df['theta_dg'].values)
    psi_wps = np.deg2rad(df['psi_dg'].values)

    x_end = df['x'].values[idx]
    y_end = df['y'].values[idx]
    z_end = df['z'].values[idx]
    theta_end = np.deg2rad(df['theta_dg'].values[idx])
    psi_end = np.deg2rad(df['psi_dg'].values[idx])
        
    start = [x_start, y_start, z_start, 0, theta_start, psi_start]
    goal = [x_end, y_end, z_end, 0, theta_end, psi_end]
    mpc_airplane.init_decision_variables()
    mpc_airplane.reinit_start_goal(start, goal)
    mpc_airplane.compute_cost()
    mpc_airplane.init_solver()
    mpc_airplane.define_bound_constraints()
    mpc_airplane.add_additional_constraints()

    total_time = []
    solution_list = []
    obstacle_history = []
    init_time = 0 
    x_start = x_wps[0]
    y_start = y_wps[0]
    z_start = z_wps[0]
    theta_start = theta_wps[0]
    psi_start = psi_wps[0]
    init_controls = [0,0,0, airplane_params['v_cmd_max']]
    phi_u = 0
    theta_u = 0
    phi_start = 0

    subtract_wps = 0
    for i in range(1, len(x_wps)- subtract_wps):
        x_end = x_wps[i]
        y_end = y_wps[i]
        z_end = z_wps[i]
        theta_end = theta_wps[i]
        psi_end = psi_wps[i]

        start = [x_start, y_start, z_start, phi_start, theta_start, psi_start, phi_u]
        goal =  [x_end,   y_end,   z_end,   phi_start, theta_start, psi_end, 0]
        if i == 1:
            some_time, solution, obstacles = mpc_airplane.solve_mpc(start, goal, 
                                                                    init_time, 10, 
                                                                    True, init_controls)
        else:
            some_time, solution, obstacles = mpc_airplane.solve_mpc(start, goal,
                                                                     0, 10, 
                                                                     True, init_controls)
        total_time.extend(some_time)
        solution_list.extend(solution)
        obstacle_history.extend(obstacles)
        current_control_info, current_state_info = data_utils.get_state_control_info(
            solution)
        current_state_history = data_utils.get_info_history(current_state_info, 
                                                            mpc_airplane.n_states)       
        current_control_history = data_utils.get_info_history(current_control_info,
                                                            mpc_airplane.n_controls)
        init_controls = [current_control_history[0][-1], current_control_history[1][-1],
            current_control_history[2][-1], current_control_history[3][-1]]
        
        x_start = current_state_history[0][-1]
        y_start = current_state_history[1][-1]
        z_start = current_state_history[2][-1]
        phi_start = current_state_history[3][-1]
        theta_start = current_state_history[4][-1]
        psi_start = current_state_history[5][-1]
        phi_u = current_state_history[6][-1]
        # theta_u = current_state_history[7][-1]
        init_time = some_time[-1]

    #%% Data 
    control_info, state_info = data_utils.get_state_control_info(solution_list)
    state_history = data_utils.get_info_history(state_info, mpc_airplane.n_states)
    control_history = data_utils.get_info_history(control_info, mpc_airplane.n_controls)

    #%% Visualize
    import matplotlib.pyplot as plt
    from matplotlib import animation
    import seaborn as sns
    import matplotlib.patches as patches

    plt.close('all')
    
    x_target = goal[0]
    y_target = goal[1]
    
    #%% Plot 2d trajectory
    fig1, ax1 = plt.subplots(figsize=(8,8))
    #set plot to equal aspect ratio
    ax1.set_aspect('equal')
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')

    ax1.plot(state_history[0], state_history[1], 'o-')
    ax1.plot(x_target, y_target, 'x')
    ax1.plot(x_wps, y_wps, 'o-', color='r')

    obs_df = pd.read_csv('obstacles.csv')
    obs_x = obs_df['x'].values
    obs_y = obs_df['y'].values
    obs_z = obs_df['z'].values
    obs_radius = obs_df['radius'].values

    for i in range(len(obs_x)):
        circle = patches.Circle((obs_x[i], obs_y[i]), obs_radius[i]/2, 
            edgecolor='r', facecolor='none')
        ax1.add_patch(circle)
    

    #%% plot 3d trajectory
    fig2 = plt.figure(figsize=(8,8))
    ax2 = fig2.add_subplot(111, projection='3d')

    z_min = min(state_history[2]) - 1
    z_upp = max(state_history[2]) + 1

    ax2.set_xlabel('x')
    ax2.set_ylabel('y')
    ax2.set_zlabel('z')
    ax2.plot(state_history[0], state_history[1], state_history[2], 'o-')
    ax2.plot(x_target, y_target, goal[2], 'x')
    ax2.plot(x_wps, y_wps, z_wps, 'o-', color='r')

    #plot obstacles as cylinders
    if Config.MULTIPLE_OBSTACLE_AVOID:
        for obstacle in Config.OBSTACLES:
            x = obstacle[0]
            y = obstacle[1]
            z = 0
            radius = Config.OBSTACLE_DIAMETER/2
            height = z_upp - z_min
            n_space = 10

            ax2.plot_surface(
                x + radius * np.outer(np.cos(np.linspace(0, 2 * np.pi, n_space)), np.ones(n_space)),
                y + radius * np.outer(np.sin(np.linspace(0, 2 * np.pi, n_space)), np.ones(n_space)),
                z + height * np.outer(np.ones(n_space), np.linspace(0, 1, n_space)),
                color='g',
                alpha=0.2
            )

    ax2.set_zlim(z_min, z_upp)
    #set plot to equal aspect ratio
    # ax2.set_aspect('equal')
    
    #%% plot psi and theta as subplots with time
    #get time 
    fig3, ax3 = plt.subplots(4, 1, figsize=(8,8))
    ax3[0].plot(np.rad2deg(state_history[4]), '-', label='theta')
    ax3[0].plot(np.rad2deg(control_history[1]), '-', label='theta rate command')
    ax3[0].set_xlabel('time [s]')
    ax3[0].set_ylabel('theta')
    ax3[0].legend()

    #plot phi command in fourth subplot
    ax3[1].plot(np.rad2deg(state_history[3]), '-', label='phi')
    ax3[1].plot(np.rad2deg(control_history[0]), '-', label='phi command')
    ax3[1].set_xlabel('time [s]')
    ax3[1].set_ylabel('phi [deg]')
    ax3[1].legend()
    
    ax3[2].plot(np.rad2deg(state_history[5]), '-', label='psi')
    ax3[2].plot(np.rad2deg(control_history[2]), '-', label='psi rate command')
    #draw a line at 360 deg
    #ax3[2].plot([time[0], time[-1]], [360, 360], '--', color='k')
    ax3[2].set_xlabel('time [s]')
    ax3[2].set_ylabel('psi')
    ax3[2].legend()

    #plot velocity command in third subplot
    ax3[3].plot(control_history[3], '-', label='airspeed command')
    ax3[3].set_xlabel('time [s]')
    ax3[3].set_ylabel('Airspeed [m/s]')
    ax3[3].legend()

