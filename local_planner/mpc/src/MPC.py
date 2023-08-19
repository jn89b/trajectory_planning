import casadi as ca
import numpy as np
import math as m
from time import time
from src import Config

class MPC():
    def __init__(self, model, dt_val:float, N:int,
            Q:ca.diagcat, R:ca.diagcat):

        print("model: ", model) 
        self.model = model
        self.f = model.function
        self.dt_val = dt_val
        self.N = N

        self.n_states = self.model.n_states
        self.n_controls = self.model.n_controls

        self.Q = Q
        self.R = R

        #cost function
        self.cost_fn = 0

    def init_decision_variables(self):
        #decision variables
        """intialize decision variables for state space models"""
        self.X = ca.SX.sym('X', self.n_states, self.N + 1)
        self.U = ca.SX.sym('U', self.n_controls, self.N)

        #column vector for storing initial and target locations
        self.P = ca.SX.sym('P', self.n_states + self.n_states)

        self.OPT_variables = ca.vertcat(
            self.X.reshape((-1, 1)),   # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
            self.U.reshape((-1, 1))
        )
        
    def define_bound_constraints(self):
        """define bound constraints of system"""
        self.variables_list = [self.X, self.U]
        self.variables_name = ['X', 'U']
        
        #function to turn decision variables into one long row vector
        self.pack_variables_fn = ca.Function('pack_variables_fn', self.variables_list, 
                                             [self.OPT_variables], self.variables_name, 
                                             ['flat'])
        
        #function to turn decision variables into respective matrices
        self.unpack_variables_fn = ca.Function('unpack_variables_fn', [self.OPT_variables], 
                                               self.variables_list, ['flat'], 
                                               self.variables_name)

        ##helper functions to flatten and organize constraints
        self.lbx = self.unpack_variables_fn(flat=-ca.inf)
        self.ubx = self.unpack_variables_fn(flat=ca.inf)

    def add_additional_constraints(self):
        """add additional constraints to system"""
        

    def compute_cost(self):
        #tired of writing self
        #dynamic constraints 
        self.g = []
        self.g = self.X[:,0] - self.P[:self.n_states]
        
        P = self.P
        Q = self.Q
        R = self.R
        n_states = self.n_states
        
        for k in range(self.N):
            states = self.X[:, k]
            controls = self.U[:, k]
            state_next = self.X[:, k+1]
            
            #penalize states and controls for now, can add other stuff too
            self.cost_fn = self.cost_fn \
                + (states - P[n_states:]).T @ Q @ (states - P[n_states:]) \
                + controls.T @ R @ controls                 

            # self.cost_fn =             
            ##Runge Kutta
            k1 = self.f(states, controls)
            k2 = self.f(states + self.dt_val/2*k1, controls)
            k3 = self.f(states + self.dt_val/2*k2, controls)
            k4 = self.f(states + self.dt_val * k3, controls)
            state_next_RK4 = states + (self.dt_val / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            self.g = ca.vertcat(self.g, state_next - state_next_RK4) #dynamic constraints
      
        if Config.OBSTACLE_AVOID:
            for k in range(self.N):
                #penalize obtacle distance
                x_pos = self.X[0,k]
                y_pos = self.X[1,k]                
                obs_distance = ca.sqrt((x_pos - Config.OBSTACLE_X)**2 + \
                                        (y_pos - Config.OBSTACLE_Y)**2)
                

                obs_constraint = -obs_distance + (Config.ROBOT_DIAMETER/2) + \
                    (Config.OBSTACLE_DIAMETER/2)

                self.g = ca.vertcat(self.g, obs_constraint) 

        if Config.MULTIPLE_OBSTACLE_AVOID:
            for obstacle in Config.OBSTACLES:
                obs_x = obstacle[0]
                obs_y = obstacle[1]
                obs_diameter = obstacle[2]

                for k in range(self.N):
                    #penalize obtacle distance
                    x_pos = self.X[0,k]
                    y_pos = self.X[1,k]                
                    obs_distance = ca.sqrt((x_pos - obs_x)**2 + \
                                            (y_pos - obs_y)**2)
                    

                    obs_constraint = -obs_distance + (Config.ROBOT_DIAMETER/2) + \
                        (obs_diameter/2)

                    self.g = ca.vertcat(self.g, obs_constraint)

    def init_solver(self):
        
        nlp_prob = {
            'f': self.cost_fn,
            'x': self.OPT_variables,
            'g': self.g,
            'p': self.P
        }
        

        solver_opts = {
            'ipopt': {
                'max_iter': Config.MAX_ITER,
                'print_level': Config.PRINT_LEVEL,
                'acceptable_tol': Config.ACCEPT_TOL,
                'acceptable_obj_change_tol': Config.ACCEPT_OBJ_TOL,
                # 'linear_solver': 'ma27',
            },
            # 'jit':True,
            'print_time': Config.PRINT_TIME,
            'expand': 1
        }

        #create solver
        self.solver = ca.nlpsol('solver', 'ipopt', 
            nlp_prob, solver_opts)
    
    def reinit_start_goal(self, start, goal):
        self.state_init = ca.DM(start)   # initial state
        self.state_target = ca.DM(goal)  # target state


    def move_obstacle(self, x, y):
        """move obstacle to new position"""
        self.obstacle_x = x + self.dt_val * Config.OBSTACLE_VX
        self.obstacle_y = y + self.dt_val * Config.OBSTACLE_VY 
    
    def shift_timestep(self,step_horizon, t_init, state_init, u, f):
        """
        we shift the time horizon over one unit of the step horizon
        reinitialize the time, new state (which is next state), and 
        the control parameters
        """
        f_value = f(state_init, u[:, 0]) #calls out the runge kutta
        next_state = ca.DM.full(state_init + (step_horizon * f_value))

        #shift time and controls
        next_t = t_init + step_horizon
        next_control = ca.horzcat(
            u[:, 1:],
            ca.reshape(u[:, -1], -1, 1)
        )

        return next_t, next_state, next_control


    def solve_mpc(self, start, goal, t0=0, sim_time = 10):
        """main loop to solve for MPC"""
        #Packing the solution into a single row vector
        main_loop = time()  # return time in sec
        n_states = self.n_states
        n_controls = self.n_controls
        
        solution_list = []
    
        self.state_init = ca.DM(start)        # initial state
        self.state_target = ca.DM(goal)  # target state
        self.t0 = t0
        self.u0 = ca.DM.zeros((self.n_controls, self.N))  # initial control
        self.X0 = ca.repmat(self.state_init, 1, self.N+1)         # initial state full
        times = np.array([[0]]) 
        time_history = [self.t0]
        mpc_iter = 0
        obstacle_history = []

        if Config.OBSTACLE_AVOID:
            obs_x = Config.OBSTACLE_X
            obs_y = Config.OBSTACLE_Y

        while (ca.norm_2(self.state_init - self.state_target) > 1e-1) \
            and (self.t0 < sim_time):
            #initial time reference
            t1 = time()

            if Config.MOVING_OBSTACLE:
                obs_x, obs_y = self.move_obstacle(obs_x, obs_y)
                obstacle_history.append((obs_x, obs_y))        

            self.init_solver()
            self.compute_cost()

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
            
            self.target = [goal[0], goal[1], self.state_init[2][0]]

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
            time_history.append(self.t0)

            mpc_iter = mpc_iter + 1

        main_loop_time = time()
        ss_error = ca.norm_2(self.state_init - self.state_target)

        print('\n\n')
        print('Total time: ', main_loop_time - main_loop)
        print('avg iteration time: ', np.array(times).mean() * 1000, 'ms')
        print('final error: ', ss_error)
         
        return time_history, solution_list, obstacle_history                


        
