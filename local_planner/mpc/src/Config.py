import numpy as np

def create_obstacles(num_obstacles=1, obstacle_diameter=0.5, 
    x_min=0, x_max=10, y_min=0, y_max=10):
    
    """
    Create 2d obstacles in the environment
    """
    obstacles = []
    for i in range(num_obstacles):

        if i == num_obstacles-1:
            x = GOAL_X
            y = GOAL_Y
            obstacles.append([x, y, obstacle_diameter])
            continue

        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        obstacles.append([x, y, obstacle_diameter])

    return obstacles


## START 
START_X = 10
START_Y = 10
START_PSI = np.deg2rad(270)

## GOAL
GOAL_X = 350#750
GOAL_Y = 350#500
GOAL_Z = 25
GOAL_PSI = 0

#### OBSTACLES ####
OBSTACLE_AVOID = False
MOVING_OBSTACLE = False
MULTIPLE_OBSTACLE_AVOID = False

OBSTACLE_X = 10
OBSTACLE_Y = 10
OBSTACLE_DIAMETER = 0.1
OBSTACLE_VX = 0.0
OBSTACLE_VY = 0.0

X_MAX = 90
Y_MAX = 90
X_MIN = 10
Y_MIN = 10

N_OBSTACLES = 10 # +1 for goal
if MULTIPLE_OBSTACLE_AVOID:
    OBSTACLES = create_obstacles(N_OBSTACLES, OBSTACLE_DIAMETER,
        x_min=X_MIN, x_max=X_MAX, y_min=Y_MIN, y_max=Y_MAX) 

ROBOT_DIAMETER = 0.5

#NLP solver options
MAX_ITER = 250
MAX_TIME = 0.15
PRINT_LEVEL = 1
ACCEPT_TOL = 1e-6
ACCEPT_OBJ_TOL = 1e-6
PRINT_TIME = 1

# AIRCRAFT PARAMS
"""
    // from last_letter skywalker_2013/aerodynamics.yaml
        // thanks to Georacer!
        float s = 0.45;
        float b = 1.88;
        float c = 0.24;
        float c_lift_0 = 0.56;
        float c_lift_deltae = 0;
        float c_lift_a = 6.9;
        float c_lift_q = 0;
        float mcoeff = 50;
        float oswald = 0.9;
        float alpha_stall = 0.4712;
        float c_drag_q = 0;
        float c_drag_deltae = 0.0;
        float c_drag_p = 0.1;
        float c_y_0 = 0;
        float c_y_b = -0.98;
        float c_y_p = 0;
        float c_y_r = 0;
        float c_y_deltaa = 0;
        float c_y_deltar = -0.2;
        float c_l_0 = 0;
        float c_l_p = -1.0;
        float c_l_b = -0.12;
        float c_l_r = 0.14;
        float c_l_deltaa = 0.25;
        float c_l_deltar = -0.037;
        float c_m_0 = 0.045;
        float c_m_a = -0.7;
        float c_m_q = -20;
        float c_m_deltae = 1.0;
        float c_n_0 = 0;
        float c_n_b = 0.25;
        float c_n_p = 0.022;
        float c_n_r = -1;
        float c_n_deltaa = 0.00;
        float c_n_deltar = 0.1;
        float deltaa_max = 0.3491;
        float deltae_max = 0.3491;
        float deltar_max = 0.3491;
        // the X CoG offset should be -0.02, but that makes the plane too tail heavy
        // in manual flight. Adjusted to -0.15 gives reasonable flight
        Vector3f CGOffset{-0.15, 0, -0.05};
    } coefficient;

"""
RHO = 1.225 # air density kg/m^3
S = 0.45 # wing area meters squared
B = 1.88 # wing span meters
C = 0.24 # wing chord meters
MASS = 2 # mass kg
C_LIFT_0 = 0.56 # lift coefficient at zero angle of attack
C_LIFT_DELTA_E = 0.0 # lift coefficient at zero angle of attack
C_LIFT_A = 6.9 # lift coefficient at zero angle of attack
C_LIFT_Q = 0.0 # lift coefficient at zero angle of attack
M_COEFF = 50.0 # mass coefficient
OSWALD = 0.9 # oswald efficiency factor
ALPHA_STALL = 0.4712 # stall angle of attack
C_DRAG_Q = 0.0 # drag coefficient due to pitch rate
C_DRAG_DELTA_E = 0.0 # drag coefficient due to elevator deflection
C_DRAG_P = 0.1 # drag coefficient due to roll rate

C_Y_0 = 0.0 # side force coefficient at zero angle of attack
C_Y_B = -0.98 # side force coefficient due to sideslip
C_Y_P = 0.0 # side force coefficient due to roll rate
C_Y_R = 0.0 # side force coefficient due to yaw rate
C_Y_DELTA_A = 0.0 # side force coefficient due to aileron deflection
C_Y_DELTA_R = -0.2 # side force coefficient due to rudder deflection
C_L_0 = 0.0 # rolling moment coefficient at zero angle of attack
C_L_P = -1.0 # rolling moment coefficient due to roll rate
C_L_B = -0.12 # rolling moment coefficient due to sideslip
C_L_R = 0.14 # rolling moment coefficient due to yaw rate
C_L_DELTA_A = 0.25 # rolling moment coefficient due to aileron deflection
C_L_DELTA_R = -0.037 # rolling moment coefficient due to rudder deflection
C_M_0 = 0.045 # pitching moment coefficient at zero angle of attack
C_M_A = -0.7 # pitching moment coefficient due to angle of attack
C_M_Q = -20.0 # pitching moment coefficient due to pitch rate
C_M_DELTA_E = 1.0 # pitching moment coefficient due to elevator deflection
C_N_0 = 0.0 # yawing moment coefficient at zero angle of attack
C_N_B = 0.25 # yawing moment coefficient due to sideslip
C_N_P = 0.022 # yawing moment coefficient due to roll rate
C_N_R = -1.0 # yawing moment coefficient due to yaw rate
C_N_DELTA_A = 0.0 # yawing moment coefficient due to aileron deflection
C_N_DELTA_R = 0.1 # yawing moment coefficient due to rudder deflection
DELTA_A_MAX = 0.3491 # max aileron deflection
DELTA_E_MAX = 0.3491 # max elevator deflection
DELTA_R_MAX = 0.3491 # max rudder deflection
CG_OFFSET = np.array([-0.15, 0.0, -0.05]) # center of gravity offset
