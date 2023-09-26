
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import numpy as np

# Create a figure and a 3D Axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Initialize an empty plot for the animation
line, = ax.plot([], [], [], '-o', fillstyle='none')

# Function to initialize the animation
def init():
    line.set_data([], [])
    line.set_3d_properties([])
    return line,

# Function to update the animation for each frame
def update(frame):
    path = sim_data['paths'][frame]
    x_wp = [x[0] for x in path]
    y_wp = [x[1] for x in path]
    z_wp = [x[2] for x in path]
    line.set_data(x_wp, y_wp)
    line.set_3d_properties(z_wp)
    ax.set_title('RCS Weight ' + str(sim_data['weights'][frame]))
    return line,

# Your sim_data dictionary containing paths and weights
sim_data = {
    'paths': [np.random.rand(10, 3) for _ in range(5)],
    'weights': [1, 2, 3, 4, 5]
}

# Create the animation
ani = FuncAnimation(fig, update, frames=len(sim_data['paths']), init_func=init, blit=True)

# Show the animation
plt.show()


# from casadi import *

# N_refpath = 10
# ref_path_s = np.linspace(0, 1, N_refpath)
# p = MX.sym('p', N_refpath, 1)
# x = MX.sym('x', 1, 1)

# interpol_path_x = casadi.interpolant("interpol_spline_x", "bspline", [ref_path_s])
# interp_exp = interpol_path_x(x, p)
# interp_fun = Function('interp_fun', [x, p], [interp_exp])

# # test interp_fun
# x_test = MX.sym('x_test')
# #p_test = np.linspace(0, 1, N_refpath)
# p_test = MX.sym('p_test', N_refpath)
# y_test = interp_fun(x_test, p_test)
# print(y_test)

# opti = casadi.Opti()

# N = 2
# p = opti.variable(2, N)

