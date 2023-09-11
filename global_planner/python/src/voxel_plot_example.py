import numpy as np
import plotly.graph_objects as go

def cylinder(x, y, z, r, dz):
    """Create a cylindrical mesh located at x, y, z, with radius r and height dz"""
    center_z = np.linspace(0, dz, 15)
    theta = np.linspace(0, 2*np.pi, 15)
    theta_grid, z_grid = np.meshgrid(theta, center_z)
    x_grid = r * np.cos(theta_grid) + x
    y_grid = r * np.sin(theta_grid) + y
    z_grid = z_grid + z
    return x_grid, y_grid, z_grid

def circle(x, y, z, r):
    """Create a circular mesh located at x, y, z with radius r"""
    r_discr = np.linspace(0, r, 2)
    theta_discr = np.linspace(0, 2*np.pi, 15)
    r_grid, theta_grid = np.meshgrid(r_discr, theta_discr)
    x_circle = r_grid * np.cos(theta_grid) + x
    y_circle = r_grid * np.sin(theta_grid) + y
    z_circle = np.zeros_like(x_circle) + z
    return x_circle, y_circle, z_circle

# cylinder mesh
x_cyl, y_cyl, z_cyl = cylinder(0, 0, 0, 2, 8)
# bottom cap
x_circle1, y_circle1, z_circle1 = circle(0, 0, 0, 2)
# top cap
x_circle2, y_circle2, z_circle2 = circle(0, 0, 8, 2)

colorscale = [[0, '#636EFA'],
             [1, '#636EFA']]

fig = go.Figure([
    go.Surface(x=x_cyl, y=y_cyl, z=z_cyl, colorscale=colorscale, showscale=False, opacity=0.5),
    go.Surface(x=x_circle1, y=y_circle1, z=z_circle1, showscale=False, opacity=0.5),
    go.Surface(x=x_circle2, y=y_circle2, z=z_circle2, showscale=False, opacity=0.5),
])
fig.show()