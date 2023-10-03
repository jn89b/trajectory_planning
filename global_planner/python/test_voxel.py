#test out the function 
from src.Raytrace import another_fast_voxel
import plotly.graph_objects as go

if __name__ == "__main__":
    x0 = -10
    y0 = -5
    z0 = 0

    x1 = 20
    y1 = 30
    z1 = 10

    obs_list = []

    rays = another_fast_voxel(x0,y0,z0,x1,y1,z1,obs_list)
    print(rays)

    voxel_x = [x[0] for x in rays]
    voxel_y = [x[1] for x in rays]
    voxel_z = [x[2] for x in rays]
    
    voxel_data = go.Scatter3d(
        x=voxel_x,
        y=voxel_y,
        z=voxel_z,
        opacity=0.2,
        mode='markers',
        name='voxel_data',
        marker=dict(
            color='darkseagreen',
            size=3,
            line=dict(
                color='gold',
                width=0.1
            )
        )
    )


    # marker_data = go.Scatter3d(
    #     x=max_x_data, 
    #     y=max_y_data, 
    #     z=max_z_data,  
    #     opacity=0.5, 
    #     mode='markers',
    #     name='detection_points',
    #     marker=dict(
    #         color='blueviolet',
    #         size=3,
    #         line=dict(
    #             color='gold',
    #             width=0.1
    #         )
    #     )
    # )

    radar_data = go.Scatter3d(
        x=[x0],
        y=[y0],
        z=[z0],
        opacity=1.0,
        mode='markers',
        name='radar',
        marker=dict(
            color='red',
            size=10,
            line=dict(
                color='gold',
                width=1.0
            )
        )
    )

    fig=go.Figure(data=voxel_data)
    # fig.add_trace(voxel_data)
    fig.add_trace(radar_data)

    fig.show()
    fig.write_html("radar_voxel.html")