import numpy as np
import casadi as ca

def rot2d(psi):
    return np.array([[np.cos(psi), -np.sin(psi)],
                     [np.sin(psi), np.cos(psi)]])


def rot2d_casadi(psi):
    return ca.vertcat(
        ca.horzcat(ca.cos(psi), -ca.sin(psi)),
        ca.horzcat(ca.sin(psi), ca.cos(psi))
    )


#rotation 3d
def rot3d(roll, pitch, yaw):
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    R = np.dot(R_x, np.dot(R_y, R_z))
    return R

def rot3d_casadi(roll, pitch, yaw):
    R_x = ca.vertcat(
        ca.horzcat(1, 0, 0),
        ca.horzcat(0, ca.cos(roll), -ca.sin(roll)),
        ca.horzcat(0, ca.sin(roll), ca.cos(roll))
    )

    R_y = ca.vertcat(
        ca.horzcat(ca.cos(pitch), 0, ca.sin(pitch)),
        ca.horzcat(0, 1, 0),
        ca.horzcat(-ca.sin(pitch), 0, ca.cos(pitch))
    )

    R_z = ca.vertcat(
        ca.horzcat(ca.cos(yaw), -ca.sin(yaw), 0),
        ca.horzcat(ca.sin(yaw), ca.cos(yaw), 0),
        ca.horzcat(0, 0, 1)
    )

    R = ca.mtimes(R_z, ca.mtimes(R_y, R_x))
    return R


   
# #example of rot2d_casadi 
# ref_angle = ca.SX.sym('psi')
# #test = rot2d_casadi(psi)

# ref_point = ca.vertcat(0, 0)
 
# effector_profile = np.array([[0, 0],
#                                 [1, 0],
#                                 [0, 1]])

# test = (rot2d_casadi(ref_angle) @ effector_profile)
# example of using rot3d 

# ref_roll = np.deg2rad(0)
# ref_pitch = np.deg2rad(270)
# ref_yaw = np.deg2rad(0)

# ref_point = np.array([0, 0, 0])
# ref_point = ref_point.reshape(3, 1)

# effector_profile = np.array([
#                              [0, 0, 0],
#                              [2, 0, 0],
#                              [0,2,0]])



# test = (rot3d(ref_roll, ref_pitch, ref_yaw) @ np.transpose(effector_profile)) + ref_point
# print(test)

# #check if distance between ref_point and effector_profile is the same
# print(np.linalg.norm(ref_point - effector_profile[0]))
# print(np.linalg.norm(ref_point - test[0]))



# #plot the effector profile
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# plt.close('all')

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# ax.plot(effector_profile[:,0], effector_profile[:,1], effector_profile[:,2],
#     c='r', marker='o' , label='effector profile origin')
# ax.plot(test[0], test[1], test[2], c='b', marker='o', label='rotated effector profile')

# ax.scatter(ref_point[0], ref_point[1], ref_point[2], label='origin')

# ax.set_xlabel('X Label')
# ax.set_ylabel('Y Label')
# ax.set_zlabel('Z Label')
# ax.set_title('Ref roll: {}, Ref pitch: {}, Ref yaw: {}'.format(np.rad2deg(ref_roll), 
#     np.rad2deg(ref_pitch), np.rad2deg(ref_yaw)))

# #set the same scale for all axis
# max_range = np.array([effector_profile[:,0].max()-effector_profile[:,0].min(),
#     effector_profile[:,1].max()-effector_profile[:,1].min(),
#     effector_profile[:,2].max()-effector_profile[:,2].min()]).max() / 2.0

# mid_x = (effector_profile[:,0].max()+effector_profile[:,0].min()) * 0.5
# mid_y = (effector_profile[:,1].max()+effector_profile[:,1].min()) * 0.5
# mid_z = (effector_profile[:,2].max()+effector_profile[:,2].min()) * 0.5

# ax.set_xlim(mid_x - max_range, mid_x + max_range)
# ax.set_ylim(mid_y - max_range, mid_y + max_range)
# ax.set_zlim(mid_z - max_range, mid_z + max_range)

# plt.legend()


# plt.show()


