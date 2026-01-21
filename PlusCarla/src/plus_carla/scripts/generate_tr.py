import numpy as np

def get_transformation_matrix(x, y, z, roll, pitch, yaw):
    """
    Compute the 4x4 homogeneous transformation matrix given
    translation (x, y, z) and rotation (roll, pitch, yaw) in degrees.
    
    Args:
        x (float): Translation along X-axis.
        y (float): Translation along Y-axis.
        z (float): Translation along Z-axis.
        roll (float): Rotation around X-axis (in degrees).
        pitch (float): Rotation around Y-axis (in degrees).
        yaw (float): Rotation around Z-axis (in degrees).
    
    Returns:
        np.ndarray: 4x4 transformation matrix.
    """

    # Convert degrees to radians
    roll, pitch, yaw = np.radians([roll, pitch, yaw])

    # Compute rotation matrices
    c_x, s_x = np.cos(roll), np.sin(roll)  # Roll
    c_y, s_y = np.cos(pitch), np.sin(pitch)  # Pitch
    c_z, s_z = np.cos(yaw), np.sin(yaw)  # Yaw

    # Rotation matrix (ZYX order: first yaw, then pitch, then roll)
    R = np.array([
        [c_y * c_z,  c_y * s_z,  -s_y],
        [s_x * s_y * c_z - c_x * s_z,  s_x * s_y * s_z + c_x * c_z,  s_x * c_y],
        [c_x * s_y * c_z + s_x * s_z,  c_x * s_y * s_z - s_x * c_z,  c_x * c_y]
    ])

    # Translation vector
    t = np.array([x, y, z])

    # Construct 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R  # Insert rotation matrix
    T[:3, 3] = t   # Insert translation vector

    return T

# Example Usage
x, y, z = 2.48, -0.097, 1.557  # Translation
roll, pitch, yaw = 0, -3.2, -10  # Rotation in degrees

T = get_transformation_matrix(x, y, z, roll, pitch, yaw)
T1 = get_transformation_matrix(1.146, 1.296, -0.758, 0, 0, 55)
p = np.array([[0, 0, 0, 1]])
p_w = (T1 @ p.T).T
p_a = (np.linalg.inv(T) @ p_w.T).T

xyz = p[:, :3]
intensity = p[:, 3:]

# Convert to homogeneous (N,4)
ones = np.ones((xyz.shape[0], 1))
xyz_one = np.hstack([xyz, ones])
# To world
xyz_world = (T1 @ xyz_one.T).T

# To LiDAR 1 frame
xyz_in_target = (np.linalg.inv(T) @ xyz_world.T).T[:, :3]
pt_in_target = np.hstack([xyz_in_target, intensity])

print(pt_in_target)

print("Transformation Matrix (T):\n", T)

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Example transformation matrix (right-handed)

def plot_transform(T, ax=None, label='Frame', length=0.5):
    """
    Plot a 3D transform with right-handed axes: X-forward, Y-right, Z-down.
    T is a 4x4 transformation matrix.
    """
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

    origin = T[:3, 3]
    R = T[:3, :3]

    x_axis = origin + R[:, 0] * length  # X (forward)
    y_axis = origin + R[:, 1] * length  # Y (right)
    z_axis = origin + R[:, 2] * length  # Z (down)

    ax.quiver(*origin, *(x_axis - origin), color='r', label='X (forward)')
    ax.quiver(*origin, *(y_axis - origin), color='g', label='Y (right)')
    ax.quiver(*origin, *(z_axis - origin), color='b', label='Z (down)')

    ax.text(*origin, label, fontsize=10)

    return ax

def t_up(x, y, z, roll, pitch, yaw):
    """
    Create a 4x4 transformation matrix from position and roll-pitch-yaw (in radians).

    Args:
        x, y, z: position
        roll, pitch, yaw: orientation in radians (X-forward, Y-right, Z-up)

    Returns:
        4x4 numpy array representing the transformation matrix.
    """

    # Compute rotation matrices
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Combined rotation: R = Rz * Ry * Rx
    R = Rz @ Ry @ Rx

    # Build 4x4 matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]

    return T

def plot_transform_up(T, axis_length=1.0):
    """
    Draw a transformed coordinate frame based on a 4x4 transformation matrix.

    Args:
        T (numpy.ndarray): 4x4 transformation matrix.
        axis_length (float): Length of each axis to draw.
    """
    assert T.shape == (4, 4), "Input must be a 4x4 transformation matrix."

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Origin
    origin = T[:3, 3]

    # Axes directions
    x_dir = T[:3, 0] * axis_length
    y_dir = T[:3, 1] * axis_length
    z_dir = T[:3, 2] * axis_length

    # Draw X (forward)
    ax.quiver(*origin, *x_dir, color='r', linewidth=2)
    ax.text(*(origin + x_dir), 'X (forward)', color='r')

    # Draw Y (right)
    ax.quiver(*origin, *y_dir, color='g', linewidth=2)
    ax.text(*(origin + y_dir), 'Y (right)', color='g')

    # Draw Z (up)
    ax.quiver(*origin, *z_dir, color='b', linewidth=2)
    ax.text(*(origin + z_dir), 'Z (up)', color='b')

    # Set plot limits centered around the origin
    max_range = axis_length * 1.5
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_zlim(-5, 5)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()

T = np.array([[ 9.9997795401046130e-01, 6.5582123555759462e-03,
       1.1702546185109902e-03, 4.5776690999999996e+00],
       [6.5633023414739837e-03, -9.9996887846787041e-01,
       -4.4024004840397049e-03, 2.1626802000000001e-01],
       [1.1413462000000000e-03, 4.4099828000000001e-03,
       -9.9998962999999996e-01, -7.0499997999999997e-01], [0., 0., 0., 1. ]])
T_rear_l_radar = np.array([ [-9.9979689854589615e-01, 1.9807854769441722e-02,
       -3.7162545526003461e-03, 2.4940000000000002e+00],
       [-1.9699623753652310e-02, -9.9943603695423144e-01,
       -2.7194353472386430e-02, 1.4299999999999999e+00],
       [-4.2528205269394886e-03, -2.7115621443277740e-02,
       9.9962325732833524e-01, -5.1410140000000004e-01], [0., 0., 0., 1. ]])
T_front_center_Radar = np.array([[0.90630779, 0.42261826, 0., 4.407999999999999e+00],
                                 [-0.42261826, 9.06e-01, 0., 0.245],
                                 [0., 0., 1., 7.149999e-01],
                                 [0., 0., 0., 1.]])
T_front_radar_sr = np.array([[ 9.06e-01, 4.23e-01, 0., 4.437999999999999e+00],
                             [-4.23e-01, 9.06e-01, 0., 0.005e+00],
                             [0., 0., 1., 7.329999e-01],
                             [0., 0., 0., 1. ]])

T = np.array([[ 0.985453, -0.164315, 0.043397, 2.480986],
       [0.163459, 0.986291, 0.022597, 0.099346],
       [-0.046516, -0.015174, 0.998803, 1.598060], [0., 0., 0., 1. ]])

# Plot
# ax = plot_transform(T, label='Base Frame')
# # ax = plot_transform(T1, label='Base Frame')
# ax.set_xlim([-3, 3])
# ax.set_ylim([-3, 3])
# ax.set_zlim([-3, 3])
# ax.invert_yaxis()
# # ax.invert_zaxis()  # Make Z point downward
# ax.set_xlabel('X (forward)')
# ax.set_ylabel('Y (right)')
# ax.set_zlabel('Z (down)')
# ax.set_title('Right-Handed Frame')

# ax.text2D(0.15, 0.98, 'x axis', transform=ax.transAxes, color='red', fontsize=12)
# ax.text2D(0.15, 0.96, 'y axis', transform=ax.transAxes, color='green', fontsize=12)
# ax.text2D(0.15, 0.94, 'z axis', transform=ax.transAxes, color='blue', fontsize=12)

# plt.show()
T2 = t_up(1.146, 1.296, -0.758, 0, 0, -125)
print(T2)
plot_transform_up(T)