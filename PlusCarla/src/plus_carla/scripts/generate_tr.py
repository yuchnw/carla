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
        [c_y * c_z,  -c_y * s_z,  s_y],
        [c_x * s_z + s_x * s_y * c_z,  c_x * c_z - s_x * s_y * s_z,  -s_x * c_y],
        [s_x * s_z - c_x * s_y * c_z,  s_x * c_z + c_x * s_y * s_z,   c_x * c_y]
    ])

    # Translation vector
    t = np.array([x, y, z])

    # Construct 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R  # Insert rotation matrix
    T[:3, 3] = t   # Insert translation vector

    return T

# Example Usage
x, y, z = -0.077, -1.265, 1.487  # Translation
roll, pitch, yaw = -90, -10, -33  # Rotation in degrees

T = get_transformation_matrix(x, y, z, roll, pitch, yaw)

print("Transformation Matrix (T):\n", T)