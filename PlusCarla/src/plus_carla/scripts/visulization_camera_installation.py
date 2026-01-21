import argparse
import numpy as np
import math
import cv2

import tf.transformations

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


args_parser = argparse.ArgumentParser()
args_parser.add_argument("files", nargs="+", help="input calibration file")

args = args_parser.parse_args()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
axis_color = ["r", "g", "b"]


def get_right_camera2imu(left_camera2imu, r_left2right, t_left2right):
    r_left2right_inv = np.linalg.inv(r_left2right)
    t_right2left = -np.matmul(r_left2right, t_left2right)

    T_right2left = np.identity(4)
    T_right2left[:3,:3] = r_left2right_inv
    T_right2left[:3, 3] = t_right2left[:3, 0]

    T_right2imu = np.matmul(left_camera2imu, T_right2left)

    return T_right2imu


def plot_axis(tr_sensor_to_imu, sensor_name, axis_length = 0.3):
    for idx in range(0, 3):
        scale = axis_length / np.linalg.norm(tr_sensor_to_imu[:3, idx])
        xs = [tr_sensor_to_imu[0][3], tr_sensor_to_imu[0][3] + tr_sensor_to_imu[0][idx] * scale]
        ys = [tr_sensor_to_imu[1][3], tr_sensor_to_imu[1][3] + tr_sensor_to_imu[1][idx] * scale]
        zs = [tr_sensor_to_imu[2][3], tr_sensor_to_imu[2][3] + tr_sensor_to_imu[2][idx] * scale]
        ax.plot(xs, ys, zs, axis_color[idx])

        ax.text(tr_sensor_to_imu[0][3],
                tr_sensor_to_imu[1][3],
                tr_sensor_to_imu[2][3],
                sensor_name)


def extract_sensor_name(sensor_name):
    sensor_names = []
    if "_halfres" in sensor_name:
        sensor_name = sensor_name.replace("_halfres", "")

    if "left_right" in sensor_name:
        left_sensor_name = sensor_name.replace("_right", "")
        right_sensor_name = sensor_name.replace("left_", "")
        sensor_names.append(left_sensor_name)
        sensor_names.append(right_sensor_name)
    else:
        sensor_names.append(sensor_name)

    return sensor_names


for file in args.files:
    file_storage = cv2.FileStorage()
    file_storage.open(file, cv2.FileStorage_READ)
    matrix = file_storage.getNode("Tr_cam_to_imu").mat()
    sensor_name = file_storage.getNode("sensor_name").string()

    sensor_names = extract_sensor_name(sensor_name)

    np_matrix = np.reshape(matrix, (4, 4))
    r, p, y = tf.transformations.euler_from_matrix(np_matrix[:3][:3])

    print(f"Sensor name: {sensor_names[0]}, roll: {r * 180 / math.pi}, pitch: {p * 180 / math.pi}, yaw: {y * 180 / math.pi}")

    plot_axis(np_matrix, sensor_names[0])

    
    if file_storage.getNode("type").string() == "stereo_camera":
        R_left2right = np.reshape(file_storage.getNode("R").mat(), (3, 3))
        t_left2right = np.reshape(file_storage.getNode("T").mat(), (3, 1))
        np_matrix = get_right_camera2imu(np_matrix, R_left2right, t_left2right)
        r, p, y = tf.transformations.euler_from_matrix(np_matrix[:3][:3])
        print(f"Sensor name: {sensor_names[1]}, roll: {r * 180 / math.pi}, pitch: {p * 180 / math.pi}, yaw: {y * 180 / math.pi}")

        plot_axis(np_matrix, sensor_names[1])

np_matrix = np.identity(4)
plot_axis(np_matrix, "Calibration_IMU")


ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_ylim(-2, 2)

ax.view_init(elev=20., azim=180)

ax.text2D(0.15, 0.98, 'x axis', transform=ax.transAxes, color='red', fontsize=12)
ax.text2D(0.15, 0.96, 'y axis', transform=ax.transAxes, color='green', fontsize=12)
ax.text2D(0.15, 0.94, 'z axis', transform=ax.transAxes, color='blue', fontsize=12)

plt.show()
