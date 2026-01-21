import argparse
import math
import numpy as np
import os
from collections import OrderedDict
import yaml

def rotate_x(psi):
    """return a matrix to rotate round x axis by psi radians"""
    return np.array(
        [
            [1, 0, 0],
            [0, math.cos(psi), -math.sin(psi)],
            [0, math.sin(psi), math.cos(psi)],
        ]
    )


def rotate_y(theta):
    """return a matrix to rotate round y axis by theta radians, aka"""
    return np.array(
        [
            [math.cos(theta), 0, math.sin(theta)],
            [0, 1, 0],
            [-math.sin(theta), 0, math.cos(theta)],
        ]
    )


def rotate_z(phi):
    """return a matrix to rotate round z axis by phi radians"""
    return np.array(
        [
            [math.cos(phi), -math.sin(phi), 0],
            [math.sin(phi), math.cos(phi), 0],
            [0, 0, 1],
        ]
    )

def r_matrix(psi, theta, phi, convert_z_to_x=False):
    """utility to combine euler angles into one rotation matrix"""
    # return rotate_z(phi).dot(rotate_y(theta).dot(rotate_x(psi)))
    extra_rotate = rotate_x(math.radians(-90)).dot(rotate_y(math.radians(90))) if convert_z_to_x else np.eye(3) 
    return rotate_z(phi).dot(rotate_y(theta).dot(rotate_x(psi).dot(extra_rotate)))

def tr_matrix(angles_and_offsets, convert_z_to_x=False):
    """utility to glom all the rotation and translation parts into one matrix"""
    (psi, theta, phi, x, y, z) = angles_and_offsets
    tr = np.eye(4, 4)
    tr[:3, :3] = r_matrix(psi, theta, phi, convert_z_to_x)
    tr[:3, 3] = [x, y, z]
    return tr

def generate_tf(x, y, z, roll, pitch, yaw, convert_z_to_x=False):
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    return tr_matrix((roll, pitch, yaw, x, y, z), convert_z_to_x)

class OpenCVMatrix(dict):
    """Wraps only the matrix dict for YAML representation."""
    pass

class CalibGenerator:
    def __init__(self, args):
        self.args = args
        self.date = args.date
        self.output_dir = args.output_dir
        self.load_config_file()

    def load_config_file(self):
        file_path = self.args.sensor_config
        self.car = os.path.splitext(os.path.basename(file_path))[0]
        with open(file_path, 'r') as f:
            self.sensor_config = yaml.safe_load(f)

    def calculate_tr_cam(self, T):
        # T = generate_tf(2.48, 0.097, 1.557, 0, 3.2, 10)
        data = T.flatten().tolist()
        self.Tr_cam_to_imu = OpenCVMatrix({   
                "rows": T.shape[0],
                "cols": T.shape[1],
                "dt": "d",
                "data": T.reshape(-1).tolist(),
            })

    def opencv_matrix_representer(self, dumper, data):
        return dumper.represent_mapping("tag:yaml.org,2002:opencv-matrix", data)

    def write_yaml(self):
        calib = {
                "type": self.type,
                "car": self.car,
                "date": self.date,
                "sensor_name": self.sensor_name,
                "Tr_cam_to_imu": self.Tr_cam_to_imu,
            }
        output = os.path.join(self.output_dir, self.car + '_' + self.date + '_' + self.sensor_name + '.yml')
        print(self.car + '_' + self.date + '_' + self.sensor_name + '.yml')
        yaml.add_representer(OpenCVMatrix, self.opencv_matrix_representer)
        

        with open(output, "w") as f:
            f.write("%YAML:1.0\n---\n")
            yaml.dump(calib, f, default_flow_style=None)

    def generate_calib(self):
        for sensor in self.sensor_config['sensors']:
            convert_z_to_x = False
            x = float(sensor['x'])
            y = -float(sensor['y'])
            z = float(sensor['z'])
            roll = float(sensor['roll'])
            pitch = -float(sensor['pitch'])
            yaw = -float(sensor['yaw'])
            if sensor['type'] == 'camera':
                if sensor['name'].startswith('left_') or sensor['name'].startswith('right_'):
                    convert_z_to_x = True
                else:
                    continue
            if sensor['type'] == 'lidar' and sensor.get('is_main_lidar') != True:
                continue
            T = generate_tf(x, y, z, roll, pitch, yaw, convert_z_to_x)
            self.type = sensor['type']
            self.sensor_name = sensor['name']
            self.calculate_tr_cam(T)
            self.write_yaml()


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(
        description='Calibration File Generator')
    argparser.add_argument(
        '--sensor-config',
        help='confil file(.yml) including position and rotation of all sensors',
        required=True)
    argparser.add_argument(
        '--date',
        type=str,
        help='date postfix',
        required=True)
    argparser.add_argument(
        '--output-dir',
        help='output path dir to save all calibration files generated',
        required=True)
    args = argparser.parse_args()

    cb = CalibGenerator(args)
    cb.generate_calib()
    # print(generate_tf(-0.077, 1.265, 1.487, -90, 30, 153, True).tolist())
    # import utils
    # print(utils.plus_xy_to_carla_xy(153863.9191307544, -39597.097718444646))
