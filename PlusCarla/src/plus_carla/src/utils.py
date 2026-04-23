import sys
sys.path.append('/opt/plusai/lib/python/plusmap')
from pyproj import Proj
import pyproj_eqdc

import google.protobuf.text_format as protobuf_text_format
import logging
import math
import numpy as np
import os
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import yaml
from collections import deque

EQDC_CONVERTER = pyproj_eqdc.EqdcConverter()
EQDC_CONVERTER.configure('NA', 'WGS84')
GEO_REF = '+proj=tmerc +lat_0=29.81145 +lon_0=-98.0087 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs' # default proj_str 

CAR = deque(['dodge.charger_2020', 'audi.tt', 'dodge.charger_police_2020', 'ford.crown', 'citroen.c3', 'ford.mustang', 
             'jeep.wrangler_rubicon', 'lincoln.mkz_2020', 'tesla.cybertruck', 'mercedes.coupe', 'chevrolet.impala', 
             'mercedes.coupe_2020', 'mini.cooper_s_2021', 'nissan.patrol', 'nissan.patrol_2021', 'tesla.model3', 'toyota.prius'])

VAN = deque(['ford.ambulance', 'volkswagen.t2', 'mercedes.sprinter', 'volkswagen.t2_2021'])

BUS = deque(['mitsubishi.fusorosa'])

TRUCK = deque(['carlamotors.carlacola', 'carlamotors.european_hgv', 'carlamotors.firetruck'])

MOTO = deque(['harley-davidson.low_rider', 'kawasaki.ninja', 'vespa.zx125', 'yamaha.yzf'])

BICYCLE = deque(['bh.crossbike', 'diamondback.century', 'gazelle.omafiets'])

bp_map = {'CAR': CAR, 'VAN': VAN, 'BUS': BUS, 'TRUCK': TRUCK, 'MOTO': MOTO, 'BICYCLE': BICYCLE}

def set_geo_ref(geo_ref):
    global GEO_REF
    if geo_ref:
        GEO_REF = geo_ref

def convert_to_carla_coord(lat, lon, x_offset=0, y_offset=0):
    if x_offset != 0 or y_offset != 0:
        eqdc_x, eqdc_y = EQDC_CONVERTER.from_latlon(lat, lon)
        x = eqdc_x - x_offset
        y = eqdc_y - y_offset
    else:
        proj = Proj(GEO_REF)
        x, y = proj(lon, lat)
    return x,-y

def plus_xy_to_carla_xy(x, y, x_offset=0, y_offset=0):
    if x_offset != 0 or y_offset != 0:
        carla_x = x - x_offset
        carla_y = -(y - y_offset)
    else:
        lat, lon = EQDC_CONVERTER.to_latlon(x, y)
        carla_x, carla_y = convert_to_carla_coord(lat, lon)
    return carla_x, carla_y # centimeter to meter

def get_xodr_offset(world):
    import xml.etree.ElementTree as ET

    xodr = world.get_map().to_opendrive()
    root = ET.fromstring(xodr)

    header = root.find("header")
    user_data = header.find("userData")
    global_offset = user_data.find("globalOffset")

    x = float(global_offset.get("x"))
    y = float(global_offset.get("y"))
    z = float(global_offset.get("z"))

    print(x, y, z)

    return x, y

def sanitize_text(text, encoding='utf-8', errors='replace'):
    if isinstance(text, bytes):
        return text.decode(encoding, errors=errors)
    else:
        return str(text)
    
def spherical_to_cartesian(azimuth_deg, elevation_deg, range):
    azimuth = np.deg2rad(azimuth_deg)
    elevation = np.deg2rad(90 + elevation_deg)
    x = range * np.sin(elevation) * np.cos(azimuth)
    y = range * np.sin(elevation) * np.sin(azimuth)
    z = range * np.cos(elevation)

    return [x, y, -z]

def transform_between_frames(points, T_from, T_to, offset_y=0, offset_z=0):
    # Separate XYZ and intensity
    xyz = points[:, :3]
    intensity = points[:, 3:]

    # Convert to homogeneous (N,4)
    ones = np.ones((xyz.shape[0], 1))
    xyz_one = np.hstack([xyz, ones])
    # To world
    xyz_world = (T_from @ xyz_one.T).T
    xyz_world[:, 1] += offset_y
    xyz_world[:, 2] += offset_z

    # To main LiDAR frame
    xyz_in_target = (np.linalg.inv(T_to) @ xyz_world.T).T[:, :3]
    pt_in_target = np.hstack([xyz_in_target, intensity])

    return pt_in_target  # shape (N, 4)

def get_camera_calibration(sensor, image_w, image_h):
    """Extracts camera calibration parameters from CARLA."""
    # Intrinsic matrix (M)
    fov = float(sensor.attributes['fov'])
    f = image_w / (2.0 * np.tan(np.radians(fov) / 2.0))
    M = np.array([[f, 0, image_w / 2.0],
                  [0, f, image_h / 2.0],
                  [0, 0, 1]])
    
    # No distortion (D)
    D = np.zeros(5)

    # Rectification matrix (R)
    R = np.eye(3)

    # Projection matrix (P)
    P = np.zeros((3, 4))
    P[:3, :3] = M  # Use intrinsic matrix

    print("Intrinsic Matrix (M):\n", M)
    print("Distortion Coefficients (D):", D)
    print("Rectification Matrix (R):\n", R)
    print("Projection Matrix (P):\n", P)

    return R, P, M, D

def calculate_radar_speed(obs_pos, obs_v, radar_pos, radar_v):
    direction = obs_pos - radar_pos
    direction /= np.linalg.norm(direction)
    relative_velocity = obs_v - radar_v
    radar_speed = np.dot(relative_velocity, direction)
    return radar_speed

def form_lidar_msg(points, model):
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('velocity', 12, PointField.FLOAT32, 1),
    ]
    if model == "Lidar":
        fields.append(PointField('intensity', 16, PointField.FLOAT32, 1))

    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = '{}_frame'.format(model)

    return pc2.create_cloud(header, fields, points)

def generate_video(srcdir, ext='.png', prefix='frame'):
    from pluspy import ffmpeg_utils
    # Generate video...
    PICT_EXT = ext
    FRAME_RATE = 10
    VID_FILE = srcdir+'vid.mp4'
    vid_output = os.path.join(srcdir, VID_FILE)
    short_pict = PICT_EXT.strip(".")
    print("Building video file %s from %s files in %s" % (vid_output, PICT_EXT, srcdir))
    am_files, _ = ffmpeg_utils.generate_video_from_files(
        ffmpeg_utils.file_lister(srcdir, prefix, PICT_EXT), FRAME_RATE, vid_output,
        threads=1, gpu_accel=True)
    if am_files == 0:
        raise RuntimeError("Couldn't find any %s files to generate video." % short_pict)
    
def get_transformation_matrix_from_tf(tf):
    x = tf.location.x
    y = tf.location.y
    z = tf.location.z
    roll = -tf.rotation.roll
    pitch = tf.rotation.pitch
    yaw = tf.rotation.yaw

    return get_transformation_matrix(x, y, z, roll, pitch, yaw)

def get_transformation_matrix(x, y, z, roll, pitch, yaw):
    # Compute the 4x4 homogeneous transformation matrix given
    # translation (x, y, z) and rotation (roll, pitch, yaw) in degrees.

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

class ScenarioProcessor:
    def __init__(self, scenario_file, x_offset, y_offset):
        from simulator import simulation_data_pb2
        self.scenario = simulation_data_pb2.SimulationConfig()
        with open(scenario_file, 'r') as s_file:
            content = sanitize_text(s_file.read())
            protobuf_text_format.Merge(content, self.scenario)
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.obs_map = {}
        self.get_obs()

    def get_obs(self):
        from perception import obstacle_detection_pb2
        for v in self.scenario.scenario_data.vehicles:
            if v.is_ego:
                continue
            obs_x, obs_y = plus_xy_to_carla_xy(v.state.x, v.state.y, self.x_offset, self.y_offset)
            obs_type = obstacle_detection_pb2.PerceptionObstacle.Type.Name(v.spec.type)
            self.obs_map[v.id] = [obs_x, obs_y, -math.degrees(v.state.yaw), v.state.v, obs_type]
        logging.error("%s", self.obs_map)

    def get_ego(self):
        for v in self.scenario.scenario_data.vehicles:
            if v.is_ego:
                ego_x, ego_y = plus_xy_to_carla_xy(v.state.x, v.state.y, self.x_offset, self.y_offset)
                print(ego_x, ego_y)
                return ego_x, ego_y

class SensorConfig:
    def __init__(self, file_path):
        try:
            with open(file_path, 'r') as f:
                self.sensor_config = yaml.safe_load(f)
        except (ValueError, RuntimeError) as e:
            raise ValueError("Failed to load file {}: {}".format(file_path, e))

        self.imu_pos = self.sensor_config['imu']

        for sensor in self.sensor_config['sensors']:
            new_conf = {}
            for key in sensor['config']:
                new_conf[key] = str(sensor['config'][key])
            sensor['config'] = new_conf

def get_blueprint_from_category(type_name):
    item = bp_map[type_name].popleft()
    bp_map[type_name].append(item)
    return 'vehicle.' + item
