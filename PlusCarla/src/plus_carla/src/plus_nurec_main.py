from email.header import Header
import glob
import os
import sys

# Add ROS and PlusAI Python paths early to ensure packages are importable
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
sys.path.append('/opt/plusai/lib/python')

import argparse
import carla
import cv2
import grpc
import json
import logging
import math
import numpy as np
import os
import queue
import rospy
import time
from concurrent import futures
from collections import defaultdict
from geometry_msgs.msg import Point32
from perception import obstacle_detection_pb2
from plus_carla.srv import SimConnect, SimConnectResponse
from pyproj import Proj
from radar_msgs.msg import RadarTrackArray, RadarTrack
from sensor_msgs.msg import CompressedImage as Image
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation as R
from typing import Dict, List, Any, Optional, Set, Callable, Union, Tuple
import xml.etree.ElementTree as ET
import zipfile

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from proto import image_stream_pb2
from proto import image_stream_pb2_grpc

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../../PythonAPI/examples/plus/plussim"))
from projection_functions import get_t_rig_enu_from_ecef, get_latlon_from_t_ecef, yaw_from_t_ecef
from plus_nurec_client import PlusNurecClient, load_world_from_xodr, get_ego_trajectory

import utils
import traceback
import threading

vehicle = None
obs_vehicle_list = {}
yaw_offset = 0

SPAWN_OBS = False
SIM_STARTED = False

request_event = threading.Event()
latest_response = None
last_processed_frame = -1
current_frame = None
ego_traj = []
traj_index = 0
plus_nurec_client = None

class SensorManager:
    def __init__(self, world, sensor_type, transform, attached, sensor_options):
        self.surface = None
        self.world = world
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
        self.sensor_options = sensor_options
        self.ros_image = None
        self.pc = None
        self.radar_tracks = defaultdict(list)
        self.radar = None

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == 'LiDAR':
            self.lidar_queue = queue.Queue()
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
            self.M = utils.get_transformation_matrix_from_tf(transform)

            lidar_bp.set_attribute('pattern_file', 'default')
            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])
            self.sensor_name = lidar_bp.get_attribute('role_name').as_str()

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(lambda data: self.lidar_callback(data, self.lidar_queue))

            return lidar
        
        elif sensor_type == 'FMCW':
            self.lidar_queue = queue.Queue()
            self.latest_lidar = None
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.fmcw')
            self.sensor_name = lidar_bp.get_attribute('role_name').as_str()
            self.M = utils.get_transformation_matrix_from_tf(transform)

            lidar_bp.set_attribute('pattern_name', '64-19.2-Uniform')
            lidar_bp.set_attribute('motion_compensate', 'true')

            raycast_modes = {'frame': '0', 'line': '1', 'point': '2'}
            lidar_bp.set_attribute('raycast_mode', raycast_modes['frame'])

            lidar_bp.set_attribute('noise_stddev', '0.05')
            lidar_bp.set_attribute('dropoff_general_rate', '0.3')

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            try:
                lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)
                lidar.listen(lambda data: self.fmcw_callback(data, self.lidar_queue))

                return lidar
            except Exception as e:
                logging.error("Error spawing FMCW lidar: %s", e)

        elif sensor_type == "Radar":
            radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
            self.z_offset = transform.location.z
            self.ground_removal_threshold = 0.1 if self.z_offset < -0.5 else 0.2
            for key in sensor_options:
                radar_bp.set_attribute(key, sensor_options[key])
            radar_bp.set_attribute('radar_type', 'Altos')
            self.sensor_name = radar_bp.get_attribute('role_name').as_str()
            self.radar_type = radar_bp.get_attribute('radar_type').as_str()
            self.obstacles = {}
            self.v = [0., 0., 0.]

            radar = self.world.spawn_actor(radar_bp, transform, attach_to=attached)
            radar.listen(self.radar_callback)

            return radar

        else:
            return None

    def lidar_callback(self, data, lidar_queue):
        points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))

        self.pc = points.copy()
        self.pc[:,1] = -self.pc[:,1]
        # Add velocity
        velocity = np.zeros((self.pc.shape[0], 1), dtype=np.float32)
        # Concatenate -> shape becomes (N, 5)
        self.pc = np.concatenate([self.pc, velocity], axis=1)
        if SIM_STARTED:
            lidar_queue.put(self.pc)

    def fmcw_callback(self, data, lidar_queue):
        point_cloud = np.frombuffer(data.raw_data,
                                    dtype=np.dtype([('azimuth', np.float32),
                                                    ('elevation', np.float32),
                                                    ('range', np.float32),
                                                    ('intensity', np.float32),
                                                    ('velocity', np.float32),
                                                    ('cos_angle', np.float32),
                                                    ('instance', np.uint32),
                                                    ('class', np.uint32),
                                                    ('point_idx', np.uint32),
                                                    ('beam_idx', np.ubyte),
                                                    ('valid', bool),
                                                    ('dynamic', bool)])).copy()

        # Negate the azimuth to account for scanning direction
        point_cloud['azimuth'] = -point_cloud['azimuth']

        point_cloud = point_cloud[point_cloud['valid'] == 1]

        points = np.array(
            utils.spherical_to_cartesian(point_cloud['azimuth'], point_cloud['elevation'], 
                                   point_cloud['range'])).T
        intensity = point_cloud['intensity'].reshape(-1, 1)  # shape: (N, 1)
        velocity = point_cloud['velocity'].reshape(-1, 1)  # shape: (N, 1)

        points = np.hstack((points, velocity))  # shape: (N, 4)
        points = np.hstack((points, intensity))  # shape: (N, 5)

        if SIM_STARTED:
            lidar_queue.put(points)

    def radar_callback(self, data):
        self.radar_tracks.clear()
        self.radar = RadarTrackArray()
        self.radar.header = rospy.Header()
        self.radar.header.frame_id = "radar"
        self.cur_tf = self.sensor.get_transform()
        points = []

        for detection in data:
            id = detection.actor_id

            # Convert from spherical to cartesian
            depth = detection.depth
            azimuth = detection.azimuth
            altitude = detection.altitude
            velocity = detection.velocity

            x = depth * math.cos(azimuth) * math.cos(-altitude)
            y = depth * math.sin(-azimuth) * math.cos(altitude)
            z = depth * math.sin(altitude)

            if abs(1.288+self.z_offset+z) < self.ground_removal_threshold:
                continue
            if depth < 1:
                continue

            if id in self.obstacles.keys():
                obs = self.obstacles[id]
                obs_pos = np.array([obs[0], obs[1], 0])
                obs_v = np.array([obs[2], obs[3], 0])
                radar_pos = np.array([self.cur_tf.location.x, self.cur_tf.location.y, self.cur_tf.location.z])
                radar_v = np.array([self.v[0], self.v[1], self.v[2]])
                velocity = utils.calculate_radar_speed(obs_pos, obs_v, radar_pos, radar_v)

                vx = velocity * math.cos(azimuth) * math.cos(-altitude)
                vy = velocity * math.sin(-azimuth) * math.cos(altitude)

                self.radar_tracks[id].append({'x': x, 'y': y, 'z': z, 'vx': vx, 'vy': vy})

            if self.radar_type == 'Altos':
                points.append((x, y, z, velocity))

        if self.radar_type == "Altos":
            self.pc = utils.form_lidar_msg(points, "Radar")
            self.radar = RadarTrackArray()
            return

        for track_id, points in self.radar_tracks.items():
            radar_track = RadarTrack()
            sumx = sum(p['x'] for p in points)
            sumy = sum(p['y'] for p in points)
            sumz = sum(p['z'] for p in points)
            sumvx = sum(p['vx'] for p in points)
            sumvy = sum(p['vy'] for p in points)

            count = len(points)
            point = Point32()
            point.x = sumx / count
            point.y = sumy / count
            point.z = sumz / count

            radar_track.track_shape.points.append(point)
            radar_track.linear_velocity.x = sumvx / count
            radar_track.linear_velocity.y = sumvy / count
            radar_track.linear_acceleration.x = 0.0
            radar_track.track_id = track_id

            self.radar.tracks.append(radar_track)

        # If use default radar model pointcloud would be disabled
        self.pc = PointCloud2()

    def destroy(self):
        self.sensor.destroy()

def capture_image(img_bytes):
    """Captures an image from a NUREC client response and converts it to ROS CompressedImage format."""
    ros_image = Image()
    if img_bytes == b"":
        return ros_image
    
    ros_image.header.stamp = rospy.Time.now()
    ros_image.format = "jpeg"
    resized_image = resize_image(img_bytes)
    ros_image.data = resized_image

    return ros_image

def resize_image(img_bytes):
    np_arr = np.frombuffer(img_bytes, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    # Resize the image to 1920x1080
    resized_image = cv2.resize(img, (1920, 1080), interpolation=cv2.INTER_AREA)
    _, jpeg = cv2.imencode('.jpg', resized_image)
    return jpeg.tobytes()

def handle_plus_vehicle_control(req):
    global vehicle, obs_vehicle_list, SPAWN_OBS, yaw_offset, SIM_STARTED, ego_traj
    global latest_response, last_processed_frame, current_frame, traj_index, plus_nurec_client

    if not vehicle:
        rospy.logerr("Vehicle not initialized!")
        return SimConnectResponse(False, Image(), Image(), Image(), Image(), Image(), Image(), Image(), Image(), Image(), Image(), None, None, None, None, None)
    if not SIM_STARTED:
        SIM_STARTED = True

    # if last_processed_frame == current_frame:
    #     return SimConnectResponse(False, Image(), Image(), Image(), Image(), Image(), Image(), Image(), Image(), Image(), Image(), None, None, None, None, None)

    # last_processed_frame = current_frame
    # Move the vehicle
    ego_lat = req.ego_lat
    ego_lon = req.ego_lon
    ego_x, ego_y = utils.convert_to_carla_coord(ego_lat, ego_lon)
    ego_vx = req.ego_vx
    ego_vy = req.ego_vy
    # ego_yaw = -math.degrees(req.ego_yaw)

    print("loop")
    ego_pose = np.array(ego_traj[traj_index])
    ego_tf = plus_nurec_client.get_projection_poses(ego_pose)
    vehicle.set_transform(ego_tf)
    latest_response = plus_nurec_client.request_image_at_pose(ego_tf)

    traj_index += 1

    ego_tf_ecef = plus_nurec_client.t_world_base @ ego_pose
    ego_latlonalt = get_latlon_from_t_ecef(ego_tf_ecef)
    ego_yaw = yaw_from_t_ecef(ego_tf_ecef, ego_latlonalt[0], ego_latlonalt[1], [1, 0, 0])

    ## Assign new obs pose
    try:
        obs_info = req.obstacle_info
        pb = obstacle_detection_pb2.ObstacleDetection()
        pb.ParseFromString(bytes(obs_info))
    except:
        traceback.print_exc()
    obs_for_radar = {}

    for radar in radar_list:
        radar.obstacles = obs_for_radar
        radar.v = [ego_vx, ego_vy, 0.]

    request_event.set()

    return SimConnectResponse(True,
                              capture_image(latest_response.front_left_image),
                              capture_image(latest_response.front_right_image),
                              Image(),
                              capture_image(latest_response.front_center_image),
                              capture_image(latest_response.left_front_image),
                              capture_image(latest_response.left_side_image),
                              capture_image(latest_response.left_rear_image),
                              capture_image(latest_response.right_front_image),
                              capture_image(latest_response.right_side_image),
                              capture_image(latest_response.right_rear_image),
                              left_aeva_lidar.latest_lidar,
                              front_center_radar.radar,
                              left_rear_radar_sr.radar,
                              right_rear_radar_sr.radar,
                              front_center_radar.pc,
                              ego_latlonalt[0],
                              ego_latlonalt[1],
                              ego_latlonalt[2],
                              ego_yaw,)

def run_simulation(args, client):
    """This function performed one test run using the args parameters
    and connecting to the carla client passed.
    """
    vehicle_list = []

    global vehicle, obs_vehicle_list
    global camera_list, lidar_list, radar_list
    global latest_response, current_frame, ego_traj, traj_index, plus_nurec_client
    camera_list = []
    lidar_list = []
    radar_list = []

    try:
        # Connect to simulator node
        rospy.init_node('plus_carla_server')
        s = rospy.Service('plus_carla', SimConnect, handle_plus_vehicle_control)

        # Write logging info into file
        os.makedirs(args.log_dir, exist_ok=True)
        path = os.path.join(args.log_dir, "carla_client.log")
        fh = logging.FileHandler(path)
        fh.setLevel(logging.INFO)
        fh.setFormatter(logging.Formatter(
            "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
        ))

        logging.getLogger().addHandler(fh)

        ## Call Nurec Client to load map and set up grpc connection
        plus_nurec_client = PlusNurecClient(args.usdz_file)
        data, world = load_world_from_xodr(plus_nurec_client.usdz_path, client)
        plus_nurec_client.set_up_world(world)
        plus_nurec_client.extract_trajectory(data)
        plus_nurec_client.set_up_grpc()
        ego_traj = plus_nurec_client.traj

        original_settings = world.get_settings()

        if args.sync:
            traffic_manager = client.get_trafficmanager(8000)
            settings = world.get_settings()
            traffic_manager.set_synchronous_mode(True)
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)


        # Instanciating the vehicle to which we attached the sensors
        bp = world.get_blueprint_library().filter('vehicle.mercedes.sprinter')[0]
        # Spawn ego vehicle at the first point of the trajectory
        ego_pose = np.array(ego_traj[0])
        spawn_loc = plus_nurec_client.get_projection_poses(ego_pose)
        spawn_loc.location.z += 0.5
        vehicle = world.spawn_actor(bp, spawn_loc)
        current_frame = world.tick()
        vehicle_list.append(vehicle)

        # IMU
        imu_bp = world.get_blueprint_library().find('sensor.other.imu')
        imu = world.spawn_actor(imu_bp, carla.Transform(carla.Location(x=0.497595, z=1.288), carla.Rotation(yaw=+00)), attach_to=vehicle)

        sc = utils.SensorConfig(args.sensor_config)
        for s in sc.sensor_config['sensors']:
            if s['model'] == 'FMCW' or (s['model'] == 'LiDAR' and 'pattern_name' in s['config'].keys()):
                s['config']['pattern_file'] = args.lidar_scan_pattern
            sensor = SensorManager(world, s['model'], carla.Transform(carla.Location(x=s['x'], y=s['y'], z=s['z']),
                                                                     carla.Rotation(roll=s['roll'], pitch=s['pitch'], yaw=s['yaw'])),
                                   imu, s['config'])
            if s['type'] == 'camera':
                continue
            elif s['type'] == 'radar':
                radar_list.append(sensor)
            else:
                lidar_list.append(sensor)

            globals()[s['name']] = sensor

        #Simulation loop

        while not rospy.is_shutdown():
            # Carla Tick
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()

            try:
                if SIM_STARTED:
                    left_latest = left_aeva_lidar.lidar_queue.get(True, 1.0)
                    right_latest_b = utils.transform_between_frames(right_aeva_lidar.lidar_queue.get(True, 1.0), right_aeva_lidar.M, left_aeva_lidar.M)
                    left_os0_b = utils.transform_between_frames(left_OS0_lidar.lidar_queue.get(True, 1.0), left_OS0_lidar.M, left_aeva_lidar.M, offset_y=2.462, offset_z=0.29)
                    right_os0_b = utils.transform_between_frames(right_OS0_lidar.lidar_queue.get(True, 1.0), right_OS0_lidar.M, left_aeva_lidar.M, offset_y=-2.462, offset_z=0.29)
                    combined_points = np.vstack((left_latest, right_latest_b, left_os0_b, right_os0_b))

                    left_aeva_lidar.latest_lidar = utils.form_lidar_msg(combined_points, "Lidar")
                    time.sleep(0.005)  # This can fix Open3D jittering issues.
            except queue.Empty:
                logging.error("Timed out waiting for LiDAR")

            request_event.clear()

    finally:
        [sensor.destroy() for sensor in camera_list + lidar_list + radar_list]
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])
        # client.apply_batch([carla.command.DestroyActor(x) for x in obs_vehicle_list.keys()])

        world.apply_settings(original_settings)

def main():
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)
        run_simulation(args, client)

        time.sleep(1)

    except KeyboardInterrupt:
        logging.info('\nCancelled by user. Bye!')


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(
        description='CARLA Simulator')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--async',
        dest='sync',
        action='store_false',
        help='Asynchronous mode execution')
    argparser.set_defaults(sync=True)
    argparser.add_argument(
        '--scenario-file',
        help='scenario file including ego and obstacle agents init poses',
        required=True)
    argparser.add_argument(
        '--sensor-config',
        help='config file(.yml) including position and rotation of all sensors',
        required=True)
    argparser.add_argument(
        '--init-yaw',
        type=float,
        required=True)
    argparser.add_argument(
        '--opposite-lane-obs-id',
        type=int,
        required=True,
        help='ID index threshold for obstacles on the opposite lane')
    argparser.add_argument(
        '--lidar-scan-pattern',
        type=str,
        default="/opt/carla/ScanPatterns.yaml",
        help='Aeva lidar scan pattern file path(should be able to be accessed by CARLA server)')
    argparser.add_argument(
        '--map',
        type=str,
        default="default",
        help='Specify which map to run the CARLA simulation on, it should be packed into CARLA server')
    argparser.add_argument(
        '--proj-str',
        type=str,
        help='XODR projection string, used for converting lat/lon to CARLA x/y')
    argparser.add_argument(
        '--log-dir',
        type=str,
        default='/opt/plusai/log',
        help='log file folder path')
    argparser.add_argument(
        '--usdz-file',
        help='USDZ file path for NUREC client, should be the same as the one used for nurec_server',
        required=True)

    args = argparser.parse_args()
    main()
