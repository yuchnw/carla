import argparse
from xmlrpc import client
import cv2
import grpc
import json
import math
import numpy as np
import os
from pyproj import Transformer
import pyproj_eqdc
import queue
from scipy.spatial.transform import Rotation as R
import sys
import time
from typing import Dict, List, Any, Optional, Set, Callable, Union, Tuple
import xml.etree.ElementTree as ET
import zipfile

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from proto import image_stream_pb2
from proto import image_stream_pb2_grpc

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../../PythonAPI/examples/plus/plussim"))
from projection_functions import get_t_rig_enu_from_ecef, get_latlon_from_t_ecef, yaw_from_t_ecef

import carla
import utils

def extract_json_from_usdz(usdz_file: str, json_file: str):
    result = []

    with zipfile.ZipFile(usdz_file, "r") as zip_ref:
        for file_info in zip_ref.infolist():
            if file_info.filename == json_file:
                with zip_ref.open(file_info.filename) as source_file:
                    json_data = json.load(source_file)
                    result = json_data

    return result

def get_ego_trajectory(usdz_file: str):
    json_array = extract_json_from_usdz(
        usdz_file, "rig_trajectories.json"
    )
    init_timestamp = json_array["rig_trajectories"][0]["T_rig_world_timestamps_us"][0]
    t_world_base = json_array["T_world_base"]
    return init_timestamp, json_array["rig_trajectories"][0]["T_rig_worlds"], t_world_base

def mat_to_carla_transform(mat: np.ndarray) -> carla.Transform:
    euler_angles = R.from_matrix(mat[:3, :3]).as_euler("zyx", degrees=False)
    euler_angles[1] = -euler_angles[1]
    euler_angles[2] = -euler_angles[2]
    return carla.Transform(
        carla.Location(x=mat[0, 3], y=-mat[1, 3], z=mat[2, 3]),
        carla.Rotation(
            pitch=euler_angles[2] * 180 / np.pi,
            yaw=-euler_angles[0] * 180 / np.pi,
            roll=euler_angles[1] * 180 / np.pi,
        ),
    )

def mat_to_carla_transform_3dgut(mat: np.ndarray) -> carla.Transform:
    euler_angles = R.from_matrix(mat[:3, :3]).as_euler("zyx", degrees=False)
    return carla.Transform(
        carla.Location(x=mat[0, 3], y=mat[1, 3], z=mat[2, 3]),
        carla.Rotation(
            pitch=euler_angles[1] * 180 / np.pi,
            yaw=euler_angles[0] * 180 / np.pi,
            roll=euler_angles[2] * 180 / np.pi,
        ),
    )

def load_world_from_xodr(usdz_path, client):
    try:
        with zipfile.ZipFile(usdz_path, "r") as zip_ref:
            # Check if map.xodr exists in the zip file
            if "map.xodr" not in zip_ref.namelist():
                available_files = zip_ref.namelist()
                raise KeyError(
                    f"map.xodr not found in {usdz_path}. Available files: {available_files}"
                )

            # Read the map.xodr file content
            with zip_ref.open("map.xodr") as xodr_file:
                data = xodr_file.read().decode("utf-8")
                filename = os.path.basename(usdz_path)
                print("Successfully loaded map.xodr from {}".format(filename))

            world = client.generate_opendrive_world(
                data,
                carla.OpendriveGenerationParameters(
                    vertex_distance=2.0,
                    max_road_length=500.0,
                    wall_height=0.0,
                    additional_width=7.6,
                    smooth_junctions=True,
                    enable_mesh_visibility=True,
                ),
            )
            # world = client.get_world()

            return data, world
    except Exception as e:
        print(f"Error reading XODR from NUREC file {usdz_path}: {e}")
        raise

class TrajectoryFollower:
    def __init__(self):
        pass

    def calculate_steering(self, target_transform: carla.Transform) -> tuple:
        """
        Calculate steering angle to reach target position using lookahead.
        
        Args:
            target_transform: Lookahead target transform to reach
            
        Returns:
            Tuple of (steering_value, steering_angle_degrees) 
        """
        # Get current vehicle transform
        vehicle_transform = self.vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        vehicle_rotation = vehicle_transform.rotation
        
        # Calculate vector from vehicle to lookahead target
        target_location = target_transform.location
        dx = target_location.x - vehicle_location.x
        dy = target_location.y - vehicle_location.y
        
        # Convert to vehicle coordinate system
        vehicle_yaw_rad = math.radians(vehicle_rotation.yaw)
        local_x = dx * math.cos(vehicle_yaw_rad) + dy * math.sin(vehicle_yaw_rad)
        local_y = -dx * math.sin(vehicle_yaw_rad) + dy * math.cos(vehicle_yaw_rad)
        
        # Calculate desired steering angle using lookahead point
        if abs(local_x) < 0.1:  # Avoid division by zero
            steering_angle = 0.0
        else:
            steering_angle = math.degrees(math.atan2(local_y, local_x))
        
        # Apply proportional control
        steering_angle *= self.steering_kp
        
        # Clamp to max steering angle
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
        
        # Convert to normalized steering (-1 to 1)
        normalized_steering = steering_angle / self.max_steering_angle
        
        return normalized_steering, steering_angle

    def calculate_throttle_brake(self, target_speed_ms: float, time_delta: float) -> tuple:
        """
        Calculate throttle and brake values based on target speed.
        
        Args:
            target_speed_ms: Target speed in m/s
            
        Returns:
            Tuple of (throttle, brake, p_term, i_term, d_term) values
        """
        # Get current vehicle speed
        velocity = self.vehicle.get_velocity()
        current_speed_ms = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        # Calculate speed error
        speed_error = target_speed_ms - current_speed_ms
        
        # PID control for speed
        self.speed_error_integral *= self.speed_error_decay
        self.speed_error_integral += speed_error * time_delta
        speed_error_derivative = (speed_error - self.last_speed_error) / time_delta
        
        # Calculate individual PID terms
        k_term = self.speed_kf * target_speed_ms
        p_term = self.speed_kp * speed_error
        i_term = self.speed_ki * self.speed_error_integral
        d_term = self.speed_kd * speed_error_derivative
        
        speed_control = k_term + p_term + i_term + d_term
        
        self.last_speed_error = speed_error
        
        # Convert to throttle/brake
        if speed_control > 0:
            # Need to accelerate
            throttle = min(speed_control, self.max_throttle)
            brake = 0.0
        else:
            # Need to brake
            throttle = 0.0
            brake = min(-speed_control, self.max_brake)
        
        return throttle, brake, k_term, p_term, i_term, d_term

    def update(self, current_world_time: float) -> carla.VehicleControl:
        """
        Update the trajectory follower and get vehicle control.
        
        Args:
            current_world_time: Current world time in seconds
            
        Returns:
            Vehicle control commands
        """
        if self.trajectory_complete:
            return carla.VehicleControl()
        
        time_delta = current_world_time - self.last_world_time
        if time_delta <= 0:
            time_delta = 0.01
        self.last_world_time = current_world_time
        
        # Get current target point, required speed, and lookahead point
        target_info = self.get_current_target_info(current_world_time)
        if target_info is None:
            return carla.VehicleControl()
        
        target_transform, target_speed_ms, lookahead_transform = target_info
        
        # Calculate control values using lookahead for steering
        steering, steering_angle = self.calculate_steering(lookahead_transform)
        throttle, brake, k_term, p_term, i_term, d_term = self.calculate_throttle_brake(target_speed_ms, time_delta)
        
        
        # Create control command
        control = carla.VehicleControl()
        control.throttle = throttle
        control.brake = brake
        control.steer = steering
        control.hand_brake = False
        control.reverse = False
        control.manual_gear_shift = False
        
        return control

def fmcw_callback(data, lidar_queue):
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

    lidar_queue.put(points)

proj_a = ("+proj=eqc +lat_ts=37.98634149 +lat_0=37.98634149 "
          "+lon_0=-121.99173067 +datum=WGS84 +units=m +no_defs")
a_to_wgs = Transformer.from_crs(proj_a, "EPSG:4326", always_xy=True)
EQDC_CONVERTER = pyproj_eqdc.EqdcConverter()
EQDC_CONVERTER.configure('NA', 'WGS84')
OFFSET_X = -7588916.727497556
OFFSET_Y =  10353498.911958147
def gs_pose_to_carla(t_3dgs):
    local_x = t_3dgs[0, 3]
    local_y = t_3dgs[1, 3]
    lon, lat = a_to_wgs.transform(local_x, local_y)
    eqdc_x, eqdc_y = EQDC_CONVERTER.from_latlon(lat, lon)
    x = eqdc_x - OFFSET_X
    y = eqdc_y - OFFSET_Y
    return x, -y

class PlusNurecClient:
    def __init__(self, usdz_path):
        self.usdz_path = usdz_path
        self.client = carla.Client("127.0.0.1", 2000)
        self.client.set_timeout(5.0)
        self.IMG_COUNTER = 0
        self.output_dir = "/home/yuchen.wang/workspace/carla/PythonAPI/examples/plus/plussim/data/"

    def set_up_grpc(self):
        self.channel = grpc.insecure_channel("localhost:50051")
        self.stub = image_stream_pb2_grpc.ImageServiceStub(self.channel)

    def set_up_world(self, world):
        self.world = world

    def get_projection_poses(self, traj_pose):
        tmp = self.t_scenario_carla @ traj_pose
        carla_pose = mat_to_carla_transform_3dgut(tmp)
        print(carla_pose)
        # Project the initial ego position to the nearest road to ensure it spawns on the road
        map = self.world.get_map()
        waypoint = map.get_waypoint(
            carla_pose.location,
            project_to_road=True,
        )
        # print(carla_pose.location.z, waypoint.transform.location.z)
        # return carla.Transform(waypoint.transform.location, carla_pose.rotation)
        return carla_pose
    
    def extract_trajectory(self, data):
        self.init_ts, self.traj, self.t_world_base = get_ego_trajectory(self.usdz_path)
        self.t_scenario_carla = get_t_rig_enu_from_ecef(self.t_world_base, data)

    def spawn_ego_vehicle(self):
        ego_pose = np.array(self.traj[0])
        spawn_loc = self.get_projection_poses(ego_pose)
        spawn_loc.location.z += 0.5
        print("Ego spawned at: {}".format(spawn_loc))
        ego_bp = self.world.get_blueprint_library().filter('vehicle.mercedes.coupe_2020')[0]
        self.ego = self.world.spawn_actor(ego_bp, spawn_loc)
        print(self.ego.bounding_box.extent.z)
        spawn_loc.location.z += 15
        spectator = self.world.get_spectator()
        spectator.set_transform(spawn_loc)

        self.world.tick()
        time.sleep(2)
        print(self.ego.get_transform())

    def spawn_lidar_sensor(self):
        lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.fmcw')
        lidar_bp.set_attribute('pattern_name', '64-19.2-Uniform')
        lidar_bp.set_attribute('pattern_file', '/home/yuchen.wang/workspace/carla/ScanPatterns.yaml')
        lidar_bp.set_attribute('motion_compensate', 'true')
        raycast_modes = {'frame': '0', 'line': '1', 'point': '2'}
        lidar_bp.set_attribute('raycast_mode', raycast_modes['frame'])
        lidar_bp.set_attribute('noise_stddev', '0.05')
        lidar_bp.set_attribute('dropoff_general_rate', '0.3')
        lidar_tf = carla.Transform(carla.Location(x=2.48,y=-0.097,z=1.557),carla.Rotation(pitch=-3.2,yaw=-10))
        lidar = self.world.spawn_actor(lidar_bp, lidar_tf, attach_to=self.ego)
        lidar_queue = queue.Queue()
        lidar.listen(lambda data: fmcw_callback(data, lidar_queue))

    def follow_trajectory(self):
        for pose in self.traj:
            ego_pose = np.array(pose)
            ego_tf = self.get_projection_poses(ego_pose)
            # print(self.t_world_base @ ego_pose)
            # print(np.linalg.inv(self.t_scenario_carla) @ np.array(ego_tf.get_matrix()).reshape(4, 4))
            self.ego.set_transform(ego_tf)
            # print(ego_tf)
            # time.sleep(0.5)
            self.world.tick()
            time.sleep(0.5)
            # print(self.ego.get_transform())
            print("===")

            self.request_image_at_pose(ego_tf)

    def request_image_at_pose(self, ego_tf: carla.Transform):
        req = image_stream_pb2.ImageRequest(    
            ego_x=ego_tf.location.x,
            ego_y=ego_tf.location.y,
            ego_z=ego_tf.location.z,
            ego_roll=ego_tf.rotation.roll,
            ego_pitch=ego_tf.rotation.pitch,
            ego_yaw=ego_tf.rotation.yaw,
            timestamp=self.init_ts+100000
        )
        # actor_transform = ego_tf.get_matrix()
        # print(np.array(actor_transform))  # 4x4 matrix

        resp = self.stub.GetLatestImage(req)

        self.save_image(resp.front_center_image, "front_center")
        # self.save_image(resp.front_left_image, "front_left")
        self.save_image(resp.left_front_image, "left_front")
        self.save_image(resp.right_side_image, "right_side")
        self.save_image(resp.left_rear_image, "left_rear")
        
        return resp
    

    def save_image(self, img_from_nurec, prefix):
        if len(img_from_nurec) == 0:
            return
        img = cv2.imdecode(
            np.frombuffer(img_from_nurec, np.uint8),
            cv2.IMREAD_COLOR)
        filename = f"{self.IMG_COUNTER:05d}"
        cv2.imwrite(f"{self.output_dir}/{prefix}_{filename}.jpg", img)
        self.IMG_COUNTER += 1


def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        "--usdz-path",
        metavar="U",
        required=True,
        help="Path to the USDZ file containing the NUREC scenario data",
    )
    args = argparser.parse_args()
    plus_nurec_client = PlusNurecClient(args.usdz_path)

    data, world = load_world_from_xodr(plus_nurec_client.usdz_path, plus_nurec_client.client)
    plus_nurec_client.set_up_world(world)

    plus_nurec_client.extract_trajectory(data)

    plus_nurec_client.set_up_grpc()

    plus_nurec_client.spawn_ego_vehicle()
    plus_nurec_client.spawn_lidar_sensor()

    plus_nurec_client.follow_trajectory()



if __name__ == "__main__":
    main()
    