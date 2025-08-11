#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Script that render multiple sensors in the same pygame window

By default, it renders four cameras, one LiDAR and one Semantic LiDAR.
It can easily be configure for any different number of sensors. 
To do that, check lines 290-308.
"""

import glob
import os
import sys

dist_path = os.path.abspath(os.path.join("../../../../", "PythonAPI/carla/dist/"))
try:
    sys.path.append(glob.glob('%s/carla-*%d.%d-%s.egg' % (
        dist_path,
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
sys.path.append('/opt/plusai/lib/python')
import argparse
import carla
import cv2
import rospy
import time
import numpy as np
import math
import queue
import random
from concurrent import futures
from pyproj import Proj
from plus_carla.srv import SimConnect, SimConnectResponse
from sensor_msgs.msg import CompressedImage as Image
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from collections import defaultdict
from geometry_msgs.msg import Point32

from radar_msgs.msg import RadarTrackArray, RadarTrack
from perception import obstacle_detection_pb2

# import google.protobuf.text_format as protobuf_text_format

import utils
from bounding_box import ClientSideBoundingBoxes


try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

display_manager = None
vehicle = None
s_fc = None
s_lf = None
s_ls = None
s_lr = None
s_rf = None
s_rs = None
s_rr = None
lidar = None
unify_lidar = {}
left_aeva = None
right_aeva = None
radar_front_lr = None
radar_l_rear_sr = None
radar_r_rear_sr = None
obs_vehicle_list = {}
yaw_offset = 0

_vehicle_moved = False
_sim_started = False

BB_COLOR = (248, 64, 24)
SPAWN_OBS = False
SIM_STARTED = False

class CustomTimer:
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time

    def time(self):
        return self.timer()

class DisplayManager:
    def __init__(self, grid_size, window_size):
        # pygame.init()
        # pygame.font.init()
        # self.display = pygame.display.set_mode(window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.display = None

        self.grid_size = grid_size
        self.window_size = window_size
        self.sensor_list = []
        self.frame_num = 0

    def get_display_size(self):
        return [int(self.window_size[0]/self.grid_size[1]), int(self.window_size[1]/self.grid_size[0])]

    def get_display_offset(self, gridPos):
        dis_size = self.get_display_size()
        return [int(gridPos[1] * dis_size[0]), int(gridPos[0] * dis_size[1])]

    def add_sensor(self, sensor):
        self.sensor_list.append(sensor)

    def render(self, bounding_boxes_map):
        if not self.render_enabled():
            return

        for s in self.sensor_list:
            s.render(bounding_boxes_map[s.sensor_name])

        self.frame_num += 1
        # pygame.image.save(self.display, '_out/frame{}.png'.format(self.frame_num))

        pygame.display.flip()

    def destroy(self):
        for s in self.sensor_list:
            s.destroy()

    def render_enabled(self):
        return self.display != None

# class SensorManager:
#     def __init__(self, world, display_man, sensor_type, transform, attached, sensor_options):
#         self.surface = None
#         self.world = world
#         self.display_man = display_man
#         self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
#         self.sensor_options = sensor_options
#         self.timer = CustomTimer()
#         self.ros_image = None
#         self.pc = None
#         self.radar_tracks = defaultdict(list)
#         self.radar = None

#         self.display_man.add_sensor(self)

#     def init_sensor(self, sensor_type, transform, attached, sensor_options):
#         if sensor_type == 'RGBCamera':
#             self.img_queue = queue.Queue()
#             # self.display_man.add_sensor(self)
#             camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
#             # disp_size = self.display_man.get_display_size()
#             height = 3840
#             width = 2160
#             camera_bp.set_attribute('image_size_x', '%s' % height)
#             camera_bp.set_attribute('image_size_y', '%s' % width)
#             # camera_bp.set_attribute('sensor_tick', '0.1')

#             for key in sensor_options:
#                 camera_bp.set_attribute(key, sensor_options[key])
#             self.sensor_name = camera_bp.get_attribute('role_name').as_str()

#             # output_path = '/home/yuchen.wang/Documents/_out/%s' % (self.sensor_name)
#             # print(output_path)
#             # if not os.path.exists(output_path):
#             #     print("create")
#             #     os.mkdir(output_path)
#             # else:
#             #     files = glob.glob(output_path+'/*')
#             #     for f in files:
#             #         os.remove(f)

#             camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
#             calibration = np.identity(3)
#             calibration[0, 2] = height / 2.0
#             calibration[1, 2] = width / 2.0
#             calibration[0, 0] = calibration[1, 1] = height / (2.0 * np.tan(camera_bp.get_attribute('fov').as_float() * np.pi / width))
#             camera.calibration = calibration
#             camera.listen(self.camera_callback)

#             # # spawn seg cam
#             # segcamera_bp = self.world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
#             # segcamera_bp.set_attribute('image_size_x', '1920')
#             # segcamera_bp.set_attribute('image_size_y', '1080')

#             # segcamera = self.world.spawn_actor(segcamera_bp, transform, attach_to=attached)
#             # print("spawned seg cam")
#             # segcamera.listen(self.save_seg_image)

#             # # spawn depth cam
#             # depcamera_bp = self.world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
#             # depcamera_bp.set_attribute('image_size_x', '1920')
#             # depcamera_bp.set_attribute('image_size_y', '1080')

#             # depcamera = self.world.spawn_actor(depcamera_bp, transform, attach_to=attached)
#             # depcamera.listen(self.save_seg_image)


#             return camera

#         elif sensor_type == 'SegCamera':
#             camera_bp = self.world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
#             camera_bp.set_attribute('image_size_x', '3840')
#             camera_bp.set_attribute('image_size_y', '2160')

#             for key in sensor_options:
#                 camera_bp.set_attribute(key, sensor_options[key])

#             self.sensor_name = camera_bp.get_attribute('role_name').as_str()

#             camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
#             camera.listen(self.save_seg_image)

#             return camera

#         elif sensor_type == 'DepthCamera':
#             camera_bp = self.world.get_blueprint_library().find('sensor.camera.depth')
#             camera_bp.set_attribute('image_size_x', '3840')
#             camera_bp.set_attribute('image_size_y', '2160')

#             for key in sensor_options:
#                 camera_bp.set_attribute(key, sensor_options[key])

#             self.sensor_name = camera_bp.get_attribute('role_name').as_str()

#             camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
#             camera.listen(self.save_dep_image)

#             return camera

#         elif sensor_type == 'LiDAR':
#             self.lidar_queue = queue.Queue()
#             lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
#             self.M = utils.get_transformation_matrix(transform)

#             for key in sensor_options:
#                 lidar_bp.set_attribute(key, sensor_options[key])
#             self.sensor_name = lidar_bp.get_attribute('role_name').as_str()

#             lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

#             lidar.listen(lambda data: self.lidar_callback(data, self.lidar_queue))

#             return lidar
        
#         elif sensor_type == 'FMCW':
#             self.lidar_queue = queue.Queue()
#             self.latest_lidar = None
#             lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.fmcw')
#             self.sensor_name = lidar_bp.get_attribute('role_name').as_str()
#             self.M = utils.get_transformation_matrix(transform)
#             executable_dir = os.path.dirname(os.path.abspath(__file__))
#             pattern_yaml_path = os.path.abspath(
#                 os.path.join(executable_dir, "../../../../ScanPatterns.yaml"))
#             # pattern_yaml_path = "/home/yuchen.wang/workspace/carla/ScanPatterns.yaml"

#             lidar_bp.set_attribute('pattern_file', pattern_yaml_path)
#             lidar_bp.set_attribute('pattern_name', '64-19.2-Uniform')
#             lidar_bp.set_attribute('motion_compensate', 'true')

#             raycast_modes = {'frame': '0', 'line': '1', 'point': '2'}
#             lidar_bp.set_attribute('raycast_mode', raycast_modes['frame'])

#             # if args.no_noise:
#             # lidar_bp.set_attribute('dropoff_general_rate', '0.0')
#             # lidar_bp.set_attribute('dropoff_intensity_limit', '1.0')
#             # lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')
#             # else:
#             lidar_bp.set_attribute('noise_stddev', '0.05')
#             lidar_bp.set_attribute('dropoff_general_rate', '0.3')

#             for key in sensor_options:
#                 lidar_bp.set_attribute(key, sensor_options[key])

#             try:
#                 lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)
#                 lidar.listen(lambda data: self.fmcw_callback(data, self.lidar_queue))

#                 return lidar
#             except Exception as e:
#                 print("Error spawing FMCW lidar: {}".format(e))

#         elif sensor_type == "Radar":
#             radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
#             self.z_offset = transform.location.z
#             self.ground_removal_threshold = 0.1 if self.z_offset < -0.5 else 0.2
#             for key in sensor_options:
#                 radar_bp.set_attribute(key, sensor_options[key])
#             self.sensor_name = radar_bp.get_attribute('role_name').as_str()
#             self.obstacles = {}
#             self.v = [0., 0., 0.]

#             radar = self.world.spawn_actor(radar_bp, transform, attach_to=attached)
#             radar.listen(self.radar_callback)

#             return radar

#         else:
#             return None

#     def save_seg_image(self, image):
#         image.save_to_disk('/home/yuchen.wang/Documents/_out/%s/seg%06d.jpg' % (self.sensor_name, image.frame), carla.ColorConverter.CityScapesPalette)

#     def save_dep_image(self, image):
#         image.save_to_disk('/home/yuchen.wang/Documents/_out/%s/dep%06d.jpg' % (self.sensor_name, image.frame), carla.ColorConverter.Depth)
    
#     def capture_image(self, sensor_img):
#         """Captures an image from a CARLA camera sensor and converts it to ROS CompressedImage format."""
#         image = np.array(sensor_img.raw_data).reshape((sensor_img.height, sensor_img.width, 4))
#         image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
#         # Resize the image to 1920x1080
#         resized_image = cv2.resize(image, (1920, 1080), interpolation=cv2.INTER_AREA)
#         _, jpeg = cv2.imencode('.jpg', resized_image)

#         # array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
#         # array = np.reshape(array, (image.height, image.width, 4))
#         # array = array[:, :, :3]
#         # cv2.imwrite('/home/yuchen.wang/Documents/_out/%s/frame%06d.jpg' % (self.sensor_name, sensor_img.frame), array, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
#         # del array
#         # import gc
#         # gc.collect()

#         self.ros_image = Image()
#         self.ros_image.header.stamp = rospy.Time.now()
#         self.ros_image.format = "jpeg"
#         self.ros_image.data = np.array(jpeg).tobytes()

#     def camera_callback(self, image):
#         image.convert(carla.ColorConverter.Raw)
#         array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
#         array = np.reshape(array, (image.height, image.width, 4))
#         array = array[:, :, :3]
#         # array = array[:, :, ::-1]

#         # if self.display_man.render_enabled():
#         #     self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

#         if SIM_STARTED:
#             self.img_queue.put(image)
#             # cv2.imwrite('/home/yuchen.wang/Documents/_out/%s/frame%06d.jpg' % (self.sensor_name, image.frame), array, [int(cv2.IMWRITE_JPEG_QUALITY), 95])

#         # del array
#         import gc
#         gc.collect()

#     def lidar_callback(self, data, lidar_queue):
#         points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
#         points = np.reshape(points, (int(points.shape[0] / 4), 4))

#         self.pc = points.copy()
#         self.pc[:,1] = -self.pc[:,1]
#         if SIM_STARTED:
#             lidar_queue.put(self.pc)

#     def fmcw_callback(self, data, lidar_queue):
#         point_cloud = np.frombuffer(data.raw_data,
#                                     dtype=np.dtype([('azimuth', np.float32),
#                                                     ('elevation', np.float32),
#                                                     ('range', np.float32),
#                                                     ('intensity', np.float32),
#                                                     ('velocity', np.float32),
#                                                     ('cos_angle', np.float32),
#                                                     ('instance', np.uint32),
#                                                     ('class', np.uint32),
#                                                     ('point_idx', np.uint32),
#                                                     ('beam_idx', np.ubyte),
#                                                     ('valid', bool),
#                                                     ('dynamic', bool)])).copy()

#         # Negate the azimuth to account for scanning direction
#         point_cloud['azimuth'] = -point_cloud['azimuth']

#         point_cloud = point_cloud[point_cloud['valid'] == 1]

#         points = np.array(
#             utils.spherical_to_cartesian(point_cloud['azimuth'], point_cloud['elevation'], 
#                                    point_cloud['range'])).T
#         # print(point_cloud['range'])
#         intensity = point_cloud['intensity'].reshape(-1, 1)  # shape: (N, 1)

#         points = np.hstack((points, intensity))  # shape: (N, 4)
#         if SIM_STARTED:
#             lidar_queue.put(points)

#     # def generate_pseudo_id(self, detection):
#     #     # Hash-based ID using depth, azimuth, altitude
#     #     key = f"{round(detection.depth, 2)}_{round(detection.azimuth, 2)}_{round(detection.altitude, 2)}"
#     #     return hash(key) % 65536

#     def radar_callback(self, data):
#         self.radar_tracks.clear()
#         self.radar = RadarTrackArray()
#         self.radar.header = rospy.Header()
#         self.radar.header.frame_id = "radar"
#         self.cur_tf = self.sensor.get_transform()

#         for detection in data:
#             # id = self.generate_pseudo_id(detection)
#             # print(id)
#             id = detection.actor_id
#             if id not in self.obstacles.keys():
#                 continue

#             # Convert from spherical to cartesian
#             depth = detection.depth
#             azimuth = detection.azimuth
#             altitude = detection.altitude
#             velocity = detection.velocity

#             obs = self.obstacles[id]
#             obs_pos = np.array([obs[0], obs[1], 0])
#             obs_v = np.array([obs[2], obs[3], 0])
#             radar_pos = np.array([self.cur_tf.location.x, self.cur_tf.location.y, self.cur_tf.location.z])
#             radar_v = np.array([self.v[0], self.v[1], self.v[2]])
#             velocity = utils.calculate_radar_speed(obs_pos, obs_v, radar_pos, radar_v)

#             x = depth * math.cos(azimuth) * math.cos(-altitude)
#             y = depth * math.sin(-azimuth) * math.cos(altitude)
#             z = depth * math.sin(altitude)

#             vx = velocity * math.cos(azimuth) * math.cos(-altitude)
#             vy = velocity * math.sin(-azimuth) * math.cos(altitude)

#             if abs(1.288+self.z_offset+z) < self.ground_removal_threshold:
#                 continue
#             if depth < 1:
#                 continue

#             self.radar_tracks[id].append({'x': x, 'y': y, 'z': z, 'vx': vx, 'vy': vy})

#         for track_id, points in self.radar_tracks.items():
#             radar_track = RadarTrack()
#             sumx = sum(p['x'] for p in points)
#             sumy = sum(p['y'] for p in points)
#             sumz = sum(p['z'] for p in points)
#             sumvx = sum(p['vx'] for p in points)
#             sumvy = sum(p['vy'] for p in points)

#             count = len(points)
#             point = Point32()
#             point.x = sumx / count
#             point.y = sumy / count
#             point.z = sumz / count

#             radar_track.track_shape.points.append(point)
#             radar_track.linear_velocity.x = sumvx / count
#             radar_track.linear_velocity.y = sumvy / count
#             radar_track.linear_acceleration.x = 0.0
#             radar_track.track_id = track_id

#             # print(point.z)

#             self.radar.tracks.append(radar_track)
#         # print("===\n")

#     def render(self, bounding_boxes):
#         if self.surface is not None:
#             for bbox in bounding_boxes:
#                 points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]
#                 # draw lines
#                 # base
#                 pygame.draw.line(self.surface, BB_COLOR, points[0], points[1])
#                 pygame.draw.line(self.surface, BB_COLOR, points[0], points[1])
#                 pygame.draw.line(self.surface, BB_COLOR, points[1], points[2])
#                 pygame.draw.line(self.surface, BB_COLOR, points[2], points[3])
#                 pygame.draw.line(self.surface, BB_COLOR, points[3], points[0])
#                 # top
#                 pygame.draw.line(self.surface, BB_COLOR, points[4], points[5])
#                 pygame.draw.line(self.surface, BB_COLOR, points[5], points[6])
#                 pygame.draw.line(self.surface, BB_COLOR, points[6], points[7])
#                 pygame.draw.line(self.surface, BB_COLOR, points[7], points[4])
#                 # base-top
#                 pygame.draw.line(self.surface, BB_COLOR, points[0], points[4])
#                 pygame.draw.line(self.surface, BB_COLOR, points[1], points[5])
#                 pygame.draw.line(self.surface, BB_COLOR, points[2], points[6])
#                 pygame.draw.line(self.surface, BB_COLOR, points[3], points[7])
            # offset = self.display_man.get_display_offset(self.display_pos)
#             self.display_man.display.blit(self.surface, offset)

#     def destroy(self):
#         self.sensor.destroy()

def wait_for_image(cam):
    try:
        img = cam.img_queue.get(timeout=1)
        cam.capture_image(img)
    except:
        print("Sensor '{}' did not return data in time".format(cam.sensor_name))

def handle_plus_vehicle_control(req):
    global vehicle, s_fc, s_fl, s_fr, s_lf, s_ls, s_lr, s_rf, s_rs, s_rr, lidar, left_aeva, right_aeva, radar_front_lr, radar_l_rear_sr, radar_r_rear_sr, obs_vehicle_list
    global SPAWN_OBS, yaw_offset, SIM_STARTED
    if not vehicle:
        rospy.logerr("Vehicle not initialized!")
        return SimConnectResponse(False, Image(), Image(), Image(), Image(), Image(), Image(), Image(), Image(), Image(), None, None)
    if not SIM_STARTED:
        SIM_STARTED = True
    # print("handle start: {}".format(time.time()))

    # Move the vehicle
    ego_lat = req.ego_lat
    ego_lon = req.ego_lon
    ego_x, ego_y = utils.convert_to_carla_coord(ego_lat, ego_lon)
    ego_vx = req.ego_vx
    ego_vy = req.ego_vy
    ego_yaw = -math.degrees(req.ego_yaw)
    new_transform = carla.Transform(carla.Location(x=ego_x, y=ego_y), carla.Rotation(yaw=ego_yaw))
    vehicle.set_transform(new_transform)
    _vehicle_moved = True
    yaw_diff = -math.degrees(req.yaw_offset) - yaw_offset
    yaw_offset = -math.degrees(req.yaw_offset)
    # print(yaw_offset)
    ## Assign new obs pose
    obs_info = req.obstacle_info
    pb = obstacle_detection_pb2.ObstacleDetection()
    pb.ParseFromString(obs_info.data)
    obs_for_radar = {}
    for obstacle in pb.obstacle:
        cur_id = obstacle.id
        cur_motion = obstacle.motion
        cur_x, cur_y = utils.plus_xy_to_carla_xy(cur_motion.x, cur_motion.y)
        cur_obs = obs_vehicle_list[cur_id]
        cur_tf = cur_obs.get_transform()
        cur_yaw = -math.degrees(cur_motion.yaw)
        new_rot = carla.Rotation(yaw=cur_yaw + yaw_offset)
        # new_rot = carla.Rotation(yaw=ego_yaw)
        new_loc = carla.Location(x=cur_x, y=cur_y, z=cur_tf.location.z)
        obs_for_radar[cur_obs.id] = [cur_x, cur_y, cur_motion.vx, cur_motion.vy]
        cur_obs.set_transform(carla.Transform(new_loc, new_rot))

    # print("done processing obs: {}".format(time.time()))

    ## Wait till all sensor queues are ready
    with futures.ThreadPoolExecutor() as executor:
        executor.map(wait_for_image, [s_fc, s_fl, s_fr, s_lf, s_ls, s_lr, s_rf, s_rs, s_rr])

    # print("handle end: {}".format(time.time()))

    radar_front_lr.obstacles = obs_for_radar
    radar_l_rear_sr.obstacles = obs_for_radar
    radar_r_rear_sr.obstacles = obs_for_radar
    radar_front_lr.v = [ego_vx, ego_vy, 0.]
    radar_l_rear_sr.v = [ego_vx, ego_vy, 0.]
    radar_r_rear_sr.v = [ego_vx, ego_vy, 0.]


    # if not SPAWN_OBS:
    #     yaw_offset = -math.degrees(req.yaw_offset)
    #     for _, obs in obs_vehicle_list:
    #         cur_tf = obs.get_transform()
    #         new_rot = carla.Rotation(yaw=cur_tf.rotation.yaw+yaw_offset)
    #         obs.set_transform(carla.Transform(cur_tf.location, new_rot))
    #         obs.set_autopilot(True)
    #     SPAWN_OBS = True

    # Get bounding boxes
    bounding_boxes_map = {}
    bounding_boxes_map[s_fc.sensor_name] = []
    for s in [s_fl, s_fr, s_lf, s_ls, s_lr, s_rf, s_rf, s_rs, s_rr]:
        bounding_boxes_map[s.sensor_name] = []
        # bounding_boxes_map[s.sensor_name] = ClientSideBoundingBoxes.get_bounding_boxes(obs_vehicle_list, s.sensor)
    # Render received data
    # display_manager.render(bounding_boxes_map)
    unify_lidar.clear()

    return SimConnectResponse(True,
                              s_fl.ros_image,
                              s_fr.ros_image,
                              s_fc.ros_image,
                              s_lf.ros_image,
                              s_ls.ros_image,
                              s_lr.ros_image,
                              s_rf.ros_image,
                              s_rs.ros_image,
                              s_rr.ros_image,
                              left_aeva.latest_lidar,
                              radar_front_lr.radar,
                              radar_l_rear_sr.radar,
                              radar_r_rear_sr.radar,
                            #   RadarTrackArray(),
                            #   RadarTrackArray(),
                            #   RadarTrackArray()
                              )

def run_simulation(args, client):
    """This function performed one test run using the args parameters
    and connecting to the carla client passed.
    """
    vehicle_list = []
    timer = CustomTimer()

    import logging
    logging.basicConfig(level=logging.DEBUG)

    global vehicle, s_fc, s_fl, s_fr, s_lf, s_ls, s_lr, s_rf, s_rf, s_rs, s_rr, lidar, left_aeva, right_aeva, radar_front_lr, radar_l_rear_sr, radar_r_rear_sr, obs_vehicle_list

    try:
        # Connect to simulator node
        rospy.init_node('plus_carla_server')
        s = rospy.Service('plus_carla', SimConnect, handle_plus_vehicle_control)

        # Getting the world and
        world = client.get_world()
        original_settings = world.get_settings()
        sp = utils.ScenarioProcessor(args.scenario_file)

        # world.set_weather(carla.WeatherParameters.WetCloudyNoon)

        if args.sync:
            traffic_manager = client.get_trafficmanager(8000)
            settings = world.get_settings()
            traffic_manager.set_synchronous_mode(True)
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)


        # Instanciating the vehicle to which we attached the sensors
        spectator = world.get_spectator()
        bp = world.get_blueprint_library().filter('vehicle.mercedes.sprinter')[0]
        init_x = -2083.1795672688713
        init_y = 2909.9611540967576
        init_x, init_y = sp.get_ego()
        transform = carla.Transform(
            # carla.Location(-2251.36285433037, 3160.0232528821402, 3), #I35-New lat_lon(29.7829405, -98.031983)
            carla.Location(init_x, init_y, 3), #(29.785196679, -98.030244176)
            carla.Rotation(yaw=args.init_yaw))
        spectator.set_transform(transform)
        vehicle = world.spawn_actor(bp, transform)
        # vehicle.set_light_state(carla.VehicleLightState(carla.VehicleLightState.HighBeam))
        vehicle_list.append(vehicle)
        # vehicle.set_autopilot(True)
        box = vehicle.bounding_box
        print(box.extent) 

        # Display Manager organize all the sensors an its display in a window
        # If can easily configure the grid and the total window size
        # display_manager = DisplayManager(grid_size=[3, 3], window_size=[args.width, args.height])

        # IMU
        imu_bp = world.get_blueprint_library().find('sensor.other.imu')
        imu = world.spawn_actor(imu_bp, carla.Transform(carla.Location(x=0.497595, z=1.288), carla.Rotation(yaw=+00)), attach_to=vehicle)

        # Front Wide Camera
        s_fc = SensorManager(world, 'RGBCamera', carla.Transform(carla.Location(x=2.46, y=0, z=1.12), carla.Rotation(yaw=+00)), 
                             imu, {'fov': '121.2'})
        # seg_fc = SensorManager(world, 'SegCamera', carla.Transform(carla.Location(x=2.46, y=0, z=1.12), carla.Rotation(yaw=+00)), 
        #                      imu, {'fov': '121.2'})
        # d_fc = SensorManager(world, 'DepthCamera', carla.Transform(carla.Location(x=2.46, y=0, z=1.12), carla.Rotation(yaw=+00)), 
        #                      imu, {'fov': '121.2'})
        
        # Front Left Camera
        s_fl = SensorManager(world, 'RGBCamera', carla.Transform(carla.Location(x=2.365, y=-0.77, z=1.12), carla.Rotation(yaw=+00)), 
                             imu, {'fov': '47', 'role_name': 'front_left_camera'})
        # seg_fl = SensorManager(world, 'SegCamera', carla.Transform(carla.Location(x=2.365, y=-0.77, z=1.12), carla.Rotation(yaw=+00)), 
        #                      imu, {'fov': '47', 'role_name': 'front_left_camera'})
        # d_fl = SensorManager(world, 'DepthCamera', carla.Transform(carla.Location(x=2.365, y=-0.77, z=1.12), carla.Rotation(yaw=+00)), 
        #                      imu, {'fov': '47', 'role_name': 'front_left_camera'})
        
        # Front Right Camera
        s_fr = SensorManager(world, 'RGBCamera', carla.Transform(carla.Location(x=2.45, y=0.318, z=1.12), carla.Rotation(yaw=+00)), 
                             imu, {'fov': '47', 'role_name': 'front_right_camera'})
        # seg_fr = SensorManager(world, 'SegCamera', carla.Transform(carla.Location(x=2.45, y=0.318, z=1.12), carla.Rotation(yaw=+00)), 
        #                      imu, {'fov': '47', 'role_name': 'front_right_camera'})
        # d_fr = SensorManager(world, 'DepthCamera', carla.Transform(carla.Location(x=2.45, y=0.318, z=1.12), carla.Rotation(yaw=+00)), 
        #                      imu, {'fov': '47', 'role_name': 'front_right_camera'})
        
        # Left Front Camera
        s_lf = SensorManager(world, 'RGBCamera', carla.Transform(carla.Location(x=-0.077, y=-1.265, z=1.487), carla.Rotation(roll=-90, pitch=-10, yaw=-33)), 
                             imu, {'fov': '121.2', 'role_name': 'left_front_camera'})
        # seg_lf = SensorManager(world, 'SegCamera', carla.Transform(carla.Location(x=-0.077, y=-1.265, z=1.487), carla.Rotation(roll=-90, pitch=-10, yaw=-33)), 
        #                      imu, {'fov': '121.2', 'role_name': 'left_front_camera'})
        # d_lf = SensorManager(world, 'DepthCamera', carla.Transform(carla.Location(x=-0.077, y=-1.265, z=1.487), carla.Rotation(roll=-90, pitch=-10, yaw=-33)), 
        #                      imu, {'fov': '121.2', 'role_name': 'left_front_camera'})
        
        # Left Side Camera
        s_ls = SensorManager(world, 'RGBCamera', carla.Transform(carla.Location(x=-0.154, y=-1.293, z=1.487), carla.Rotation(roll=-90, pitch=-32, yaw=-93)), 
                             imu, {'fov': '121.2', 'role_name': 'left_side_camera'})
        # seg_ls = SensorManager(world, 'SegCamera', carla.Transform(carla.Location(x=-0.154, y=-1.293, z=1.487), carla.Rotation(roll=-90, pitch=-32, yaw=-93)), 
        #                      imu, {'fov': '121.2', 'role_name': 'left_side_camera'})
        # d_ls = SensorManager(world, 'DepthCamera', carla.Transform(carla.Location(x=-0.154, y=-1.293, z=1.487), carla.Rotation(roll=-90, pitch=-32, yaw=-93)), 
        #                      imu, {'fov': '121.2', 'role_name': 'left_side_camera'})
        
        # Left Rear Camera
        s_lr = SensorManager(world, 'RGBCamera', carla.Transform(carla.Location(x=-0.232, y=-1.283, z=1.487), carla.Rotation(roll=-90, pitch=-4, yaw=-153)), 
                             imu, {'fov': '121.2', 'role_name': 'left_rear_camera'})
        # seg_lr = SensorManager(world, 'SegCamera', carla.Transform(carla.Location(x=-0.232, y=-1.283, z=1.487), carla.Rotation(roll=-90, pitch=-4, yaw=-153)), 
        #                      imu, {'fov': '121.2', 'role_name': 'left_rear_camera'})
        # d_lr = SensorManager(world, 'DepthCamera', carla.Transform(carla.Location(x=-0.232, y=-1.283, z=1.487), carla.Rotation(roll=-90, pitch=-4, yaw=-153)), 
        #                      imu, {'fov': '121.2', 'role_name': 'left_rear_camera'})
        
        # Right Front Camera
        s_rf = SensorManager(world, 'RGBCamera', carla.Transform(carla.Location(x=-0.077, y=1.265, z=1.487), carla.Rotation(roll=-90, pitch=-10, yaw=33)), 
                             imu, {'fov': '121.2', 'role_name': 'right_front_camera'})
        # seg_rf = SensorManager(world, 'SegCamera', carla.Transform(carla.Location(x=-0.077, y=1.265, z=1.487), carla.Rotation(roll=-90, pitch=-10, yaw=33)), 
        #                      imu, {'fov': '121.2', 'role_name': 'right_front_camera'})
        # d_rf = SensorManager(world, 'DepthCamera', carla.Transform(carla.Location(x=-0.077, y=1.265, z=1.487), carla.Rotation(roll=-90, pitch=-10, yaw=33)), 
        #                      imu, {'fov': '121.2', 'role_name': 'right_front_camera'})
        
        # Right Side Camera
        s_rs = SensorManager(world, 'RGBCamera', carla.Transform(carla.Location(x=-0.154, y=1.293, z=1.487), carla.Rotation(roll=-90, pitch=-32, yaw=93)), 
                             imu, {'fov': '121.2', 'role_name': 'right_side_camera'})
        # seg_rs = SensorManager(world, 'SegCamera', carla.Transform(carla.Location(x=-0.154, y=1.293, z=1.487), carla.Rotation(roll=-90, pitch=-32, yaw=93)), 
        #                      imu, {'fov': '121.2', 'role_name': 'right_side_camera'})
        # d_rs = SensorManager(world, 'DepthCamera', carla.Transform(carla.Location(x=-0.154, y=1.293, z=1.487), carla.Rotation(roll=-90, pitch=-32, yaw=93)), 
        #                      imu, {'fov': '121.2', 'role_name': 'right_side_camera'})
        
        # Right Rear Camera
        s_rr = SensorManager(world, 'RGBCamera', carla.Transform(carla.Location(x=-0.232, y=1.283, z=1.487), carla.Rotation(roll=-90, pitch=-4, yaw=153)), 
                             imu, {'fov': '121.2', 'role_name': 'right_rear_camera'})
        # seg_rr = SensorManager(world, 'SegCamera', carla.Transform(carla.Location(x=-0.232, y=1.283, z=1.487), carla.Rotation(roll=-90, pitch=-4, yaw=153)), 
        #                      imu, {'fov': '121.2', 'role_name': 'right_rear_camera'})
        # d_rr = SensorManager(world, 'DepthCamera', carla.Transform(carla.Location(x=-0.232, y=1.283, z=1.487), carla.Rotation(roll=-90, pitch=-4, yaw=153)), 
        #                      imu, {'fov': '121.2', 'role_name': 'right_rear_camera'})
        
        # Lidars
        left_os0 = SensorManager(world, 'LiDAR', carla.Transform(carla.Location(x=0.722, y=-1.231, z=-0.145), carla.Rotation(roll=-35)), 
                      imu, {'channels' : '128', 'range' : '75', 'horizontal_fov': '360', 'upper_fov': '45', \
                            'lower_fov': '-45', 'points_per_second': '3300000'})

        right_os0 = SensorManager(world, 'LiDAR', carla.Transform(carla.Location(x=0.722, y=1.231, z=-0.145), carla.Rotation(roll=35)), 
                      imu, {'channels' : '128', 'range' : '75', 'horizontal_fov': '360', 'upper_fov': '45', \
                            'lower_fov': '-45', 'points_per_second': '3300000'})
        
        left_aeva = SensorManager(world, 'FMCW', carla.Transform(carla.Location(x=2.48, y=-0.097, z=1.557), carla.Rotation(pitch=-3.2, yaw=-10)), 
                             imu, {'channels' : '64', 'range' : '500', 'role_name': 'left_aeva'})
        
        right_aeva = SensorManager(world, 'FMCW', carla.Transform(carla.Location(x=2.48, y=0.097, z=1.557), carla.Rotation(pitch=-3.2, yaw=10)), 
                             imu, {'channels' : '64', 'range' : '500', 'role_name': 'right_aeva'})

        # left_os2 = SensorManager(world, 'LiDAR', carla.Transform(carla.Location(x=-0.128, y=-1.26, z=1.594), carla.Rotation(pitch=-5, yaw=45)), 
        #               imu, {'channels' : '128', 'range' : '350', 'horizontal_fov': '360', 'upper_fov': '11.25', \
        #                     'lower_fov': '-11.25', 'points_per_second': '2300000'})

        # right_os2 = SensorManager(world, 'LiDAR', carla.Transform(carla.Location(x=-0.128, y=1.26, z=1.594), carla.Rotation(pitch=-5, yaw=-45)), 
        #               imu, {'channels' : '128', 'range' : '350', 'horizontal_fov': '360', 'upper_fov': '11.25', \
        #                     'lower_fov': '-11.25', 'points_per_second': '2300000'})
        
        # Radars - LR
        radar_front_lr = SensorManager(world, 'Radar', carla.Transform(carla.Location(x=4.408, y=-0.245, z=0.715), carla.Rotation(yaw=-25)), 
                             imu, {'range': '320', 'vertical_fov': '23', 'horizontal_fov': '110', 'points_per_second': '10000', 'role_name': 'front_center_radar'})
        # radar_l_rear_lr = SensorManager(world, 'Radar', carla.Transform(carla.Location(x=0.018, y=-1.295, z=1.752), carla.Rotation(yaw=180)), 
        #                      imu, {'range': '320', 'vertical_fov': '23', 'horizontal_fov': '110'})
        # radar_r_rear_lr = SensorManager(world, 'Radar', carla.Transform(carla.Location(x=0.018, y=1.295, z=1.752), carla.Rotation(yaw=180)), 
        #                      imu, {'range': '320', 'vertical_fov': '23', 'horizontal_fov': '110'})
        # radar_l_corner_lr = SensorManager(world, 'Radar', carla.Transform(carla.Location(x=3.848, y=-1.185, z=-0.548), carla.Rotation(yaw=90)), 
        #                      imu, {'range': '320', 'vertical_fov': '23', 'horizontal_fov': '110'})
        # radar_r_corner_lr = SensorManager(world, 'Radar', carla.Transform(carla.Location(x=3.848, y=1.185, z=-0.548), carla.Rotation(yaw=-90)), 
        #                      imu, {'range': '320', 'vertical_fov': '23', 'horizontal_fov': '110'})

        # Radars - SR
        # radar_front_sr = SensorManager(world, 'Radar', carla.Transform(carla.Location(x=4.438, y=0.005, z=-0.733), carla.Rotation(yaw=25)), 
        #                      imu, {'range': '320', 'vertical_fov': '23', 'horizontal_fov': '110', 'points_per_second': '1000'})
        radar_l_rear_sr = SensorManager(world, 'Radar', carla.Transform(carla.Location(x=1.146, y=-1.296, z=-0.758), carla.Rotation(yaw=-125)), 
                             imu, {'range': '80', 'vertical_fov': '23', 'horizontal_fov': '110', 'points_per_second': '100000'})
        radar_r_rear_sr = SensorManager(world, 'Radar', carla.Transform(carla.Location(x=1.146, y=1.296, z=-0.758), carla.Rotation(yaw=125)), 
                             imu, {'range': '80', 'vertical_fov': '23', 'horizontal_fov': '110', 'points_per_second': '100000', 'role_name': 'rear_right_radar'})
        # radar_l_front_sr = SensorManager(world, 'Radar', carla.Transform(carla.Location(x=1.23, y=-1.296, z=-0.638), carla.Rotation(yaw=55)), 
        #                      imu, {'range': '80', 'vertical_fov': '23', 'horizontal_fov': '110'})
        # radar_r_front_sr = SensorManager(world, 'Radar', carla.Transform(carla.Location(x=1.23, y=-1.296, z=-0.638), carla.Rotation(yaw=-55)), 
        #                      imu, {'range': '80', 'vertical_fov': '23', 'horizontal_fov': '110'})
        
        # Get camera sensor
        # camera = s_lf.sensor

        # # Image resolution
        # image_w, image_h = 1920, 1080

        # Get calibration
        # get_camera_calibration(camera, image_w, image_h)

        # Spawning obstacles
        obs_map = sp.obs_map
        tm = client.get_trafficmanager()

        obs_blueprints = world.get_blueprint_library().filter('vehicle.*.*')
        filtered_obs_blueprints = [bp for bp in obs_blueprints if not (bp.id.startswith('vehicle.bicycle.') \
        or bp.id.startswith('vehicle.bh.') or bp.id.startswith('vehicle.diamondback.') \
        or bp.id.startswith('vehicle.gazelle.') or bp.id.startswith('vehicle.vespa.'))]
        for obs_id, state in obs_map.items():
            obsyaw = args.init_yaw
            if obs_id > 22:
                obsyaw = args.init_yaw + 180
            obs_tf = carla.Transform(
                carla.Location(state[0], state[1], 2),
                # carla.Rotation(yaw=state[2]))
                carla.Rotation(yaw=obsyaw))
            # obs_bp = random.choice(world.get_blueprint_library().filter('vehicle.*.*'))
            bp_random = random.choice(filtered_obs_blueprints)
            obs = world.try_spawn_actor(bp_random, obs_tf)
            # tm.set_desired_speed(obs, state[3] * 3.6)
            if obs is not None:
                print('created {} with id = {}, carla_id = {}, target speed {}'.format(obs.type_id, obs_id, obs.id, state[3]))
                obs.set_autopilot(False)
                # obs.set_light_state(carla.VehicleLightState(carla.VehicleLightState.HighBeam))
                obs_vehicle_list[obs_id] = obs
            # time.sleep(1)

        clock = pygame.time.Clock()
        #Simulation loop
        call_exit = False
        while not rospy.is_shutdown():
            # Carla Tick
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()

            try:
                if SIM_STARTED:
                    left_latest = left_aeva.lidar_queue.get(True, 1.0)
                    right_latest = right_aeva.lidar_queue.get(True, 1.0)
                    right_latest_b = utils.transform_between_frames(right_latest, right_aeva.M, left_aeva.M)
                    left_os0_b = utils.transform_between_frames(left_os0.lidar_queue.get(True, 1.0), left_os0.M, left_aeva.M, offset_y=2.462, offset_z=0.29)
                    right_os0_b = utils.transform_between_frames(right_os0.lidar_queue.get(True, 1.0), right_os0.M, left_aeva.M, offset_y=-2.462, offset_z=0.29)
                    # left_os2_b = utils.transform_between_frames(left_os2.lidar_queue.get(True, 1.0), left_os2.M, left_aeva.M, offset_y=2.52, offset_z=0)
                    # right_os2_b = utils.transform_between_frames(right_os2.lidar_queue.get(True, 1.0), right_os2.M, left_aeva.M, offset_y=-2.52, offset_z=-1.594)
                    # combined_points = np.vstack((left_latest, right_latest_b, left_os0_b, right_os0_b, left_os2_b, right_os2_b))
                    combined_points = np.vstack((left_latest, right_latest_b, left_os0_b, right_os0_b))

                    fields = [
                        PointField('x', 0, PointField.FLOAT32, 1),
                        PointField('y', 4, PointField.FLOAT32, 1),
                        PointField('z', 8, PointField.FLOAT32, 1),
                        PointField('intensity', 12, PointField.FLOAT32, 1)
                    ]

                    header = rospy.Header()
                    header.stamp = rospy.Time.now()
                    header.frame_id = 'lidar1_frame'  # or whatever LiDAR 1 uses

                    left_aeva.latest_lidar = pc2.create_cloud(header, fields, combined_points)
                    time.sleep(0.005)  # This can fix Open3D jittering issues.
            except queue.Empty:
                print("Timed out waiting for left LiDAR")

            # for event in pygame.event.get():
            #     if event.type == pygame.QUIT:
            #         call_exit = True
            #     elif event.type == pygame.KEYDOWN:
            #         if event.key == K_ESCAPE or event.key == K_q:
            #             call_exit = True
            #             break
            clock.tick(20)  # limit to 30 FPS

        rospy.spin()

    finally:
        if display_manager:
            display_manager.destroy()

        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])

        world.apply_settings(original_settings)


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Sensor tutorial')
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
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1920x1080',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--scenario-file',
        help='scenario file including ego and obstacle agents init poses')

    argparser.add_argument(
        '--init-yaw',
        type=float,
        default=-57)

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    try:
        # client = carla.Client(args.host, args.port)
        # client.set_timeout(5.0)
        # run_simulation(args, client)
        utils.plus_xy_to_carla_xy(-7639732.967067769, 10343934.726213053)

        time.sleep(1)
        # utils.generate_video('/home/yuchen.wang/Documents/_out/', '.jpg')
        # dirs = glob.glob('/home/yuchen.wang/Documents/_out/*')
        # for d in dirs:
        #     utils.generate_video(d, '.jpg')

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
