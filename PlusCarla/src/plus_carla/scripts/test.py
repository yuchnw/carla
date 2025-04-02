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
import datetime

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import argparse
import carla
import cv2
import rospy
import time
import numpy as np
import math
from pyproj import Proj
from plus_carla.srv import SimConnect, SimConnectResponse
from sensor_msgs.msg import CompressedImage as Image

import google.protobuf.text_format as protobuf_text_format
import simulation_data_pb2


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

class CustomTimer:
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time

    def time(self):
        return self.timer()
    
def sanitize_text(text, encoding='utf-8', errors='replace'):
    if isinstance(text, bytes):
        return text.decode(encoding, errors=errors)
    else:
        return str(text)

class DisplayManager:
    def __init__(self, grid_size, window_size):
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)

        self.grid_size = grid_size
        self.window_size = window_size
        self.sensor_list = []

    def get_window_size(self):
        return [int(self.window_size[0]), int(self.window_size[1])]

    def get_display_size(self):
        return [int(self.window_size[0]/self.grid_size[1]), int(self.window_size[1]/self.grid_size[0])]

    def get_display_offset(self, gridPos):
        dis_size = self.get_display_size()
        return [int(gridPos[1] * dis_size[0]), int(gridPos[0] * dis_size[1])]

    def add_sensor(self, sensor):
        self.sensor_list.append(sensor)

    def get_sensor_list(self):
        return self.sensor_list

    def render(self):
        if not self.render_enabled():
            return

        for s in self.sensor_list:
            s.render()

        pygame.display.flip()

    def destroy(self):
        for s in self.sensor_list:
            s.destroy()

    def render_enabled(self):
        return self.display != None

class SensorManager:
    def __init__(self, world, display_man, sensor_type, transform, attached, sensor_options, display_pos):
        self.surface = None
        self.world = world
        self.display_man = display_man
        self.display_pos = display_pos
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
        self.sensor_options = sensor_options
        self.timer = CustomTimer()
        self.ros_image = None

        self.display_man.add_sensor(self)

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == 'RGBCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            disp_size = self.display_man.get_display_size()
            camera_bp.set_attribute('image_size_x', '640')
            camera_bp.set_attribute('image_size_y', '360')

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image)

            return camera
        
        else:
            return None

    def get_sensor(self):
        return self.sensor
    
    def capture_image(self, sensor_img):
        """Captures an image from a CARLA camera sensor and converts it to ROS CompressedImage format."""
        image = np.array(sensor_img.raw_data).reshape((sensor_img.height, sensor_img.width, 4))
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        # Resize the image to 1920x1080
        resized_image = cv2.resize(image, (1920, 1080), interpolation=cv2.INTER_AREA)
        _, jpeg = cv2.imencode('.jpg', resized_image)
        
        self.ros_image = Image()
        self.ros_image.header.stamp = rospy.Time.now()
        self.ros_image.format = "jpeg"
        self.ros_image.data = np.array(jpeg).tobytes()

    def save_rgb_image(self, image):
        # t_start = self.timer.time()

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        # t_end = self.timer.time()

        self.capture_image(image)

    def render(self):
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)

    def destroy(self):
        self.sensor.destroy()

class ScenarioProcessor:
    def __init__(self, scenario_file):
        self.scenario = simulation_data_pb2.SimulationConfig()
        with open(scenario_file, 'r') as s_file:
            content = sanitize_text(s_file.read())
            protobuf_text_format.Merge(content, self.scenario)
        print(self.scenario)

def convert_to_carla_coord(lat, lon):
    proj_str = '+proj=tmerc +lat_0=29.81145 +lon_0=-98.0087 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs'
    proj = Proj(proj_str)
    x, y = proj(lon, lat)
    return x,-y

def handle_plus_vehicle_control(req):
    print(datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])
    global vehicle, s_fc, s_fl, s_fr, s_lf, s_ls, s_lr, s_rf, s_rf, s_rs, s_rr, display_manager
    if not vehicle:
        rospy.logerr("Vehicle not initialized!")
        return SimConnectResponse(False, Image(), Image(), Image(), Image(), Image(), Image(), Image(), Image(), Image())
    
    # Move the vehicle
    ego_lat = req.ego_lat
    ego_lon = req.ego_lon
    ego_x, ego_y = convert_to_carla_coord(ego_lat, ego_lon)
    ego_yaw = -math.degrees(req.ego_yaw)
    new_transform = carla.Transform(carla.Location(x=ego_x, y=ego_y), carla.Rotation(yaw=ego_yaw))
    # print(ego_lat, ego_lon)
    # print(ego_x, ego_y, ego_yaw)
    vehicle.set_transform(new_transform)

    # spectator_transform = vehicle.get_transform()
    # spectator_transform.location += carla.Location(x=0,y=0,z=6)
    # print(spectator_transform)
    # spectator.set_transform(spectator_transform)

    # Render received data
    display_manager.render()

    print(datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])
    print("===")

    return SimConnectResponse(True,
                              s_fl.ros_image,
                              s_fr.ros_image,
                              s_fc.ros_image,
                              s_lf.ros_image,
                              s_ls.ros_image,
                              s_lr.ros_image,
                              s_rf.ros_image,
                              s_rs.ros_image,
                              s_rr.ros_image)

def run_simulation(args, client):
    """This function performed one test run using the args parameters
    and connecting to the carla client passed.
    """
    vehicle_list = []
    timer = CustomTimer()

    global vehicle, s_fc, s_fl, s_fr, s_lf, s_ls, s_lr, s_rf, s_rf, s_rs, s_rr, display_manager

    try:
        # Connect to simulator node
        rospy.init_node('plus_carla_server')
        s = rospy.Service('plus_carla', SimConnect, handle_plus_vehicle_control)

        # Getting the world and
        world = client.get_world()
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
        transform = carla.Transform(
            carla.Location(-1790.853, 2528.455, 3), #I35-New lat_lon(29.7886388, -98.0272216)
            carla.Rotation())
        vehicle = world.spawn_actor(bp, transform)
        vehicle_list.append(vehicle)
        # vehicle.set_autopilot(True)
        box = vehicle.bounding_box
        print(box.extent) 

        # Display Manager organize all the sensors an its display in a window
        # If can easily configure the grid and the total window size
        display_manager = DisplayManager(grid_size=[3, 3], window_size=[args.width, args.height])

        # IMU
        imu_bp = world.get_blueprint_library().find('sensor.other.imu')
        imu = world.spawn_actor(imu_bp, carla.Transform(carla.Location(x=0.497595, z=1.288), carla.Rotation(yaw=+00)), attach_to=vehicle)

        # Front Wide Camera
        s_fc = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=2.46, y=0, z=1.12), carla.Rotation(yaw=+00)), 
                        imu, {'fov': '120'}, display_pos=[0, 1])
        
        # Front Left Camera
        s_fl = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=2.365, y=-0.77, z=1.12), carla.Rotation(yaw=+00)), 
                        imu, {'fov': '47'}, display_pos=[0, 0])
        
        # Front Right Camera
        s_fr = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=2.45, y=0.318, z=1.12), carla.Rotation(yaw=+00)), 
                        imu, {'fov': '47'}, display_pos=[0, 2])
        
        # Left Front Camera
        s_lf = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=-0.077, y=-1.265, z=1.487), carla.Rotation(roll=-90, pitch=-10, yaw=-33)), 
                        imu, {'fov': '120'}, display_pos=[1, 2])
        
        # Left Side Camera
        s_ls = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=-0.154, y=-1.293, z=1.487), carla.Rotation(roll=-90, pitch=-32, yaw=-93)), 
                        imu, {'fov': '120'}, display_pos=[1, 1])
        
        # Left Rear Camera
        s_lr = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=-0.232, y=-1.283, z=1.487), carla.Rotation(roll=-90, pitch=-4, yaw=-153)), 
                        imu, {'fov': '120'}, display_pos=[1, 0])
        
        # Right Front Camera
        s_rf = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=-0.077, y=1.265, z=1.487), carla.Rotation(roll=-90, pitch=-10, yaw=33)), 
                        imu, {'fov': '120'}, display_pos=[2, 0])
        
        # Right Side Camera
        s_rs = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=-0.154, y=1.293, z=1.487), carla.Rotation(roll=-90, pitch=-32, yaw=93)), 
                        imu, {'fov': '120'}, display_pos=[2, 1])
        
        # Right Rear Camera
        s_rr = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=-0.232, y=1.283, z=1.487), carla.Rotation(roll=-90, pitch=-4, yaw=153)), 
                        imu, {'fov': '120'}, display_pos=[2, 2])
        
        # Get camera sensor
        camera = s_lr.sensor

        # Image resolution
        image_w, image_h = 1920, 1080

        # Get calibration
        R, P, M, D = get_camera_calibration(camera, image_w, image_h)

        print("Intrinsic Matrix (M):\n", M)
        print("Distortion Coefficients (D):", D)
        print("Rectification Matrix (R):\n", R)
        print("Projection Matrix (P):\n", P)

        # Spawning obstacles
        obs1_tf = carla.Transform(
            carla.Location(-1796.130, 2529.299, 3),
            carla.Rotation(yaw=-45))
        obs1_bp = world.get_blueprint_library().filter('vehicle.lincoln.mkz_2017')[0]
        obs1 = world.try_spawn_actor(obs1_bp, obs1_tf)
        if obs1 is not None:
            obs1.set_autopilot(True)
            print('created %s' % obs1.type_id)

        time.sleep(1)

        obs2_tf = carla.Transform(
            carla.Location(-1776.360, 2517.982, 3),
            carla.Rotation(yaw=-45))
        obs2_bp = world.get_blueprint_library().filter('vehicle.ford.crown')[0]
        obs2 = world.try_spawn_actor(obs2_bp, obs2_tf)
        if obs2 is not None:
            obs2.set_autopilot(True)
            print('created %s' % obs2.type_id)

        obs3_tf = carla.Transform(
            carla.Location(-1749.236, 2482.427, 3),
            carla.Rotation(yaw=-45))
        obs3_bp = world.get_blueprint_library().filter('vehicle.chevrolet.impala')[0]
        obs3 = world.try_spawn_actor(obs3_bp, obs3_tf)
        if obs3 is not None:
            obs3.set_autopilot(True)
            print('created %s' % obs3.type_id)

        obs4_tf = carla.Transform(
            carla.Location(-1821.712, 2567.373, 3),
            carla.Rotation(yaw=-45))
        obs4_bp = world.get_blueprint_library().filter('vehicle.mercedes.coupe_2020')[0]
        obs4 = world.try_spawn_actor(obs4_bp, obs4_tf)
        if obs4 is not None:
            obs4.set_autopilot(True)
            print('created %s' % obs4.type_id)

        #Simulation loop
        call_exit = False
        time_init_sim = timer.time()
        while not rospy.is_shutdown():
            # Carla Tick
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    call_exit = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == K_ESCAPE or event.key == K_q:
                        call_exit = True
                        break

            if call_exit:
                break

        rospy.spin()

    finally:
        if display_manager:
            display_manager.destroy()

        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])

        world.apply_settings(original_settings)

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

    return R, P, M, D

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

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)

        run_simulation(args, client)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
