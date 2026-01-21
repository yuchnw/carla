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
import cv2
import math
from pyproj import Proj
import pyproj_eqdc

os.environ["SDL_AUDIODRIVER"] = "dummy"

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import argparse
import random
import time
import numpy as np


try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

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
        pygame.init()
        pygame.font.init()
        # self.display = pygame.display.set_mode(window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.display=None

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

        self.time_processing = 0.0
        self.tics_processing = 0

        self.display_man.add_sensor(self)

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == 'RGBCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            disp_size = self.display_man.get_display_size()
            camera_bp.set_attribute('image_size_x', '3840')
            camera_bp.set_attribute('image_size_y', '2160')

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            self.label = camera_bp.get_attribute('role_name').as_str()

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image)

            return camera

        elif sensor_type == 'SegCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
            camera_bp.set_attribute('image_size_x', '3840')
            camera_bp.set_attribute('image_size_y', '2160')

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            self.label = camera_bp.get_attribute('role_name').as_str()

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_seg_image)

            return camera

        elif sensor_type == 'DepthCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.depth')
            camera_bp.set_attribute('image_size_x', '3840')
            camera_bp.set_attribute('image_size_y', '2160')

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            self.label = camera_bp.get_attribute('role_name').as_str()

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_dep_image)

            return camera

        elif sensor_type == 'LiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('range', '100')
            # lidar_bp.set_attribute('dropoff_general_rate', lidar_bp.get_attribute('dropoff_general_rate').recommended_values[0])
            # lidar_bp.set_attribute('dropoff_intensity_limit', lidar_bp.get_attribute('dropoff_intensity_limit').recommended_values[0])
            # lidar_bp.set_attribute('dropoff_zero_intensity', lidar_bp.get_attribute('dropoff_zero_intensity').recommended_values[0])

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.save_lidar_image)

            return lidar
        
        elif sensor_type == 'SemanticLiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
            lidar_bp.set_attribute('range', '100')

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.save_semanticlidar_image)

            return lidar
        
        elif sensor_type == "Radar":
            radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
            for key in sensor_options:
                radar_bp.set_attribute(key, sensor_options[key])

            radar = self.world.spawn_actor(radar_bp, transform, attach_to=attached)
            radar.listen(self.save_radar_image)

            return radar
        
        else:
            return None

    def get_sensor(self):
        return self.sensor

    def save_rgb_image(self, image):
        t_start = self.timer.time()

        # image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        # array = array[:, :, ::-1]

        cv2.imwrite('/home/yuchen.wang/Documents/_out/%s/%06d.jpg' % (self.label, image.frame), array, [int(cv2.IMWRITE_JPEG_QUALITY), 95])

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        del array
        import gc
        gc.collect()

    def save_seg_image(self, image):
        image.save_to_disk('/home/yuchen.wang/Documents/_out/%s/%06d.jpg' % (self.label, image.frame), carla.ColorConverter.CityScapesPalette)

    def save_dep_image(self, image):
        image.save_to_disk('/home/yuchen.wang/Documents/_out/%s/%06d.jpg' % (self.label, image.frame), carla.ColorConverter.Depth)

    def save_lidar_image(self, image):
        t_start = self.timer.time()

        disp_size = self.display_man.get_display_size()
        lidar_range = 2.0*float(self.sensor_options['range'])

        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(lidar_img)

        # cv2.imwrite('_out/%06d.jpg' % image.frame, lidar_img, [int(cv2.IMWRITE_JPEG_QUALITY), 95])

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def save_semanticlidar_image(self, image):
        t_start = self.timer.time()

        disp_size = self.display_man.get_display_size()
        lidar_range = 2.0*float(self.sensor_options['range'])

        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 6), 6))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def save_radar_image(self, radar_data):
        t_start = self.timer.time()
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (len(radar_data), 4))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def render(self):
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)

    def destroy(self):
        self.sensor.destroy()

def sanitize_text(text, encoding='utf-8', errors='replace'):
    if isinstance(text, bytes):
        return text.decode(encoding, errors=errors)
    else:
        return str(text)

def convert_to_carla_coord(lat, lon):
    proj_str = '+proj=tmerc +lat_0=29.81145 +lon_0=-98.0087 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs'
    proj = Proj(proj_str)
    x, y = proj(lon, lat)
    return x,-y

class ScenarioProcessor:
    def __init__(self, scenario_file):
        import simulation_data_pb2
        import google.protobuf.text_format as protobuf_text_format
        self.scenario = simulation_data_pb2.SimulationConfig()
        with open(scenario_file, 'r') as s_file:
            content = sanitize_text(s_file.read())
            protobuf_text_format.Merge(content, self.scenario)
        self.obs_map = {}
        self.get_obs()

    def get_obs(self):
        for v in self.scenario.scenario_data.vehicles:
            if v.is_ego:
                continue
            EQDC_CONVERTER = pyproj_eqdc.EqdcConverter()
            EQDC_CONVERTER.configure('NA', 'WGS84')
            lat, lon = EQDC_CONVERTER.to_latlon(v.state.x, v.state.y)
            obs_x, obs_y = convert_to_carla_coord(lat, lon)
            self.obs_map[v.id] = [obs_x, obs_y, -math.degrees(v.state.yaw), v.state.v]

        # self.opt_obs = {50: [-2054, 2819, 123, 20],
        #                 51: [-2060, 2837, 123, 18],
        #                 52: [-2040, 2805, 123, 18],
        #                 53: [-2046, 2807, 123, 20],
        #                 54: [-2037, 2788, 123, 21],
        #                 55: [-2019, 2773, 123, 15],
        #                 56: [-1995, 2733, 123, 20],
        #                 57: [-1975, 2708, 123, 20],}

        # for i, s in self.opt_obs.items():
        #     self.obs_map[i] = [s[0], s[1], s[2], s[3] * 3.6]
        print(self.obs_map)

def run_simulation(args, client):
    """This function performed one test run using the args parameters
    and connecting to the carla client passed.
    """

    display_manager = None
    vehicle = None
    vehicle_list = []
    timer = CustomTimer()

    try:

        # Getting the world and
        world = client.get_world()
        original_settings = world.get_settings()
        world.set_weather(carla.WeatherParameters.ClearSunset)
        # weather = carla.WeatherParameters(
        #     cloudiness=80.0,
        #     precipitation=10.0,
        #     sun_altitude_angle=-1.0)

        # world.set_weather(weather)

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
        # tf = carla.Transform(carla.Location(-2083.1795672688713, 2909.9611540967576, 3), carla.Rotation(yaw=-57))
        tf = carla.Transform(carla.Location(2910.5602716702924, -2497.4140619038058, 3), carla.Rotation(yaw=-225))
        vehicle = world.try_spawn_actor(bp, tf)
        spawn_point = tf
        # spawn_point = random.choice(world.get_map().get_spawn_points())
        # vehicle = world.spawn_actor(bp, spawn_point)
        # tf = vehicle.get_transform()
        spectator.set_transform(tf)
        vehicle_list.append(vehicle)
        vehicle.set_autopilot(True)

        # tm = client.get_trafficmanager()
        # tm.set_desired_speed(vehicle, 60)

        # Display Manager organize all the sensors an its display in a window
        # If can easily configure the grid and the total window size
        display_manager = DisplayManager(grid_size=[1,1], window_size=[args.width, args.height])

        # IMU
        imu_bp = world.get_blueprint_library().find('sensor.other.imu')
        imu = world.spawn_actor(imu_bp, carla.Transform(carla.Location(x=0.497595, z=1.288), carla.Rotation(yaw=+00)), attach_to=vehicle)

        # # Front Wide Camera
        # s_fc = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=2.46, y=0, z=1.12), carla.Rotation(yaw=+00)), 
        #                      imu, {'fov': '120', 'role_name': 'front_wide_camera'}, display_pos=[0, 1])
        
        # Front Left Camera
        s_fl = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=2.365, y=-0.77, z=1.12), carla.Rotation(yaw=+00)), 
                             imu, {'fov': '47', 'role_name': 'front_left_camera'}, display_pos=None)

        seg_fl = SensorManager(world, display_manager, 'SegCamera', carla.Transform(carla.Location(x=2.365, y=-0.77, z=1.12), carla.Rotation(yaw=+00)), 
                             imu, {'fov': '47', 'role_name': 'seg_camera'}, display_pos=None)
        d_fl = SensorManager(world, display_manager, 'DepthCamera', carla.Transform(carla.Location(x=2.365, y=-0.77, z=1.12), carla.Rotation(yaw=+00)), 
                             imu, {'fov': '47', 'role_name': 'depth_camera'}, display_pos=None)
        
        # # Front Right Camera
        # s_fr = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=2.45, y=0.318, z=1.12), carla.Rotation(yaw=+00)), 
        #                      imu, {'fov': '47', 'role_name': 'front_right_camera'}, display_pos=None)
        
        # # Left Front Camera
        # s_lf = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=-0.077, y=-1.265, z=1.487), carla.Rotation(roll=-90, pitch=-10, yaw=-33)), 
        #                      imu, {'fov': '120', 'role_name': 'left_front_camera'}, display_pos=None)
        
        # # Left Side Camera
        # s_ls = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=-0.154, y=-1.293, z=1.487), carla.Rotation(roll=-90, pitch=-32, yaw=-93)), 
        #                      imu, {'fov': '120', 'role_name': 'left_side_camera'}, display_pos=None)
        
        # # Left Rear Camera
        # s_lr = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=-0.232, y=-1.283, z=1.487), carla.Rotation(roll=-90, pitch=-4, yaw=-153)), 
        #                      imu, {'fov': '120', 'role_name': 'left_rear_camera'}, display_pos=None)
        
        # # Right Front Camera
        # s_rf = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=-0.077, y=1.265, z=1.487), carla.Rotation(roll=-90, pitch=-10, yaw=33)), 
        #                      imu, {'fov': '120', 'role_name': 'right_front_camera'}, display_pos=None)
        
        # # Right Side Camera
        # s_rs = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=-0.154, y=1.293, z=1.487), carla.Rotation(roll=-90, pitch=-32, yaw=93)), 
        #                      imu, {'fov': '120', 'role_name': 'right_side_camera'}, display_pos=None)
        
        # # Right Rear Camera
        # s_rr = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=-0.232, y=1.283, z=1.487), carla.Rotation(roll=-90, pitch=-4, yaw=153)), 
        #                      imu, {'fov': '120', 'role_name': 'right_rear_camera'}, display_pos=None)

        # scenario_file = "/home/yuchen.wang/Downloads/carla/02_ego_2nd_lane.prototxt"
        # sp = ScenarioProcessor(scenario_file)
        # obs_map = sp.obs_map
        # obs_vehicle_list = []

        # for id, state in obs_map.items():
        #     obs_tf = carla.Transform(
        #         carla.Location(state[0], state[1], 3),
        #         carla.Rotation(yaw=state[2]))
        #     obs_bp = random.choice(world.get_blueprint_library().filter('vehicle.*.*'))
        #     obs = world.try_spawn_actor(obs_bp, obs_tf)
        #     tm.set_desired_speed(obs, state[3] * 3.6)
        #     obs.set_autopilot(True)
        #     if obs is not None:
        #         print('created {} with target speed {}'.format(obs.type_id, state[3]))
        #         obs_vehicle_list.append(obs)

        # Calculate new transform 5 meters in front
        d = 20
        yaw = math.radians(spawn_point.rotation.yaw)
        dx = d * math.cos(yaw)
        dy = d * math.sin(yaw)
        new_location = carla.Location(
            x=spawn_point.location.x + dx,
            y=spawn_point.location.y + dy,
            z=spawn_point.location.z
        )
        obstf = carla.Transform(new_location, spawn_point.rotation)
        obs_bp = random.choice(world.get_blueprint_library().filter('vehicle.dodge.*'))

        # Spawn vehicle
        obs = world.spawn_actor(obs_bp, obstf)
        obs.set_autopilot(True)

        #Simulation loop
        call_exit = False
        time_init_sim = timer.time()
        while True:
            # Carla Tick
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()

            # Render received data
            display_manager.render()

            # for event in pygame.event.get():
            #     if event.type == pygame.QUIT:
            #         call_exit = True
            #     elif event.type == pygame.KEYDOWN:
            #         if event.key == K_ESCAPE or event.key == K_q:
            #             call_exit = True
            #             break

            if call_exit:
                break

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
        default='3840x2160',
        help='window resolution (default: 1280x720)')

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