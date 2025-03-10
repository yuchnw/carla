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

        self.time_processing = 0.0
        self.tics_processing = 0

        self.display_man.add_sensor(self)

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == 'RGBCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            disp_size = self.display_man.get_display_size()
            camera_bp.set_attribute('image_size_x', str(disp_size[0]))
            camera_bp.set_attribute('image_size_y', str(disp_size[1]))

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image)

            return camera
        
        else:
            return None

    def get_sensor(self):
        return self.sensor

    def save_rgb_image(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def render(self):
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)

    def destroy(self):
        self.sensor.destroy()

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

        if args.sync:
            traffic_manager = client.get_trafficmanager(8000)
            settings = world.get_settings()
            traffic_manager.set_synchronous_mode(True)
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)


        # Instanciating the vehicle to which we attached the sensors
        bp = world.get_blueprint_library().filter('european_hgv')[0]
        transform = carla.Transform(
            carla.Location(-1790.853, 2528.455, 3), #I35-New lat_lon(29.7886388, -98.0272216)
            carla.Rotation())
        vehicle = world.spawn_actor(bp, transform)
        vehicle_list.append(vehicle)
        vehicle.set_autopilot(True)

        # Display Manager organize all the sensors an its display in a window
        # If can easily configure the grid and the total window size
        display_manager = DisplayManager(grid_size=[3, 3], window_size=[args.width, args.height])

        # IMU
        imu_bp = world.get_blueprint_library().find('sensor.other.imu')
        imu = world.spawn_actor(imu_bp, carla.Transform(carla.Location(x=1.322855, z=1.303), carla.Rotation(yaw=+00)), attach_to=vehicle)

        # Front Wide Camera
        s_fc = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=2.645, y=0.0548, z=1.205), carla.Rotation(yaw=+00)), 
                        imu, {'fov': '121'}, display_pos=[0, 1])
        
        # Left Front Camera
        s_lf = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0.304, y=-1.445544, z=1.6864), carla.Rotation(roll=-6.43, pitch=-10, yaw=-33)), 
                        imu, {'fov': '121'}, display_pos=[1, 2])
        
        # Left Side Camera
        s_ls = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0.2602, y=-1.445544, z=1.6835), carla.Rotation(roll=-1.59, pitch=-32, yaw=-93)), 
                        imu, {'fov': '121'}, display_pos=[1, 1])
        
        # Left Rear Camera
        s_lr = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0.1777, y=-1.445544, z=1.678), carla.Rotation(roll=2.04, pitch=-4, yaw=-153)), 
                        imu, {'fov': '121'}, display_pos=[1, 0])
        
        # Right Front Camera
        s_rf = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0.304, y=1.445544, z=1.6864), carla.Rotation(roll=-6.43, pitch=-10, yaw=33)), 
                        imu, {'fov': '121'}, display_pos=[2, 2])
        
        # Right Side Camera
        s_rs = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0.2602, y=1.445544, z=1.6835), carla.Rotation(roll=-1.59, pitch=-32, yaw=93)), 
                        imu, {'fov': '121'}, display_pos=[2, 1])
        
        # Right Rear Camera
        s_rr = SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0.1777, y=1.445544, z=1.678), carla.Rotation(roll=2.04, pitch=-4, yaw=153)), 
                        imu, {'fov': '121'}, display_pos=[2, 0])


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

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    call_exit = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == K_ESCAPE or event.key == K_q:
                        call_exit = True
                        break

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
        default='1920x1080',
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
