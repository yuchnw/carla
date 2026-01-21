#!/usr/bin/env python3
"""
Copyright Aeva 2024

Open3D FMCW LiDAR visualization example for CARLA

Example usage:
python open3d_fmcw_lidar.py --motion-compensate --channel-color velocity
"""

import argparse
import glob
import os
import random
import sys
import time
import yaml
from queue import Queue
from queue import Empty

import cv2
import numpy as np
import open3d as o3d
from matplotlib import cm

try:
    sys.path.append(
        glob.glob('../carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


def main(args):
    """Main function of the script"""
    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    world = client.get_world()

    bp = world.get_blueprint_library().find('sensor.lidar.fmcw')
    pattern_yaml_path = "/opt/plusai/carla/ScanPatterns.yaml"

    bp.set_attribute('pattern_file', pattern_yaml_path)
    bp.set_attribute('pattern_name', '64-19.2-Uniform')
    bp.set_attribute('motion_compensate', 'true')
    transform = carla.Transform(carla.Location(x=0, y=0, z=2))
    sensor = world.spawn_actor(bp, transform)


if __name__ == "__main__":
    executable_dir = os.path.dirname(os.path.abspath(__file__))
    pattern_yaml_path = os.path.abspath(
        os.path.join(executable_dir, "../../ScanPatterns.yaml"))

    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host CARLA Simulator (default: localhost)')
    argparser.add_argument('-p',
                           '--port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port of CARLA Simulator (default: 2000)')
    argparser.add_argument('--pattern-file',
                           default=str(pattern_yaml_path),
                           type=str,
                           help='Absolute path to scan pattern yaml file')
    argparser.add_argument('--pattern-name',
                           default='64-19.2-Uniform',
                           type=str,
                           help='Name of scan pattern')
    
    args = argparser.parse_args()

    try:
        main(args)
    except KeyboardInterrupt:
        print('\nExited by user')
