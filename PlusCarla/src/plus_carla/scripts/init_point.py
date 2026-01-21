#!/usr/bin/env python

import glob
import os
import sys

dist_path = os.path.abspath(os.path.join("../../../../", "PythonAPI/carla/dist/"))
try:
    sys.path.insert(0, glob.glob('%s/carla-*%d.%d-%s.egg' % (
        dist_path,
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import utils
import time

sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
sys.path.append('/opt/plusai/lib/python')

client = carla.Client('127.0.0.1', 2000)
client.load_world('gomentum_bunkerD')
client.set_timeout(5)
world = client.get_world()


bp = world.get_blueprint_library().filter('vehicle.mercedes.sprinter')[0]
ego_x, ego_y = utils.plus_xy_to_carla_xy(-7589267.66, 10352928.89)
vehicle_tf = carla.Transform(
    carla.Location(ego_x, ego_y, 2),
    carla.Rotation(yaw=-138))
vehicle = world.spawn_actor(bp, vehicle_tf)

# 2 m above it
spectator_location = vehicle_tf.location + carla.Location(z=10.0)

# rotation: pitch = -90° looks straight down
spectator_rotation = carla.Rotation(pitch=-90, yaw=0, roll=0)

# set transform
spectator = world.get_spectator()
spec_tf = carla.Transform(spectator_location, spectator_rotation)
spectator.set_transform(spec_tf)

# Create an RGB camera blueprint
bp_lib = world.get_blueprint_library()
cam_bp = bp_lib.find('sensor.camera.rgb')
cam_bp.set_attribute('image_size_x', '1280')
cam_bp.set_attribute('image_size_y', '720')
cam_bp.set_attribute('fov', '90')

# Spawn the camera at the spectator's position
camera = world.spawn_actor(cam_bp, spec_tf)

# Define callback to save images
def save_image(image):
    image.save_to_disk('_out/spectator_%06d.png' % image.frame)

camera.listen(save_image)

# Run for 5 seconds
time.sleep(5)

camera.stop()
camera.destroy()
vehicle.destroy()