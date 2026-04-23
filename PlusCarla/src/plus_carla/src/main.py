#!/usr/bin/env python

import glob
import os
import sys

# dist_path = os.path.abspath(os.path.join("../../../../", "PythonAPI/carla/dist/"))
# try:
#     sys.path.append(glob.glob('%s/carla-*%d.%d-%s.whl' % (
#         dist_path,
#         sys.version_info.major,
#         sys.version_info.minor,
#         'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
# except IndexError:
#     pass
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
sys.path.append('/opt/plusai/lib/python')
import argparse
import carla
import cv2
import logging
import math
import numpy as np
import queue
import rospy
import time
from concurrent import futures
from collections import defaultdict
from geometry_msgs.msg import Point32
from perception import obstacle_detection_pb2
from plus_carla.srv import SimConnect, SimConnectResponse
from radar_msgs.msg import RadarTrackArray, RadarTrack
from sensor_msgs.msg import CompressedImage as Image
from sensor_msgs.msg import PointCloud2

import utils
import traceback

vehicle = None
obs_vehicle_list = {}
yaw_offset = 0
x_offset = 0
y_offset = 0

SPAWN_OBS = False
SIM_STARTED = False


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
        if sensor_type == 'RGBCamera':
            self.img_queue = queue.Queue()
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            height = 1920
            width = 1080
            camera_bp.set_attribute('image_size_x', '%s' % height)
            camera_bp.set_attribute('image_size_y', '%s' % width)

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])
            self.sensor_name = camera_bp.get_attribute('role_name').as_str()


            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            calibration = np.identity(3)
            calibration[0, 2] = height / 2.0
            calibration[1, 2] = width / 2.0
            calibration[0, 0] = calibration[1, 1] = height / (2.0 * np.tan(camera_bp.get_attribute('fov').as_float() * np.pi / width))
            camera.calibration = calibration
            camera.listen(self.camera_callback)
            
            return camera

        elif sensor_type == 'LiDAR':
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

    def camera_callback(self, image):
        image.convert(carla.ColorConverter.Raw)

        if SIM_STARTED:
            self.img_queue.put(image)

        import gc
        gc.collect()

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
            # if depth < 1:
            #     continue

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

def wait_for_image(cam):
    try:
        img = cam.img_queue.get(timeout=1)
        cam.capture_image(img)
    except:
        logging.error("Sensor '%s' did not return data in time", cam.sensor_name)

def handle_plus_vehicle_control(req):
    global vehicle, obs_vehicle_list, SPAWN_OBS, yaw_offset, x_offset, y_offset, SIM_STARTED

    if not vehicle:
        rospy.logerr("Vehicle not initialized!")
        return SimConnectResponse(False, Image(), Image(), Image(), Image(), Image(), Image(), Image(), Image(), Image(), None, None, None, None)
    if not SIM_STARTED:
        SIM_STARTED = True

    # Move the vehicle
    ego_lat = req.ego_lat
    ego_lon = req.ego_lon
    ego_x, ego_y = utils.convert_to_carla_coord(ego_lat, ego_lon, x_offset, y_offset)
    ego_vx = req.ego_vx
    ego_vy = req.ego_vy
    ego_yaw = -math.degrees(req.ego_yaw)
    new_transform = carla.Transform(carla.Location(x=ego_x, y=ego_y), carla.Rotation(yaw=ego_yaw))
    vehicle.set_transform(new_transform)
    yaw_offset = -math.degrees(req.yaw_offset)

    ## Assign new obs pose
    try:
        obs_info = req.obstacle_info
        pb = obstacle_detection_pb2.ObstacleDetection()
        pb.ParseFromString(bytes(obs_info))
    except:
        traceback.print_exc()
    obs_for_radar = {}
    for obstacle in pb.obstacle:
        cur_id = obstacle.id
        if cur_id not in obs_vehicle_list.keys():
            continue
        cur_motion = obstacle.motion
        cur_x, cur_y = utils.plus_xy_to_carla_xy(cur_motion.x, cur_motion.y, x_offset, y_offset)
        cur_obs = obs_vehicle_list[cur_id]
        cur_tf = cur_obs.get_transform()
        cur_yaw = -math.degrees(cur_motion.yaw)
        new_rot = carla.Rotation(yaw=cur_yaw + yaw_offset)
        new_loc = carla.Location(x=cur_x, y=cur_y, z=cur_tf.location.z)
        obs_for_radar[cur_obs.id] = [cur_x, cur_y, cur_motion.vx, cur_motion.vy]
        cur_obs.set_transform(carla.Transform(new_loc, new_rot))

    ## Wait till all sensor queues are ready
    with futures.ThreadPoolExecutor() as executor:
        executor.map(wait_for_image, camera_list)

    for radar in radar_list:
        radar.obstacles = obs_for_radar
        radar.v = [ego_vx, ego_vy, 0.]

    return SimConnectResponse(True,
                              front_left_camera.ros_image,
                              front_right_camera.ros_image,
                              front_long_center_camera.ros_image if 'front_long_center_camera' in globals() else Image(),
                              front_center_camera.ros_image,
                              left_front_camera.ros_image,
                              left_side_camera.ros_image,
                              left_rear_camera.ros_image,
                              right_front_camera.ros_image,
                              right_side_camera.ros_image,
                              right_rear_camera.ros_image,
                              left_aeva_lidar.latest_lidar,
                              front_center_radar.radar,
                              left_rear_radar_sr.radar,
                              right_rear_radar_sr.radar,
                              front_center_radar.pc,
                              )

def run_simulation(args, client):
    """This function performed one test run using the args parameters
    and connecting to the carla client passed.
    """
    vehicle_list = []

    global vehicle, obs_vehicle_list
    global camera_list, lidar_list, radar_list
    global x_offset, y_offset
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

        # Getting the world and loading map
        if args.map != "default":
            client.load_world(args.map)
        world = client.get_world()
        x_offset = 0
        y_offset = 0
        if args.custom_xodr:
            utils.get_xodr_offset(world)
            x_offset, y_offset = utils.get_xodr_offset(world)
        else:
            # Setting geo proj str
            utils.set_geo_ref(args.proj_str)

        original_settings = world.get_settings()
        sp = utils.ScenarioProcessor(args.scenario_file, x_offset, y_offset)

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
        init_x, init_y = sp.get_ego()
        transform = carla.Transform(
            carla.Location(init_x, init_y, 3),
            carla.Rotation(yaw=args.init_yaw))
        spectator.set_transform(transform)
        vehicle = world.spawn_actor(bp, transform)
        vehicle_list.append(vehicle)
        print(transform)

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
                camera_list.append(sensor)
            elif s['type'] == 'radar':
                radar_list.append(sensor)
            else:
                lidar_list.append(sensor)

            globals()[s['name']] = sensor

        # Spawning obstacles
        obs_map = sp.obs_map
        tm = client.get_trafficmanager()

        for obs_id, state in obs_map.items():
            obsyaw = args.init_yaw
            if args.opposite_lane_obs_id != -1 and obs_id > args.opposite_lane_obs_id:
                obsyaw = args.init_yaw + 180
            obs_tf = carla.Transform(
                carla.Location(state[0], state[1], 2),
                carla.Rotation(yaw=obsyaw))
            obs_vehicle = utils.get_blueprint_from_category(state[4])
            obs_bp = world.get_blueprint_library().filter(obs_vehicle)[0]
            obs = world.try_spawn_actor(obs_bp, obs_tf)
            if obs is not None:
                logging.error('created %s with id = %s, carla_id = %s, target speed %s', obs.type_id, obs_id, obs.id, state[3])
                obs.set_autopilot(False)
                obs_vehicle_list[obs_id] = obs

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
                    left_latest = left_aeva_lidar.lidar_queue.get(True, 1.0)
                    right_latest_b = utils.transform_between_frames(right_aeva_lidar.lidar_queue.get(True, 1.0), right_aeva_lidar.M, left_aeva_lidar.M)
                    left_os0_b = utils.transform_between_frames(left_OS0_lidar.lidar_queue.get(True, 1.0), left_OS0_lidar.M, left_aeva_lidar.M, offset_y=2.462, offset_z=0.29)
                    right_os0_b = utils.transform_between_frames(right_OS0_lidar.lidar_queue.get(True, 1.0), right_OS0_lidar.M, left_aeva_lidar.M, offset_y=-2.462, offset_z=0.29)
                    combined_points = np.vstack((left_latest, right_latest_b, left_os0_b, right_os0_b))

                    left_aeva_lidar.latest_lidar = utils.form_lidar_msg(combined_points, "Lidar")
                    time.sleep(0.005)  # This can fix Open3D jittering issues.
            except queue.Empty:
                logging.error("Timed out waiting for LiDAR")

        rospy.spin()

    finally:
        [sensor.destroy() for sensor in camera_list + lidar_list + radar_list]
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])
        client.apply_batch([carla.command.DestroyActor(x) for x in obs_vehicle_list.keys()])

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
        '--custom-xodr',
        action='store_true',
        help='Whether to use custom XODR file for map, which should be placed in CARLA server and named as "xxx.xodr"')
    argparser.set_defaults(custom_xodr=False)

    args = argparser.parse_args()
    main()
