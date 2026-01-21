import carla
import math
import numpy as np
import queue
import utils
from collections import defaultdict

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
            height = 3840
            width = 2160
            camera_bp.set_attribute('image_size_x', '%s' % height)
            camera_bp.set_attribute('image_size_y', '%s' % width)
            # camera_bp.set_attribute('sensor_tick', '0.1')

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])
            self.sensor_name = camera_bp.get_attribute('role_name').as_str()

            # output_path = '/home/yuchen.wang/Documents/_out/%s' % (self.sensor_name)
            # print(output_path)
            # if not os.path.exists(output_path):
            #     print("create")
            #     os.mkdir(output_path)
            # else:
            #     files = glob.glob(output_path+'/*')
            #     for f in files:
            #         os.remove(f)

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            calibration = np.identity(3)
            calibration[0, 2] = height / 2.0
            calibration[1, 2] = width / 2.0
            calibration[0, 0] = calibration[1, 1] = height / (2.0 * np.tan(camera_bp.get_attribute('fov').as_float() * np.pi / width))
            camera.calibration = calibration
            camera.listen(self.camera_callback)

            # # spawn seg cam
            # segcamera_bp = self.world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
            # segcamera_bp.set_attribute('image_size_x', '1920')
            # segcamera_bp.set_attribute('image_size_y', '1080')

            # segcamera = self.world.spawn_actor(segcamera_bp, transform, attach_to=attached)
            # print("spawned seg cam")
            # segcamera.listen(self.save_seg_image)

            # # spawn depth cam
            # depcamera_bp = self.world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
            # depcamera_bp.set_attribute('image_size_x', '1920')
            # depcamera_bp.set_attribute('image_size_y', '1080')

            # depcamera = self.world.spawn_actor(depcamera_bp, transform, attach_to=attached)
            # depcamera.listen(self.save_seg_image)


            return camera

        elif sensor_type == 'SegCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
            camera_bp.set_attribute('image_size_x', '3840')
            camera_bp.set_attribute('image_size_y', '2160')

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            self.sensor_name = camera_bp.get_attribute('role_name').as_str()

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_seg_image)

            return camera

        elif sensor_type == 'DepthCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.depth')
            camera_bp.set_attribute('image_size_x', '3840')
            camera_bp.set_attribute('image_size_y', '2160')

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            self.sensor_name = camera_bp.get_attribute('role_name').as_str()

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_dep_image)

            return camera

        elif sensor_type == 'LiDAR':
            self.lidar_queue = queue.Queue()
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
            self.M = utils.get_transformation_matrix(transform)

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
            self.M = utils.get_transformation_matrix(transform)
            executable_dir = os.path.dirname(os.path.abspath(__file__))
            pattern_yaml_path = os.path.abspath(
                os.path.join(executable_dir, "../../../../ScanPatterns.yaml"))
            # pattern_yaml_path = "/home/yuchen.wang/workspace/carla/ScanPatterns.yaml"

            lidar_bp.set_attribute('pattern_file', pattern_yaml_path)
            lidar_bp.set_attribute('pattern_name', '64-19.2-Uniform')
            lidar_bp.set_attribute('motion_compensate', 'true')

            raycast_modes = {'frame': '0', 'line': '1', 'point': '2'}
            lidar_bp.set_attribute('raycast_mode', raycast_modes['frame'])

            # if args.no_noise:
            # lidar_bp.set_attribute('dropoff_general_rate', '0.0')
            # lidar_bp.set_attribute('dropoff_intensity_limit', '1.0')
            # lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')
            # else:
            lidar_bp.set_attribute('noise_stddev', '0.05')
            lidar_bp.set_attribute('dropoff_general_rate', '0.3')

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            try:
                lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)
                lidar.listen(lambda data: self.fmcw_callback(data, self.lidar_queue))

                return lidar
            except Exception as e:
                print("Error spawing FMCW lidar: {}".format(e))

        elif sensor_type == "Radar":
            radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
            self.z_offset = transform.location.z
            self.ground_removal_threshold = 0.1 if self.z_offset < -0.5 else 0.2
            for key in sensor_options:
                radar_bp.set_attribute(key, sensor_options[key])
            self.sensor_name = radar_bp.get_attribute('role_name').as_str()
            self.obstacles = {}
            self.v = [0., 0., 0.]

            radar = self.world.spawn_actor(radar_bp, transform, attach_to=attached)
            radar.listen(self.radar_callback)

            return radar

        else:
            return None

    def save_seg_image(self, image):
        image.save_to_disk('/home/yuchen.wang/Documents/_out/%s/seg%06d.jpg' % (self.sensor_name, image.frame), carla.ColorConverter.CityScapesPalette)

    def save_dep_image(self, image):
        image.save_to_disk('/home/yuchen.wang/Documents/_out/%s/dep%06d.jpg' % (self.sensor_name, image.frame), carla.ColorConverter.Depth)
    
    def capture_image(self, sensor_img):
        """Captures an image from a CARLA camera sensor and converts it to ROS CompressedImage format."""
        image = np.array(sensor_img.raw_data).reshape((sensor_img.height, sensor_img.width, 4))
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        # Resize the image to 1920x1080
        resized_image = cv2.resize(image, (1920, 1080), interpolation=cv2.INTER_AREA)
        _, jpeg = cv2.imencode('.jpg', resized_image)

        # cv2.imwrite('/home/yuchen.wang/Documents/_out/%s/frame%06d.jpg' % (self.sensor_name, sensor_img.frame), array, [int(cv2.IMWRITE_JPEG_QUALITY), 95])


        self.ros_image = Image()
        self.ros_image.header.stamp = rospy.Time.now()
        self.ros_image.format = "jpeg"
        self.ros_image.data = np.array(jpeg).tobytes()

    def camera_callback(self, image):
        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        # array = array[:, :, ::-1]

        if SIM_STARTED:
            self.img_queue.put(image)
            # cv2.imwrite('/home/yuchen.wang/Documents/_out/%s/frame%06d.jpg' % (self.sensor_name, image.frame), array, [int(cv2.IMWRITE_JPEG_QUALITY), 95])

        del array
        import gc
        gc.collect()

    def lidar_callback(self, data, lidar_queue):
        points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))

        self.pc = points.copy()
        self.pc[:,1] = -self.pc[:,1]
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
        # print(point_cloud['range'])
        intensity = point_cloud['intensity'].reshape(-1, 1)  # shape: (N, 1)

        points = np.hstack((points, intensity))  # shape: (N, 4)
        if SIM_STARTED:
            lidar_queue.put(points)

    # def generate_pseudo_id(self, detection):
    #     # Hash-based ID using depth, azimuth, altitude
    #     key = f"{round(detection.depth, 2)}_{round(detection.azimuth, 2)}_{round(detection.altitude, 2)}"
    #     return hash(key) % 65536

    def radar_callback(self, data):
        self.radar_tracks.clear()
        self.radar = RadarTrackArray()
        self.radar.header = rospy.Header()
        self.radar.header.frame_id = "radar"
        self.cur_tf = self.sensor.get_transform()

        for detection in data:
            # id = self.generate_pseudo_id(detection)
            # print(id)
            id = detection.actor_id
            if id not in self.obstacles.keys():
                continue

            # Convert from spherical to cartesian
            depth = detection.depth
            azimuth = detection.azimuth
            altitude = detection.altitude
            velocity = detection.velocity

            obs = self.obstacles[id]
            obs_pos = np.array([obs[0], obs[1], 0])
            obs_v = np.array([obs[2], obs[3], 0])
            radar_pos = np.array([self.cur_tf.location.x, self.cur_tf.location.y, self.cur_tf.location.z])
            radar_v = np.array([self.v[0], self.v[1], self.v[2]])
            velocity = utils.calculate_radar_speed(obs_pos, obs_v, radar_pos, radar_v)

            x = depth * math.cos(azimuth) * math.cos(-altitude)
            y = depth * math.sin(-azimuth) * math.cos(altitude)
            z = depth * math.sin(altitude)

            vx = velocity * math.cos(azimuth) * math.cos(-altitude)
            vy = velocity * math.sin(-azimuth) * math.cos(altitude)

            if abs(1.288+self.z_offset+z) < self.ground_removal_threshold:
                continue
            if depth < 1:
                continue

            self.radar_tracks[id].append({'x': x, 'y': y, 'z': z, 'vx': vx, 'vy': vy})

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

    def render(self, bounding_boxes):
        if self.surface is not None:
            for bbox in bounding_boxes:
                points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]
                # draw lines
                # base
                pygame.draw.line(self.surface, BB_COLOR, points[0], points[1])
                pygame.draw.line(self.surface, BB_COLOR, points[0], points[1])
                pygame.draw.line(self.surface, BB_COLOR, points[1], points[2])
                pygame.draw.line(self.surface, BB_COLOR, points[2], points[3])
                pygame.draw.line(self.surface, BB_COLOR, points[3], points[0])
                # top
                pygame.draw.line(self.surface, BB_COLOR, points[4], points[5])
                pygame.draw.line(self.surface, BB_COLOR, points[5], points[6])
                pygame.draw.line(self.surface, BB_COLOR, points[6], points[7])
                pygame.draw.line(self.surface, BB_COLOR, points[7], points[4])
                # base-top
                pygame.draw.line(self.surface, BB_COLOR, points[0], points[4])
                pygame.draw.line(self.surface, BB_COLOR, points[1], points[5])
                pygame.draw.line(self.surface, BB_COLOR, points[2], points[6])
                pygame.draw.line(self.surface, BB_COLOR, points[3], points[7])

    def destroy(self):
        self.sensor.destroy()