from blueprint_library import BlueprintLibrary
from nurec_integration import NurecRenderer, dict_to_camera_spec
from nurec_render_service import NuRecRenderService
from projection_functions import get_t_rig_enu_from_ecef
from scenario import Scenario

from nre.grpc.protos.sensorsim_pb2_grpc import SensorsimServiceStub
from nre.grpc.protos.sensorsim_pb2 import (
    RGBRenderReturn,
    RGBRenderRequest,
    # AvailableCamerasRequest,
    # AvailableCamerasReturn,
    CameraSpec,
    PosePair,
    DynamicObject,
    ImageFormat,
    # OpenCVPinholeCameraParam,
    # FthetaCameraParam,
    # OpenCVFisheyeCameraParam,
    ShutterType,
    LinearCde,
)
from utils import (
    se3_to_grpc_pose,
    # actor_to_grpc_pose,
    # mat_to_carla_transform,
    undo_carla_coordinate_transform,
    make_transform_matrix
)

import grpc
from concurrent import futures
from nre.grpc.protos import image_stream_pb2
from nre.grpc.protos import image_stream_pb2_grpc

import argparse
import cv2
import imageio
import logging
import numpy as np
import os
import queue
import threading
import time
import traceback
from typing import Dict, List, Any, Optional, Set, Callable, Union, Tuple
import yaml
import zipfile

logger = logging.getLogger(__name__)


class EgoPosition:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

def get_camera_tf(
    # camera_spec: Union[Dict[str, Any], str],
    # transform: Optional[np.ndarray] = None,
    translation: np.ndarray = np.zeros(3),
):
    """
    Calculate camera transformation matrix.

    Args:
        camera_spec: Either a string (logical camera name from reconstruction) or 
                    dictionary of camera parameters (see dict_to_camera_spec for details)
        transform: 4x4 transformation matrix relative to ego vehicle coordinate frame.
                    Required if camera_spec is a dictionary, optional if using existing camera name
        translation: Additional translation offset [x, y, z] in meters (default: [0, 0, 0])

    Raises:
        ValueError: If transform is required but not provided
        Exception: If camera name is not found in available cameras
    """

    transform = np.array([[0, 0, 1, 2.365],
                          [-1, 0, 0, -0.77],
                          [0, -1, 0, 2.408],
                          [0, 0, 0, 1]])

    t_translation = np.eye(4)
    t_translation[:3, 3] = translation
    transform = t_translation @ transform

    return transform

def transform_to_matrix(x, y, z, roll, pitch, yaw):
    pitch = np.radians(pitch)
    yaw   = np.radians(yaw)
    roll  = np.radians(roll)

    cy = np.cos(yaw)
    sy = np.sin(yaw)

    cr = np.cos(roll)
    sr = np.sin(roll)

    cp = np.cos(pitch)
    sp = np.sin(pitch)

    T = np.array([
        [cp * cy,  cy * sp * sr - sy * cr,  -cy * sp * cr - sy * sr,  x],
        [cp * sy,  sy * sp * sr + cy * cr,  -sy * sp * cr + cy * sr,  y],
        [sp,      -cp * sr,                 cp * cr,                  z],
        [0.0,      0.0,                     0.0,                    1.0]
    ], dtype=np.float32)

    return T

def get_camera_spec(param) -> CameraSpec:
    if "camera_params" in param:
        cam_cfg = param["camera_params"]
        # Convert "pi" string to np.pi
        if cam_cfg.get("max_angle") == "pi":
            cam_cfg["max_angle"] = np.pi

        # Convert shutter_type string to enum
        if isinstance(cam_cfg.get("shutter_type"), str):
            cam_cfg["shutter_type"] = getattr(ShutterType, cam_cfg["shutter_type"])

        # Convert shutter_type string to enum
        if isinstance(cam_cfg.get("linear_cde"), list):
            linear_cde = LinearCde()
            linear_cde.linear_c = cam_cfg["linear_cde"][0]
            linear_cde.linear_d = cam_cfg["linear_cde"][1]
            linear_cde.linear_e = cam_cfg["linear_cde"][2]
            cam_cfg["linear_cde"] = linear_cde

        rotation = param.get("rotation")         # [roll, pitch, yaw] in rad
        translation = param.get("translation")   # [x, y, z]
        transform_matrix = make_transform_matrix(rotation, translation)

        camera_spec = dict_to_camera_spec(cam_cfg)

    return camera_spec, transform_matrix

def generate_request(
    scene_id: str,
    camera_spec: CameraSpec,
    camera_pose: np.ndarray,
    timestamp: int,
    scale: float,
    actors: [],
    format: ImageFormat,
    active_actors: Dict[int, str],
    controllable_tracks: Set[str],
    t_carla_nurec: np.ndarray,
    blueprint_library: Optional[BlueprintLibrary] = None,
    actor_blueprints: Optional[Dict[int, str]] = None,
) -> RGBRenderRequest:
    """
    Generate a gRPC render request for NUREC rendering service.
    
    Args:
        scene_id: Identifier for the NUREC scene
        camera_spec: Camera specification including intrinsics
        camera_pose: 4x4 camera pose matrix
        timestamp: Timestamp in microseconds
        scale: Resolution scaling factor
        actors: list of CARLA actors sent from PlusNurecClient
        format: Image format for rendering
        active_actors: Mapping from actor IDs to track IDs
        controllable_tracks: Set of track IDs that can be controlled
        t_carla_nurec: Transformation matrix from CARLA to NUREC coordinates
        blueprint_library: Optional blueprint library for offset calculations
        actor_blueprints: Optional mapping from actor IDs to blueprint IDs
        
    Returns:
        RGBRenderRequest: gRPC request object for rendering
    """
    camera_pose = t_carla_nurec @ camera_pose
    dynamic_objects = []
    # select all actors that have attributes and have track_id attribute
    for actor in actors:
        if actor.id in active_actors:
            track_id = active_actors[actor.id]
            if not track_id in controllable_tracks:
                continue
            # pose = actor_to_grpc_pose(
            #     actor, t_carla_nurec, blueprint_library, actor_blueprints
            # )
            ## Skip obstacles for now
            # dynamic_objects.append(
            #     DynamicObject(
            #         track_id=track_id,
            #         pose_pair=PosePair(
            #             start_pose=pose,
            #             end_pose=pose,
            #         ),
            #     )
            # )
    return RGBRenderRequest(
        scene_id=scene_id,
        resolution_h=int(camera_spec.resolution_h * scale),
        resolution_w=int(camera_spec.resolution_w * scale),
        camera_intrinsics=camera_spec,
        frame_start_us=timestamp,
        frame_end_us=timestamp + 1,  # important that these are not identical
        sensor_pose=PosePair(
            start_pose=se3_to_grpc_pose(camera_pose),
            end_pose=se3_to_grpc_pose(camera_pose),
        ),
        dynamic_objects=dynamic_objects,
        image_format=format,
        image_quality=95,
    )

class PlusNurecServer(NuRecRenderService):
    def __init__(
        self,
        usdz_path: str,
        port: int = 2000,
        fps: int = 10,
        image=None,
        reuse_container: bool = True,
        image_queue=None,
    ):
        NuRecRenderService.__init__(self, usdz_path, port, image, reuse_container)
        self.usdz_path = usdz_path
        self.port = port
        self.fps = fps
        self.blueprint_library = BlueprintLibrary()
        self.actor_blueprints = {}
        self.active_actors = []
        self.IMG_COUNTER = 0
        self.camera_config_file = "plus_nurec_camera_config.yaml"
        self.cameras = {}
        self.image_queue = image_queue

    def add_ego(self):
        if self.scenario is None:
            raise RuntimeError("Scenario not initialized. Call __enter__ first.")

    def _read_xodr_from_nurec(self, nurec_file: str) -> str:
        try:
            with zipfile.ZipFile(nurec_file, "r") as zip_ref:
                # Check if map.xodr exists in the zip file
                if "map.xodr" not in zip_ref.namelist():
                    available_files = zip_ref.namelist()
                    raise KeyError(
                        f"map.xodr not found in {nurec_file}. Available files: {available_files}"
                    )

                # Read the map.xodr file content
                with zip_ref.open("map.xodr") as xodr_file:
                    data = xodr_file.read().decode("utf-8")

                filename = os.path.basename(nurec_file)
                logger.debug(f"Successfully loaded map.xodr from {filename}")
                return data
        except Exception as e:
            logger.error(f"Error reading XODR from NUREC file {nurec_file}: {e}")
            raise
    
    def __enter__(self):
        super().__enter__()
        data = self._read_xodr_from_nurec(self.usdz_path)

        self.scenario = Scenario(self.usdz_path)

        ego_poses = self.scenario.ego_poses.poses

        t_world_base = self.scenario.t_world_base
        self.t_scenario_carla = get_t_rig_enu_from_ecef(t_world_base, data)

        print(t_world_base)
        print("===============")

        # self.camera_spec, self.camera_tf = get_camera_spec(self.camera_config_file, "camera_front_rgb")
        # Load all camera specs and transforms from config file
        with open(self.camera_config_file, "r") as f:
            camera_configs = yaml.safe_load(f)
        for cam_cfg in camera_configs:
            name = cam_cfg["camera_params"].get("name")
            # TODO: handle case where name is not within the nine cameras we support
            spec, tf = get_camera_spec(cam_cfg)
            self.cameras[name] = (spec, tf)

        # print(self.cameras)

        self.scenario.tracks.set_view_transform(self.t_scenario_carla)
        self.scenario.tracks.set_mininmum_lifetime(1 / 10)

        # self.add_ego()

        self.renderer = NurecRenderer(
            self.scenario,
            "localhost",
            self.port,
            self.active_actors,
            self.t_scenario_carla,
            self.blueprint_library,
            self.actor_blueprints,
        )

        self._warm_cache()

        return self

    def _warm_cache(self) -> None:
        """
        Renders an initial image at the ego's starting position before the scenario starts.
        This helps initialize the rendering pipeline.
        """
        logger.info("Warming renderer cache...")
        if not self.renderer or self.scenario is None:
            logger.warning(
                "Cannot warm cache: renderer or scenario not initialized"
            )
            return

        # camera_logical_id = list(self.scenario.camera_calibrations.values())[0].logical_sensor_name
        # camera_spec = self.renderer.get_camera_spec(camera_logical_id)

        # self.do_render([], self.camera_spec, np.eye(4))
        # Warm cache for all cameras
        for cam_name, (camera_spec, camera_tf) in self.cameras.items():
            self.do_render([], camera_spec, camera_tf)

        logger.debug("Cache warmed")

    def on_tick(self, actor_transform):
        actor_transform = np.array(actor_transform)

        for cam_name, (camera_spec, camera_tf) in self.cameras.items():
            camera_transform = (
                undo_carla_coordinate_transform(actor_transform) @ camera_tf
            )
            image_bytes = self.do_render(
                [],
                camera_spec,
                camera_transform,
            )

            self.image_queue[cam_name].put(image_bytes)

    def do_render(self, world_snapshot, camera_spec: CameraSpec, pose: np.ndarray, resolution_ratio: float = 0.25) -> np.ndarray:
        timestamp = int(self.scenario.tracks.current_time)

        # bound timestamp to the range of the scenario
        timestamp = min(timestamp, self.renderer.end_timestamp - 1)

        # print(pose)
        # print(self.renderer.t_carla_nurec)
        # print(timestamp)

        request = generate_request(
            self.renderer.scene_id,
            camera_spec,
            pose,
            timestamp,
            resolution_ratio,
            world_snapshot,
            ImageFormat.JPEG,
            self.renderer.active_actors,
            self.scenario.controllable_tracks,
            self.renderer.t_carla_nurec,
            self.renderer.blueprint_library,
            self.renderer.actor_blueprints,
        )
        response = self.renderer.client_service.render_rgb(request)

        image = self.renderer.jpeg_decoder.decode(response.image_bytes)
        # convert to uint8
        image = np.array(image.cpu()).astype(np.uint8)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) # Convert RGB to BGR for OpenCV

        _, encoded = cv2.imencode(".jpg", image)

        # Convert to bytes
        # self.image_bytes = encoded.tobytes()
        image_bytes = encoded.tobytes()

        return image_bytes

        # write as jpeg to disk under data/camera_name/
        # out_dir = "/home/yuchen.wang/workspace/carla/PythonAPI/examples/nvidia/nurec/data"
        # os.makedirs(out_dir, exist_ok=True)
        # array = image.astype(np.uint8)
        # filename = f"{self.IMG_COUNTER:05d}"
        # imageio.imwrite(f"{out_dir}/{filename}.jpg", array)
        # return image


# def set_ego_simple_trajectory_following(self, time_spacing: int = 25000) -> SimpleTrajectoryFollower:
#     """
#     Set up simple trajectory following for the ego vehicle starting from current time.
    
#     Args:
#         time_spacing: Time spacing between trajectory points in microseconds (default: 25ms)

#     Returns:
#         SimpleTrajectoryFollower: The created trajectory follower instance
#     """
#     if self.scenario is None:
#         raise RuntimeError("Scenario not initialized. Call __enter__ first.")

#     # Create trajectory follower
#     trajectory_follower = SimpleTrajectoryFollower(
#         nurec_actor=ego_actor, world=self.get_world()
#     )

#     # Set trajectory from current time to end of scenario
#     current_time = self.get_sim_time()
#     _, end_time = self.get_scenario_time_range()

#     # Use current scenario time as start, or scenario start if we haven't started yet
#     start_time = max(
#         current_time, self.scenario.metadata["pose-range"]["start-timestamp_us"]
#     )

#     trajectory_follower.set_trajectory_from_track(
#         start_time, end_time, time_spacing=time_spacing
#     )

def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        "--host",
        metavar="H",
        default="127.0.0.1",
        help="IP of the host server (default: 127.0.0.1)",
    )
    argparser.add_argument(
        "-p",
        "--port",
        metavar="P",
        default=2000,
        type=int,
        help="TCP port to listen to (default: 2000)",
    )
    argparser.add_argument(
        '-o', '--output-dir',
        metavar='O',
        default="data",
        help='output directory (data)')
    argparser.add_argument(
        "-np",
        "--nurec-port",
        metavar="Q",
        default=46435,
        type=int,
        help="nurec port (default: 46435)",
    )
    argparser.add_argument(
        "-u",
        "--usdz-filename",
        metavar="U",
        required=True,
        help="Path to the USDZ file containing the NUREC scenario data",
    )
    args = argparser.parse_args()

    pose_queue = queue.Queue()
    image_queue = {"front_left": queue.Queue(),
                   "front_right": queue.Queue(),
                   "front_center": queue.Queue(),
                   "left_front": queue.Queue(),
                   "left_side": queue.Queue(),
                   "left_rear": queue.Queue(),
                   "right_front": queue.Queue(),
                   "right_side": queue.Queue(),
                   "right_rear": queue.Queue()}

    # Start gRPC server
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    image_stream_pb2_grpc.add_ImageServiceServicer_to_server(ImageService(pose_queue, image_queue), server)

    server.add_insecure_port("[::]:50051")
    server.start()

    with PlusNurecServer(
        args.usdz_filename,
        port=args.nurec_port,
        fps=30,
        image_queue=image_queue,
    ) as plus_nurec:
        try:

            print("Starting replay")
            while True:

                matrix = pose_queue.get()
                plus_nurec.on_tick(matrix)
                # time.sleep(1)
        except KeyboardInterrupt:
            logger.info("Keyboard interrupt detected, exiting gracefully.")
            return
        except Exception as e:
            print(e)
            traceback.print_exc()


class ImageService(image_stream_pb2_grpc.ImageServiceServicer):

    def __init__(self, pose_queue, image_queue):
        self.pose_queue = pose_queue
        self.image_queue = image_queue

    def safe_get(self, q, timeout=0.1):
        try:
            return q.get(timeout=timeout)
        except queue.Empty:
            return b""

    def GetLatestImage(self, request, context):
        print("Received ImageRequest")

        matrix = transform_to_matrix(request.ego_x,
                                     request.ego_y,
                                     request.ego_z,
                                     request.ego_roll,
                                     request.ego_pitch,
                                     request.ego_yaw)

        self.pose_queue.put(matrix)

        return image_stream_pb2.ImageResponse(front_center_image=self.safe_get(self.image_queue["front_center"]),
                                              front_left_image=self.safe_get(self.image_queue["front_left"]),
                                              front_right_image=self.safe_get(self.image_queue["front_right"]),
                                              left_front_image=self.safe_get(self.image_queue["left_front"]),
                                              left_side_image=self.safe_get(self.image_queue["left_side"]),
                                              left_rear_image=self.safe_get(self.image_queue["left_rear"]),
                                              right_front_image=self.safe_get(self.image_queue["right_front"]),
                                              right_side_image=self.safe_get(self.image_queue["right_side"]),
                                              right_rear_image=self.safe_get(self.image_queue["right_rear"]))


if __name__ == "__main__":
    # serve()
    main()