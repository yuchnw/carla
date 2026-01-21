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

import carla

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)

# Read OpenDRIVE file
# with open("/home/yuchen.wang/Downloads/bunkerD_dynamic_width.xodr", "r") as f:
#     xodr_data = f.read()

# Optional parameters
# params = carla.OpendriveGenerationParameters(
#     vertex_distance=2.0,
#     max_road_length=500.0,
#     wall_height=0.0,
#     additional_width=0.0,
#     smooth_junctions=True,
#     enable_mesh_visibility=True
# )

# Generate world
# world = client.generate_opendrive_world(xodr_data, params)
world = client.get_world()

# spectator = world.get_spectator()

# spectator.set_transform(
#     carla.Transform(
#         carla.Location(x=0, y=0, z=80),
#         carla.Rotation(pitch=-90)
#     )
# )

# Basic validation
carla_map = world.get_map()
print("Map name:", carla_map.name)

# Waypoint sanity check
wps = carla_map.generate_waypoints(2.0)
print("Total waypoints:", len(wps))
debug = world.debug

# Get any waypoint on the lane
wp = wps[0]

print("Lane width:", wp.lane_width)

# for wp in carla_map.generate_waypoints(10.0):
#     start = wp.transform.location
#     forward = wp.transform.get_forward_vector()
#     end = start + forward * 2.0

#     debug.draw_arrow(
#         start, end,
#         thickness=0.1,
#         arrow_size=0.3,
#         color=carla.Color(255, 0, 0),
#         life_time=10.0
#     )