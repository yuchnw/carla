import carla
import math
import statistics
import os


client = carla.Client("localhost", 2000)
client.set_timeout(10.0)

xodr_path = "/home/yuchen.wang/workspace/carla/Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive/chris_fix.xodr"
xodr_path = "/home/yuchen.wang/Downloads/bunkerD_v2.xodr"
xodr_path = "/home/yuchen.wang/Downloads/rescenario.xodr"
xodr_path = "/home/yuchen.wang/Downloads/export_last_3dgs_gomentum_20260326/map.xodr"

with open(xodr_path, encoding='utf-8') as od_file:
    try:
        data = od_file.read()
    except OSError:
        print('file could not be readed.')
print('load opendrive map %r.' % os.path.basename(xodr_path))
vertex_distance = 2.0  # in meters
max_road_length = 500.0 # in meters
wall_height = 0      # in meters
extra_width = 0.6      # in meters
world = client.generate_opendrive_world(
    data, carla.OpendriveGenerationParameters(
        vertex_distance=vertex_distance,
        max_road_length=max_road_length,
        wall_height=wall_height,
        additional_width=extra_width,
        smooth_junctions=True,
        enable_mesh_visibility=True))

# spawn_loc = carla.Transform(carla.Location(x=101.270897, y=-3.053370, z=-1.065442), carla.Rotation(pitch=0.491760, yaw=43.462425, roll=-2.753377))
# print("Ego spawned at: {}".format(spawn_loc))
# ego_bp = world.get_blueprint_library().filter('vehicle.mercedes.coupe_2020')[0]
# world.spawn_actor(ego_bp, spawn_loc)

# world = client.get_world()
m = world.get_map()

geo = m.transform_to_geolocation(carla.Location(x=-34, y=-32, z=0))

print(geo.latitude, geo.longitude)

# spawn_loc = carla.Transform(carla.Location(x=622.063049, y=558.192627, z=1.000000), carla.Rotation(pitch=0, yaw=-136, roll=0))
# spawn_loc = carla.Transform(carla.Location(x=-350.932502444163, y=570.0219581462443, z=1.000000), carla.Rotation(pitch=0, yaw=-58, roll=0))
# print("Ego spawned at: {}".format(spawn_loc))
# ego_bp = world.get_blueprint_library().filter('vehicle.mercedes.coupe_2020')[0]
# spectator = world.get_spectator()
# spectator.set_transform(spawn_loc)
# world.spawn_actor(ego_bp, spawn_loc)
# world.tick()

# # debug = world.debug

# waypoints = m.generate_waypoints(1.0)
# TARGET_ROAD_ID = 10  # change this

# lanes = {}

# for wp in waypoints:
#     # if wp.road_id != TARGET_ROAD_ID:
#     #     continue
#     if wp.road_id not in lanes:
#         lanes[wp.road_id] = {}

#     lane_id = wp.lane_id
#     if lane_id not in lanes[wp.road_id]:
#         lanes[wp.road_id][lane_id] = []

#     lanes[wp.road_id][lane_id].append(wp)

# for road_id, road_lanes in lanes.items():

#     for lane_id, lane_wps in road_lanes.items():
#         lane_wps.sort(key=lambda w: w.s)

#         with open(f"/home/yuchen.wang/Downloads/{road_id}lane_{lane_id}_left.txt", "w") as f_left, \
#             open(f"/home/yuchen.wang/Downloads/{road_id}lane_{lane_id}_right.txt", "w") as f_right:

#             for wp in lane_wps:
#                 loc = wp.transform.location
#                 yaw = math.radians(wp.transform.rotation.yaw)
#                 half_width = wp.lane_width / 2.0

#                 # CARLA: X forward, Y right
#                 right_x = -math.sin(yaw)
#                 right_y =  math.cos(yaw)

#                 # Left boundary
#                 left_x = loc.x - right_x * half_width
#                 left_y = loc.y - right_y * half_width
#                 left_z = loc.z

#                 # Right boundary
#                 right_b_x = loc.x + right_x * half_width
#                 right_b_y = loc.y + right_y * half_width
#                 right_b_z = loc.z

#                 f_left.write(f"{left_x:.3f} {left_y:.3f} {left_z:.3f}\n")
#                 f_right.write(f"{right_b_x:.3f} {right_b_y:.3f} {right_b_z:.3f}\n")

# for wp in waypoints:
#     text = f"{wp.road_id}:{wp.section_id}:{wp.lane_id}"
#     # print(wp.road_id)

#     # right = wp.get_right_lane()
#     # left  = wp.get_left_lane()
#     # world.debug.draw_string(
#     #     wp.transform.location,
#     #     text,
#     #     draw_shadow=False,
#     #     color=carla.Color(255, 255, 255),
#     #     life_time=150.0
#     # )

#     # world.debug.draw_point(
#     #     wp.transform.location,
#     #     size=0.1,
#     #     life_time=100,
#     #     color=carla.Color(255,0,0)
#     # )
# #     print("one waypoint")

#     # if right:
#     #     world.debug.draw_point(
#     #         right.transform.location,
#     #         size=0.1,
#     #         life_time=100,
#     #         color=carla.Color(0,0,255)
#     #     )

#     # if left:
#     #     world.debug.draw_point(
#     #         left.transform.location,
#     #         size=0.1,
#     #         life_time=100,
#     #         color=carla.Color(255,255,0)
#     #     )

#     # loc = wp.transform.location
#     # road_z = loc.z

#     # # Raycast down around this (x,y) to find the first mesh hit
#     # start = carla.Location(loc.x, loc.y, road_z + 50.0)
#     # end   = carla.Location(loc.x, loc.y, road_z - 50.0)

    # hits = world.cast_ray(start, end)
    # if not hits:
    #     continue


    # mesh_z = hits[0].location.z
    # d = road_z - mesh_z  # positive means mesh is below XODR

    # diffs.append(d)