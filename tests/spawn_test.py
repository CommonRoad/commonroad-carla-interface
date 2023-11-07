import os
import time
import carla
import logging

from carlacr.helper.config import CarlaParams

''' This is just a script to test CARLA by itself '''


def draw_spawnpoints(cworld: carla.World, points, road_id=None):
    for spawn_point in points:
        if ((road_id is None) or (road_id == spawn_point.road_id)):
            cworld.debug.draw_string(spawn_point.transform.location, 'O', draw_shadow=False,
                                     color=carla.Color(r=0, g=255, b=0), persistent_lines=True)


def log_spawnpoints(points):
    for point in points:
        info = point.transform
        logger.info("")


logger = logging.getLogger(__name__)

odr_map = "/../maps/DEU_Aachen-16_21_I-1.cr.xodr"
odr_map_path = os.path.dirname(__file__) + odr_map

with open(odr_map_path, encoding="utf-8") as od_file:
    data = od_file.read()

config = CarlaParams()
config.map = odr_map_path

client = carla.Client(config.host, config.port)
client.generate_opendrive_world(data,
                                carla.OpendriveGenerationParameters(vertex_distance=config.map_params.vertex_distance,
                                        max_road_length=config.map_params.max_road_length,
                                        wall_height=config.map_params.wall_height,
                                        additional_width=config.map_params.extra_width, smooth_junctions=True,
                                        enable_mesh_visibility=True, enable_pedestrian_navigation=False))
time.sleep(config.sleep_time)
world: carla.World = client.get_world()
world_map: carla.Map = world.get_map()
spawn_points = world_map.get_spawn_points()
logger.info(spawn_points)
# draw_spawnpoints(world, world_map.generate_waypoints(distance=1.0), None)

world.debug.draw_point(carla.Location(x=0, y=0, z=0), size=0.5, color=carla.Color(r=0, g=0, b=255), life_time=0)

world.debug.draw_point(carla.Location(x=2, y=0, z=0), size=0.5, color=carla.Color(r=255, g=0, b=0), life_time=0)

world.debug.draw_point(carla.Location(x=0, y=2, z=0), size=0.5, color=carla.Color(r=0, g=255, b=0), life_time=0)

# world.debug.draw_point(carla.Location(x=4, y=76, z=0), size=0.5, color=carla.Color(r=255, g=255, b=255), life_time=0)

lib: carla.BlueprintLibrary = world.get_blueprint_library()
blueprint1 = lib.find("vehicle.audi.a2")
transform1 = carla.Transform(carla.Location(x=4, y=2, z=0), carla.Rotation(yaw=60))

actor1: carla.Actor = world.spawn_actor(blueprint1, transform1)
# now check new actor transform

blueprint2 = lib.find("vehicle.kawasaki.ninja")
transform2 = carla.Transform(carla.Location(x=4, y=76, z=0), carla.Rotation(yaw=0))
actor2: carla.Actor = world.spawn_actor(blueprint2, transform2)

world.tick()

print(actor1.get_transform().location.x)
print(actor1.get_transform().location.y)
print(actor1.get_transform().location.z)
print()

print(actor2.get_transform().location.x)
print(actor2.get_transform().location.y)
print(actor2.get_transform().location.z)


actor1.set_target_velocity(carla.Vector3D(x=10, y=0, z=0))

# actor2.set_target_velocity(carla.Vector3D(x=-5, y=0, z=0))


# this works flawlessly
