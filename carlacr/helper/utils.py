import carla
import math
import numpy as np
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.state import InitialState

def create_cr_vehicle_from_actor(ego_actor: carla.Actor) -> DynamicObstacle:
    vel_vec = ego_actor.get_velocity()
    vel = math.sqrt(vel_vec.x ** 2 + vel_vec.y ** 2)
    transform = ego_actor.get_transform()
    location = transform.location
    rotation = transform.rotation
    length = ego_actor.bounding_box.extent.x * 2
    width = ego_actor.bounding_box.extent.y * 2
    return DynamicObstacle(0, ObstacleType.CAR, Rectangle(length, width),
                           InitialState(0, np.array([location.x, -location.y]),
                                        -((rotation.yaw * math.pi) / 180), vel, 0, 0, 0))

def create_cr_pedestrian_from_actor(ego_actor: carla.Walker) -> DynamicObstacle:
    vel_vec = ego_actor.get_velocity()
    vel = math.sqrt(vel_vec.x ** 2 + vel_vec.y ** 2)
    transform = ego_actor.get_transform()
    location = transform.location
    rotation = transform.rotation
    length = ego_actor.bounding_box.extent.x * 2
    width = ego_actor.bounding_box.extent.y * 2
    return DynamicObstacle(0, ObstacleType.CAR, Rectangle(length, width),
                           InitialState(0, np.array([location.x, -location.y]),
                                        -((rotation.yaw * math.pi) / 180), vel, 0, 0, 0))