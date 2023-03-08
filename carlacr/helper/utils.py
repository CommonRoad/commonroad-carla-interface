import carla
import math
import numpy as np
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.geometry.shape import Rectangle, Circle
from commonroad.scenario.state import InitialState, PMState, KSState
from commonroad.common.util import make_valid_orientation

def create_cr_vehicle_from_actor(actor: carla.Actor) -> DynamicObstacle:
    vel_vec = actor.get_velocity()
    vel = math.sqrt(vel_vec.x ** 2 + vel_vec.y ** 2)
    transform = actor.get_transform()
    location = transform.location
    rotation = transform.rotation
    orientation = -((rotation.yaw * math.pi) / 180)
    length = actor.bounding_box.extent.x * 2
    width = actor.bounding_box.extent.y * 2
    return DynamicObstacle(0, ObstacleType.CAR, Rectangle(length, width),
                           InitialState(0, np.array([location.x, -location.y]),
                                        orientation, vel, 0, 0, 0))

def create_cr_pm_state_from_actor(actor: carla.Actor, time_step: int) -> PMState:
    vel_vec = actor.get_velocity()
    location = actor.get_transform().location
    return PMState(time_step, np.array([location.x, -location.y]), vel_vec.x, vel_vec.y)

def create_cr_ks_state_from_actor(actor: carla.Vehicle, time_step: int) -> KSState:
    vel_vec = actor.get_velocity()
    vel = math.sqrt(vel_vec.x ** 2 + vel_vec.y ** 2)
    transform = actor.get_transform()
    location = transform.location
    rotation = transform.rotation
    orientation = make_valid_orientation(-((rotation.yaw * math.pi) / 180))
    steering_angle = make_valid_orientation(actor.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel) * (math.pi/180))
    return KSState(time_step, np.array([location.x, -location.y]), steering_angle, vel, orientation)


def create_cr_pedestrian_from_walker(actor: carla.Walker, default_shape: bool = False) -> DynamicObstacle:
    vel_vec = actor.get_velocity()
    vel = math.sqrt(vel_vec.x ** 2 + vel_vec.y ** 2)
    transform = actor.get_transform()
    location = transform.location
    rotation = transform.rotation
    length = actor.bounding_box.extent.x * 2
    width = actor.bounding_box.extent.y * 2
    if default_shape:
        shape = Circle(0.4)
    elif abs(length) == math.inf or abs(width) == math.inf:
        shape = Circle(0.4)
    elif length == width:
        shape = Circle(length / 2)
    else:
        shape = Rectangle(length, width)
    return DynamicObstacle(0, ObstacleType.CAR, shape,
                           InitialState(0, np.array([location.x, -location.y]),
                                        -((rotation.yaw * math.pi) / 180), vel, 0, 0, 0))