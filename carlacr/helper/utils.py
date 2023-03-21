import carla
import math
import os
import numpy as np
from typing import Union, List
import logging
import psutil
import shutil

from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.geometry.shape import Rectangle, Circle
from commonroad.scenario.state import InitialState, PMState, KSState, CustomState
from commonroad.common.util import make_valid_orientation
from commonroad.planning.goal import GoalRegion, Interval, AngleInterval
from commonroad.scenario.scenario import Scenario


logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


# def _get_nearby_vehicles(self, vehicles, ego, distance_th):
#     """Shows nearby vehicles of the hero actor"""
#     info_text = []
#     if self.hero_actor is not None and len(vehicles) > 1:
#         location = self.hero_transform.location
#         vehicle_list = [x[0] for x in vehicles if x[0].id != self.hero_actor.id]
#
#         def distance(v):
#             return location.distance(v.get_location())
#
#         for n, vehicle in enumerate(sorted(vehicle_list, key=distance)):
#             if n > distance_th:
#                 break
#             vehicle_type = get_actor_display_name(vehicle, truncate=22)
#             info_text.append('% 5d %s' % (vehicle.id, vehicle_type))
#     self._hud.add_info('NEARBY VEHICLES', info_text)

def create_goal_region_from_state(state: Union[KSState, PMState], ks_state: bool = True) -> GoalRegion:
    if ks_state:
        return GoalRegion([CustomState(time_step=Interval(state.time_step, state.time_step),
                                       position=Circle(3, state.position),
                                       velocity=Interval(max(0.0, state.velocity - 10), state.velocity + 10),
                                       orientation=AngleInterval(state.orientation - 0.25, state.orientation + 0.25))])
    else:
        velocity = max(state.velocity, state.velocity_y)
        orientation = math.atan2(state.velocity_y, state.velocity)
        return GoalRegion([CustomState(time_step=Interval(state.time_step, state.time_step),
                                       position=Circle(10, state.position),
                                       velocity=Interval(max(0.0, velocity - 10), velocity + 10),
                                       orientation=AngleInterval(orientation - 0.25, orientation + 0.25))])


def create_cr_vehicle_from_actor(actor: carla.Actor, cr_id: int) -> DynamicObstacle:
    vel_vec = actor.get_velocity()
    vel = math.sqrt(vel_vec.x ** 2 + vel_vec.y ** 2)
    transform = actor.get_transform()
    location = transform.location
    orientation = -((transform.rotation.yaw * math.pi) / 180)
    length = actor.bounding_box.extent.x * 2
    width = actor.bounding_box.extent.y * 2
    return DynamicObstacle(cr_id, ObstacleType.CAR, Rectangle(length, width),
                           InitialState(0, np.array([location.x, -location.y]),
                                        orientation, vel, 0, 0, 0))


def create_cr_pm_state_from_actor(actor: carla.Actor, time_step: int) -> PMState:
    vel_vec = actor.get_velocity()
    vel = math.sqrt(vel_vec.x ** 2 + vel_vec.y ** 2)
    transform = actor.get_transform()
    location = transform.location
    orientation = -((transform.rotation.yaw * math.pi) / 180)
    velocity_x = math.cos(orientation) * vel
    velocity_y = math.sin(orientation) * vel
    return PMState(time_step, np.array([location.x, -location.y]), velocity_x, velocity_y)


def create_cr_ks_state_from_actor(actor: carla.Vehicle, time_step: int) -> KSState:
    vel_vec = actor.get_velocity()
    vel = math.sqrt(vel_vec.x ** 2 + vel_vec.y ** 2)
    transform = actor.get_transform()
    location = transform.location
    rotation = transform.rotation
    orientation = make_valid_orientation(-((rotation.yaw * math.pi) / 180))
    try:
        steer = actor.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
    except RuntimeError:
        steer = 0
    steering_angle = make_valid_orientation(steer * (math.pi/180))
    return KSState(time_step, np.array([location.x, -location.y]), steering_angle, vel, orientation)

 #   slow_vehicles: {'vehicle.seat.leon', 'vehicle.dodge.charger_2020', 'vehicle.audi.tt', 'vehicle.chevrolet.impala', 'vehicle.mercedes.coupe', 'vehicle.lincoln.mkz_2017', 'vehicle.volkswagen.t2_2021', 'vehicle.citroen.c3', 'vehicle.bmw.grandtourer', 'vehicle.mini.cooper_s_2021', 'vehicle.nissan.micra', 'vehicle.dodge.charger_police_2020', 'vehicle.tesla.model3', 'vehicle.nissan.patrol_2021', 'vehicle.jeep.wrangler_rubicon', 'vehicle.audi.etron', 'vehicle.ford.crown', 'vehicle.mini.cooper_s'}


def create_cr_pedestrian_from_walker(actor: carla.Walker, cr_id: int, default_shape: bool = False) -> DynamicObstacle:
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
    return DynamicObstacle(cr_id, ObstacleType.PEDESTRIAN, shape,
                           InitialState(0, np.array([location.x, -location.y]),
                                        -((rotation.yaw * math.pi) / 180), vel, 0, 0, 0))


def calc_max_timestep(sc: Scenario) -> int:
    """
    Calculates maximal time step of current scenario.

    :param sc: scenario to calculate max time step
    :return: length of scenario
    """
    time_steps = [obstacle.prediction.final_time_step for obstacle in sc.dynamic_obstacles]
    return np.max(time_steps) if time_steps else 0


def find_pid_by_name(process_name: str) -> List[int]:
    """
    Get a list of all the PIDs of a all the running process whose name contains
    the given string processName

    :returns List of possible PIDs
    """
    processes = []
    for proc in psutil.process_iter():
        try:
            if process_name.lower() in proc.name().lower():
                processes.append(proc.pid)
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return processes


def make_video(path: str, video_name: str):
    """Creates a video of the images recorded by camera sensor using ffmepg.

    @param path: Path to png images stored by camera sensor.
    @param video_name: Name which new video should have.
    """
    tmp_path = os.path.join(path, "_tmp")
    if not os.path.exists(path):
        os.mkdir(path)
    if not os.path.exists(tmp_path):
        os.mkdir(tmp_path)
    try:
        os.system(f"ffmpeg -framerate 10 -hide_banner -loglevel error -pattern_type glob -i \'{tmp_path}/*.png\'"
                  f" -c:v libx264 -pix_fmt yuv420p {path}/{video_name}.mp4")
        shutil.rmtree(tmp_path)
    except Exception as e:
        if os.path.exists(os.path.join(path, video_name + ".mp4")):
            logger.debug("mp4 created!")
            pass
        else:
            logger.error(e)
