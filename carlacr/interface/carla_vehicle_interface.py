import sys
from math import sqrt
from numpy import array, pi, random
import logging
import carla
from commonroad.scenario.state import CustomState as State
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import (DynamicObstacle, ObstacleType)
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad.prediction.prediction import TrajectoryPrediction

from carlacr.helper.vehicle_dict import (vehicle_dict)

logger = logging.getLogger(__name__)


class CarlaVehicleInterface:
    """
    A InterfaceObstacle is a intermediate obstacle representation to
    help translate between CR-Obstacles and CARLA-Obstacles
    Based on CARLAs PythonAPI example: PythonAPI/examples/spawn_npc.py
    Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
    Barcelona (UAB).
    This work is licensed under the terms of the MIT license.
    For a copy, see <https://opensource.org/licenses/MIT>.
    """

    def __init__(self, cr_scenario: Scenario, carla_client: carla.Client, traffic_manager_port: int = 8000):
        """

        :param cr_scenario: CommonRoad scenario object
        :param carla_client: carla.Client() object connected to the simulation
        :param traffic_manager_port: port of the CARLA traffic manager
        """
        self.is_spawned = False
        self.client = carla_client
        self.tm_port = traffic_manager_port
        self.scenario = cr_scenario
        self.commonroad_id = None
        self.carla_id = None
        self.has_controler = None
        self.dynamic_obstacle: DynamicObstacle = None

    def __str__(self):
        resp = f"commonroad_id: {self.commonroad_id}\n"
        resp += f"carla_id: {self.carla_id}\n"
        resp += f"is_spawned: {self.is_spawned}\n"
        resp += f"traffic manager port: {self.tm_port}\n"
        return resp

    def get_cr_state(self, time_step=0) -> State:
        """
        Get current CommonRoad state if spawned, else None

        :return: CommonRoad state of the CARLA vehicle represented by this class
        """
        if self.carla_id:
            try:
                actor = self.client.get_world().get_actor(self.carla_id)
                vel_vec = actor.get_velocity()
                vel = sqrt(vel_vec.x ** 2 + vel_vec.y ** 2)  # velocity
                # ang_vec = actor.get_angular_velocity()
                # ang = sqrt(ang_vec.x ** 2 + ang_vec.y ** 2)  # angular velocity
                transform = actor.get_transform()
                location = transform.location
                rotation = transform.rotation
                return State(position=array([location.x, -location.y]), orientation=-((rotation.yaw * pi) / 180),
                             velocity=vel, time_step=time_step)
            except Exception as e:
                logger.debug("Following error occured while retrieving current position for:")
                logger.debug(self)
                logger.error(e, exc_info=sys.exc_info())
                return None
        else:
            return None

    def get_cr_dynamic_obstacle(self):
        return self.dynamic_obstacle

    def create_cr_dynamic_obstacle(self) -> DynamicObstacle:
        """
        Returns a CommonRoad DynamicObstacle from this CARLA Vehicle

        :return: CommonRoad dynamic obstacle of the CARLA vehicle represented by this class
        """
        if self.is_spawned and self.carla_id:
            actor = self.client.get_world().get_actor(self.carla_id)
            length = actor.bounding_box.extent.x * 2
            width = actor.bounding_box.extent.y * 2
            dynamic_obstacle_type = ObstacleType.CAR  # TODO: check if maybe bike or truck etc.
            dynamic_obstacle_shape = Rectangle(width=width, length=length)
            dynamic_obstacle_init_state = self.get_cr_state(time_step=0)
            dynamic_obstacle_trajectory = Trajectory(initial_time_step=0, state_list=[dynamic_obstacle_init_state])
            # TODO: get Vehicle Light State -> Problem: not all vehicles have a light state yet!
            obs = DynamicObstacle(self.commonroad_id,
                                  dynamic_obstacle_type,
                                  dynamic_obstacle_shape,
                                  initial_state=dynamic_obstacle_init_state,
                                  prediction=TrajectoryPrediction(trajectory=dynamic_obstacle_trajectory,
                                                                  shape=dynamic_obstacle_shape))
            self.dynamic_obstacle = obs
            return obs
        return None

    def update_after_spawn(self, spawned=True, cr_id=None, actor_id=None):
        """
        Stores assigned CommonRoad & CARLA Id in the class and sets is_spawned flag

        :param spawned: sets is_spawned flag
        :param cr_id: sets CommonRoad id for the class
        :param actor_id: sets CARLA id for the class
        """
        self.is_spawned = spawned
        self.commonroad_id = cr_id
        self.carla_id = actor_id

    def get_spawnable(self, random_vehicle=True, blue_print=None, spawn_point=None) -> carla.command.SpawnActor:

        """
        :param random_vehicle: if true a random blue print & random spawn point will be used to create a spawnable
        :param blue_print: blue print to be used to create the vehicle
        :param spawn_point: spawn point to be used to spawn the vehicle
        :return: a spawnable actor to be spawned in bulk
        """
        world = self.client.get_world()

        traffic_manager = self.client.get_trafficmanager(self.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(1.0)

        # Get spawn-point:
        if random_vehicle & (spawn_point is None):
            possible_spawn_points = world.get_map().get_spawn_points()
            if not possible_spawn_points:
                logger.warning("There are no spawnable points for carla vehicle")
                return None
            transform = random.choice(possible_spawn_points)
        elif spawn_point:
            transform = spawn_point
        else:
            logging.warning("Set random=True or provide a spawn_point")
            return None
        # Select blue-print:
        if random_vehicle & (blue_print is None):
            choice = random.choice(list(vehicle_dict))
            bps = world.get_blueprint_library().filter(choice)
            if not bps:
                # Carla can not find the vehicle in vehicle_dict. Require an update on the vehicle name
                # https://carla.readthedocs.io/en/latest/bp_library/
                raise AttributeError(choice)
            bp = bps[0]
        elif blue_print:
            bp = blue_print
        else:
            logger.debug("Set random=True or provide a blue_print")
            return None
        # Configure Blueprint
        if bp.has_attribute('color'):
            color = random.choice(
                bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)
        if bp.has_attribute('driver_id'):
            driver_id = random.choice(bp.get_attribute('driver_id').recommended_values)
            bp.set_attribute('driver_id', driver_id)
            self.carla_id = driver_id
        bp.set_attribute('role_name', 'autopilot')

        # prepare the light state of the cars to spawn
        light_state = carla.VehicleLightState.NONE
        spawn_actor = carla.command.SpawnActor
        set_autopilot = carla.command.SetAutopilot
        set_vehicle_light_state = carla.command.SetVehicleLightState
        future_actor = carla.command.FutureActor

        # spawn the cars and set their autopilot and light state all together
        return (spawn_actor(bp, transform)
                .then(set_autopilot(future_actor, True, traffic_manager.get_port()))
                .then(set_vehicle_light_state(future_actor, light_state)))
