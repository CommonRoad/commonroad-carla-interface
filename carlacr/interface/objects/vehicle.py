import logging
from typing import Optional, List
import carla
import math
import random

from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType, ObstacleRole
from commonroad.scenario.trajectory import State

from carlacr.helper.vehicle_dict import (similar_by_area, similar_by_length, similar_by_width)
from carlacr.helper.config import ObstacleParams, ApproximationType, VehicleControlType
from carlacr.interface.obstacle.obstacle_interface import ObstacleInterface
from carlacr.interface.controller.controller import create_carla_transform, TransformControl
from carlacr.interface.controller.vehicle_controller import PIDController, AckermannController, WheelController, \
    VehiclePathFollowingControl
from carlacr.interface.controller.keyboard_controller import KeyboardVehicleController
from carlacr.helper.utils import create_cr_vehicle_from_actor

logger = logging.getLogger(__name__)


class VehicleInterface(ObstacleInterface):
    """One to one representation of a CommonRoad obstacle to be worked with in CARLA."""

    def __init__(self, cr_obstacle: DynamicObstacle, spawned: bool = False,
                 carla_id: Optional[int] = None, config: ObstacleParams = ObstacleParams()):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        super().__init__(cr_obstacle, config)
        self._is_spawned = spawned
        self._carla_id = carla_id
        self._hud = None
        self._vis_world = None
        self._controller = self._init_controller()

    def _init_controller(self):
        if self._config.controller_type is VehicleControlType.TRANSFORM:
            self._controller = TransformControl()
        elif self._config.controller_type is VehicleControlType.PID:
            self._controller = PIDController(actor=None, config=self._config.control)
        elif self._config.controller_type is VehicleControlType.ACKERMANN:
            self._controller = AckermannController(config=self._config.control)
        elif self._config.controller_type is VehicleControlType.STEERING_WHEEL:
            self._controller = WheelController()
        elif self._config.controller_type is VehicleControlType.KEYBOARD:
            self._controller = KeyboardVehicleController()
        elif self._config.controller_type is VehicleControlType.PLANNER:
            self._controller = None
        elif self._config.controller_type is VehicleControlType.PATH:
            self._controller = VehiclePathFollowingControl()


    def spawn(self, world: carla.World, time_step: int, tm: Optional[carla.TrafficManager] = None):
        """
        Tries to spawn the vehicle (incl. lights if supported) in the given CARLA world and returns the spawned vehicle.

        :param world: the CARLA world object
        :return: if spawn successful the according CARLA actor else None
        """
        self._world = world
        if self._cr_base is None or time_step != self._cr_base.initial_state.time_step:
            self._spawn_v2(world, time_step)

        transform = create_carla_transform(self._cr_base.initial_state)

        if self._cr_base.obstacle_type in \
                [ObstacleType.CAR, ObstacleType.TRUCK, ObstacleType.BUS, ObstacleType.PRIORITY_VEHICLE,
                 ObstacleType.PARKED_VEHICLE, ObstacleType.MOTORCYCLE, ObstacleType.TAXI]:
            obstacle_blueprint = self._find_blueprint(world)

            obstacle = world.try_spawn_actor(obstacle_blueprint, transform)
            if not obstacle:
                logger.error(f"Error while spawning CR obstacle: {self.cr_obstacle.obstacle_id}")
                spawn_points = world.get_map().get_spawn_points()
                closest = None
                best_dist = math.inf
                for point in spawn_points:
                    dist = point.location.distance(transform.location)
                    if dist < best_dist:
                        best_dist = dist
                        closest = point
                obstacle = world.try_spawn_actor(obstacle_blueprint, closest)
                logger.info(f"Obstacle {self.cr_obstacle.obstacle_id} spawned {best_dist}m away from original position")

            obstacle.set_simulate_physics(self._config.physics)
            logger.debug("Spawn successful: CR-ID %s CARLA-ID %s", self._commonroad_id, obstacle.id)
            # Set up the lights to initial states:
            vehicle = world.get_actor(obstacle.id)
            if self._cr_base.initial_signal_state:
                if vehicle:
                    sig = self._cr_base.initial_signal_state
                    self._set_up_lights(vehicle=vehicle, sig=sig)
            yaw = transform.rotation.yaw * (math.pi / 180)
            vx = self._cr_base.initial_state.velocity * math.cos(yaw)
            vy = self._cr_base.initial_state.velocity * math.sin(yaw)
            obstacle.set_target_velocity(carla.Vector3D(vx, vy, 0))
            self._carla_id = obstacle.id
            self._is_spawned = True
        else:
            self._spawn_v2(world, time_step)

        if self._config.controller_type == VehicleControlType.PATH:
            if self.get_role() == ObstacleRole.DYNAMIC:
                tm.set_path(world.get_actor(self._carla_id), self._get_path())

    def _spawn_v2(self, world, time_step):
        # function taken from CARLA
        # Spawns the hero actor when the script runs"""
        # Get a random blueprint.
        blueprint = random.choice(world.get_blueprint_library().filter(self._config.simulation.filter_vehicle))
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        # Spawn the player.
        ego_actor = None
        while ego_actor is None:
            spawn_points = world.get_map().get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            ego_actor = world.try_spawn_actor(blueprint, spawn_point)

        # Save it in order to destroy it when closing program
        self._is_spawned = True
        self._carla_id = ego_actor.id
        self._commonroad_id = 0
        self._spawn_timestep = time_step
        self._cr_base = create_cr_vehicle_from_actor(ego_actor, 0)

    def _find_blueprint(self, world):
        nearest_vehicle_type = None
        if self._config.approximation_type == ApproximationType.LENGTH:
            nearest_vehicle_type = similar_by_length(self._cr_base.obstacle_shape.length,
                                                     self._cr_base.obstacle_shape.width, 0)
        if self._config.approximation_type == ApproximationType.WIDTH:
            nearest_vehicle_type = similar_by_width(self._cr_base.obstacle_shape.length,
                                                    self._cr_base.obstacle_shape.width, 0)
        if self._config.approximation_type == ApproximationType.AREA:
            nearest_vehicle_type = similar_by_area(self._cr_base.obstacle_shape.length,
                                                   self._cr_base.obstacle_shape.width, 0)
        obstacle_blueprint = world.get_blueprint_library().filter(nearest_vehicle_type[0])[0]
        return obstacle_blueprint

    def _set_up_lights(self, vehicle, state: State = None, sig=None):
        """
        Sets up the lights of the Obstacle.

        :param state: state at the time step
        :param sig: signals of the Obstacle
        """
        # vehicle = world.get_actor(self.carla_id)
        z = carla.VehicleLightState.NONE
        vehicle.set_light_state(z)
        if sig is None and state is not None:
            sig = self._cr_base.signal_state_at_time_step(state.time_step)
        if sig:
            if sig.braking_lights:
                z = z | carla.VehicleLightState.Brake
            if sig.indicator_left and not sig.hazard_warning_lights:
                z = z | carla.VehicleLightState.LeftBlinker
            if sig.indicator_right and not sig.hazard_warning_lights:
                z = z | carla.VehicleLightState.RightBlinker
            if sig.hazard_warning_lights:
                z = z | carla.VehicleLightState.RightBlinker
                z = z | carla.VehicleLightState.LeftBlinker
            vehicle.set_light_state(carla.VehicleLightState(z))

    def _get_path(self) -> List[carla.Location]:
        if self._cr_base.obstacle_role is not ObstacleRole.DYNAMIC:
            return [carla.Location(x=self._cr_base.initial_state.position[0],
                                   y=-self._cr_base.initial_state.position[1],
                                   z=0.5)]
        else:
            path = []
            for time_step in range(0, len(self._cr_base.prediction.trajectory.state_list), self._config.path_sampling):
                state = self._cr_base.prediction.trajectory.state_list[time_step]
                path.append(carla.Location(x=state.position[0],  y=-state.position[1], z=0.5))
            if len(self._cr_base.prediction.trajectory.state_list) % self._config.path_sampling != 0:
                state = self._cr_base.prediction.trajectory.state_list[-1]
                path.append(carla.Location(x=state.position[0], y=-state.position[1], z=0.5))
            return path

    def tick(self, state: State):
        self._controller.control(state)
        self._time_step += 1
