import logging
from typing import Optional, List
import carla
import math
import random
import pygame.time

from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType, ObstacleRole
from commonroad.scenario.obstacle import SignalState

from carlacr.helper.vehicle_dict import (similar_by_area, similar_by_length, similar_by_width)
from carlacr.helper.config import VehicleParams, ApproximationType, VehicleControlType
from carlacr.objects.actor import ActorInterface
from carlacr.controller.controller import create_carla_transform, TransformControl
from carlacr.controller.vehicle_controller import PIDController, AckermannController, WheelController, \
    VehicleTMPathFollowingControl
from carlacr.controller.keyboard_controller import KeyboardVehicleController
from carlacr.helper.utils import create_cr_vehicle_from_actor

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class VehicleInterface(ActorInterface):
    """Interface between CARLA vehicle and CommonRoad pedestrian."""

    def __init__(self, cr_obstacle: DynamicObstacle, world: carla.World, tm: Optional[carla.TrafficManager],
                 actor: Optional[carla.Vehicle] = None, config: VehicleParams = VehicleParams()):
        """
        Initializer of vehicle interface.

        :param cr_obstacle: CommonRoad obstacle corresponding to the actor.
        :param world: CARLA world
        :param tm: CARLA traffic manager.
        :param config: Vehicle or pedestrian config parameters.
        :param actor: New CARLA actor. None if actor is not spawned yet.
        """
        super().__init__(cr_obstacle, world, tm, actor, config)
        self._hud = None
        self._vis_world = None

    def _init_controller(self):
        """Initializes CARLA vehicle controller."""
        if self._config.controller_type is VehicleControlType.TRANSFORM:
            self._controller = TransformControl(self._actor)
        elif self._config.controller_type is VehicleControlType.PID:
            self._controller = PIDController(actor=self._actor, config=self._config.control,
                                             dt=self._config.simulation.time_step)
        elif self._config.controller_type is VehicleControlType.ACKERMANN:
            self._controller = AckermannController(self._actor, config=self._config.control)
        elif self._config.controller_type is VehicleControlType.STEERING_WHEEL:
            self._controller = WheelController(self._actor)
        elif self._config.controller_type is VehicleControlType.KEYBOARD:
            self._controller = KeyboardVehicleController(self._actor)
        elif self._config.controller_type is VehicleControlType.PLANNER:
            self._controller = None
        elif self._config.controller_type is VehicleControlType.PATH_TM:
            self._controller = VehicleTMPathFollowingControl(self._actor)
        elif self._config.controller_type is VehicleControlType.PATH_AGENT:
            self._controller = None

    def _spawn(self, time_step: int):
        """
        Tries to spawn the vehicle (incl. lights if supported) in the given CARLA world at provided time step.

        :param time_step: Current time step.
        """
        if self._cr_obstacle is None:
            self._create_random_actor()
        if time_step != self._cr_obstacle.initial_state.time_step or self.spawned:
            return

        if self._cr_obstacle.obstacle_type in \
                [ObstacleType.CAR, ObstacleType.TRUCK, ObstacleType.BUS, ObstacleType.PRIORITY_VEHICLE,
                 ObstacleType.PARKED_VEHICLE, ObstacleType.MOTORCYCLE, ObstacleType.TAXI]:
            self._actor = self._create_cr_actor()
        else:
            raise RuntimeError("Unknown obstacle type")

        # init traffic manager if vehicle will be controlled by it
        self._init_tm_actor_path()

    def _init_tm_actor_path(self):
        """Initializes traffic manager for path if corresponding control type is used"""
        if self._cr_obstacle.obstacle_role is ObstacleRole.DYNAMIC:
            if self._config.controller_type == VehicleControlType.PATH_TM:
                self._tm.set_path(self._actor, self._get_path())
            elif self._config.controller_type == VehicleControlType.PATH_AGENT:
                self._controller.set_path(self._get_path())

    def _create_cr_actor(self):
        """Creates CARLA vehicle given CommonRoad dynamic obstacle."""
        obstacle_blueprint = self._match_blueprint()
        transform = create_carla_transform(self._cr_obstacle.initial_state)
        actor = self._world.try_spawn_actor(obstacle_blueprint, transform)
        if not actor:
            logger.error("Error while spawning CR obstacle: %s", self.cr_obstacle.obstacle_id)
            spawn_points = self._world.get_map().get_spawn_points()
            closest = None
            best_dist = math.inf
            for point in spawn_points:
                dist = point.location.distance(transform.location)
                if dist < best_dist:
                    best_dist = dist
                    closest = point
            actor = self._world.try_spawn_actor(obstacle_blueprint, closest)
            logger.info("Obstacle %s spawned %s m away from original position", self.cr_obstacle.obstacle_id, best_dist)
        actor.set_simulate_physics(self._config.physics)
        logger.debug("Spawn successful: CR-ID %s CARLA-ID %s", self._cr_obstacle.obstacle_id, actor.id)
        # Set up the lights to initial states:
        vehicle = self._world.get_actor(actor.id)
        if self._cr_obstacle.initial_signal_state:
            if vehicle:
                sig = self._cr_obstacle.initial_signal_state
                self._set_light(sig=sig)
        yaw = transform.rotation.yaw * (math.pi / 180)
        vx = self._cr_obstacle.initial_state.velocity * math.cos(yaw)
        vy = self._cr_obstacle.initial_state.velocity * math.sin(yaw)
        actor.set_target_velocity(carla.Vector3D(vx, vy, 0))
        return actor

    def _create_random_actor(self):
        """Creates a random actor"""
        # TODO check whether this can be combined with function in traffic manager.
        blueprint = random.choice(self._world.get_blueprint_library().filter(self._config.simulation.filter_vehicle))
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        ego_actor = None
        while ego_actor is None:
            spawn_points = self._world.get_map().get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            ego_actor = self._world.try_spawn_actor(blueprint, spawn_point)

        self._cr_obstacle = create_cr_vehicle_from_actor(ego_actor, 0)
        self._actor = ego_actor

    def _match_blueprint(self):
        """Matches actor dimensions to available CARLA actor blueprint based on length, with, or area."""
        nearest_vehicle_type = None
        if self._config.approximation_type == ApproximationType.LENGTH:
            nearest_vehicle_type = similar_by_length(self._cr_obstacle.obstacle_shape.length,
                                                     self._cr_obstacle.obstacle_shape.width, 0)
        if self._config.approximation_type == ApproximationType.WIDTH:
            nearest_vehicle_type = similar_by_width(self._cr_obstacle.obstacle_shape.length,
                                                    self._cr_obstacle.obstacle_shape.width, 0)
        if self._config.approximation_type == ApproximationType.AREA:
            nearest_vehicle_type = similar_by_area(self._cr_obstacle.obstacle_shape.length,
                                                   self._cr_obstacle.obstacle_shape.width, 0)
        obstacle_blueprint = self._world.get_blueprint_library().filter(nearest_vehicle_type[0])[0]
        return obstacle_blueprint

    def _set_light(self, sig: SignalState):
        """
        Sets lights of vehicle.

        :param sig: Current signals of vehicle.
        """
        z = carla.VehicleLightState.NONE
        if sig is not None:
            if sig.braking_lights:
                z = z | carla.VehicleLightState.Brake
            if sig.indicator_left and not sig.hazard_warning_lights:
                z = z | carla.VehicleLightState.LeftBlinker
            if sig.indicator_right and not sig.hazard_warning_lights:
                z = z | carla.VehicleLightState.RightBlinker
            if sig.hazard_warning_lights:
                z = z | carla.VehicleLightState.RightBlinker
                z = z | carla.VehicleLightState.LeftBlinker
            self._actor.set_light_state(carla.VehicleLightState(z))

    def register_clock(self, clock: pygame.time.Clock, hud, vis_world):
        """
        Registers some properties required for some controllers.

        :param clock: Pygame clock.
        :param hud: Head-up display object.
        :param vis_world: Visualized world.
        """
        self._controller.register(clock, hud, vis_world)

    def _get_path(self) -> List[carla.Location]:
        """
        Computes path which will be followed by CARLA traffic manager given CommonRoad trajectory.

        :return: List of CARLA locations.
        """
        if self._cr_obstacle.obstacle_role is not ObstacleRole.DYNAMIC:
            return [carla.Location(x=self._cr_obstacle.initial_state.position[0],
                                   y=-self._cr_obstacle.initial_state.position[1],
                                   z=0.5)]
        path = []
        for time_step in range(0, len(self._cr_obstacle.prediction.trajectory.state_list), self._config.path_sampling):
            state = self._cr_obstacle.prediction.trajectory.state_list[time_step]
            path.append(carla.Location(x=state.position[0],  y=-state.position[1], z=0.5))
        if len(self._cr_obstacle.prediction.trajectory.state_list) % self._config.path_sampling != 0:
            state = self._cr_obstacle.prediction.trajectory.state_list[-1]
            path.append(carla.Location(x=state.position[0], y=-state.position[1], z=0.5))
        return path

    def tick(self, time_step: int):
        """
        Performs one-step planning/simulation. If actor is not spawned yet, it will be spawned.

        :param time_step: Current time step.
        """
        if not self.spawned:
            self._spawn(time_step)
            self._init_controller()
        elif self._cr_obstacle.obstacle_role is ObstacleRole.DYNAMIC and time_step > 0:
            self._controller.control(self.cr_obstacle.state_at_time(time_step))
            self._set_light(self.cr_obstacle.signal_state_at_time_step(time_step))
