import logging
import math
import random
from typing import List, Optional, Union

import carla
from carla import VehicleControl
from commonroad.planning.planner_interface import TrajectoryPlannerInterface
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleRole, ObstacleType, SignalState
from commonroad.scenario.scenario import Scenario
from crpred.predictor_interface import PredictorInterface

from crcarla.controller.commonroad_planner import CommonRoadPlannerController
from crcarla.controller.controller import TransformControl, create_carla_transform
from crcarla.controller.keyboard_controller import KeyboardVehicleController
from crcarla.controller.steering_wheel import SteeringWheelController
from crcarla.controller.vehicle_controller import (
    AckermannController,
    PIDController,
    VehicleBehaviorAgentPathFollowingControl,
    VehicleTMPathFollowingControl,
)
from crcarla.helper.config import (
    ApproximationType,
    EgoPlanner,
    EgoVehicleParams,
    VehicleControlType,
    VehicleParams,
)
from crcarla.helper.utils import create_cr_vehicle_from_actor
from crcarla.helper.vehicle_dict import similar_by_area, similar_by_length, similar_by_width
from crcarla.objects.actor import ActorInterface

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class VehicleInterface(ActorInterface):
    """Interface between CARLA vehicle and CommonRoad pedestrian."""

    def __init__(
        self,
        cr_obstacle: DynamicObstacle,
        world: carla.World,
        tm: Optional[carla.TrafficManager],
        config: Union[VehicleParams, EgoVehicleParams] = VehicleParams(),
        actor: Optional[carla.Vehicle] = None,
        planner: Optional[TrajectoryPlannerInterface] = None,
        predictor: Optional[PredictorInterface] = None,
        sc: Optional[Scenario] = None,
        pp: Optional[PlanningProblem] = None,
    ):
        """
        Initializer of vehicle interface.

        :param cr_obstacle: CommonRoad obstacle corresponding to the actor.
        :param world: CARLA world
        :param tm: CARLA traffic manager.
        :param config: Vehicle or pedestrian config parameters.
        :param actor: New CARLA actor. None if actor is not spawned yet.
        :param planner: CommonRoad trajectory planner.
        :param predictor: CommonRoad predictor.
        :param sc: CommonRoad scenario.
        :param pp: CommonRoad planning problem.
        """
        super().__init__(cr_obstacle, world, tm, actor, config)
        self._planner = planner
        self._predictor = predictor
        self._pp = pp
        self._sc = sc

    def _init_controller(self):
        """Initializes CARLA vehicle controller."""
        if hasattr(self._config, "ego_planner"):
            if self._config.ego_planner is EgoPlanner.STEERING_WHEEL:
                self._controller = SteeringWheelController(self._actor, self._config.control.steering_wheel_params)
            elif self._config.ego_planner is EgoPlanner.KEYBOARD:
                self._controller = KeyboardVehicleController(self._actor, self._config.simulation.time_step)
            elif self._config.ego_planner is EgoPlanner.PLANNER:
                self._controller = CommonRoadPlannerController(
                    self._actor,
                    self._planner,
                    self._predictor,
                    self._pp,
                    self._sc,
                    self._config.carla_controller_type,
                    self._config.simulation.time_step,
                    self._config.simulation.time_horizon_seconds,
                    self._config.control,
                )
        else:
            if self._config.carla_controller_type is VehicleControlType.TRANSFORM:
                self._controller = TransformControl(self._actor)
            elif self._config.carla_controller_type is VehicleControlType.PID:
                self._controller = PIDController(
                    actor=self._actor,
                    config=self._config.control,
                    dt=self._config.simulation.time_step,
                )
            elif self._config.carla_controller_type is VehicleControlType.ACKERMANN:
                self._controller = AckermannController(
                    self._actor, config=self._config.control, dt=self._config.simulation.time_step
                )
            elif self._config.carla_controller_type is VehicleControlType.PATH_TM:
                self._controller = VehicleTMPathFollowingControl(self._actor)
            elif self._config.carla_controller_type is VehicleControlType.PATH_AGENT:
                self._controller = VehicleBehaviorAgentPathFollowingControl(self._actor)

    def get_scenario(self) -> Scenario:
        """Getter for scenario

        :return CommonRoad scenario.
        """
        return self._sc

    def _spawn(self, time_step: int):
        """
        Tries to spawn the vehicle (incl. lights if supported) in the given CARLA world at provided time step.

        :param time_step: Current time step.
        """
        if self._cr_obstacle is None:
            self._create_random_actor(self._sc.generate_object_id())
        if time_step != self._cr_obstacle.initial_state.time_step or self.spawned:
            return

        if self._cr_obstacle.obstacle_type in [
            ObstacleType.CAR,
            ObstacleType.TRUCK,
            ObstacleType.BUS,
            ObstacleType.PRIORITY_VEHICLE,
            ObstacleType.PARKED_VEHICLE,
            ObstacleType.MOTORCYCLE,
            ObstacleType.TAXI,
        ]:
            self._actor = self._create_cr_actor()
        else:
            raise RuntimeError("VehicleInterface::_spawn: Unknown obstacle type.")

        if not self._actor:
            logger.warning("VehicleInterface::_spawn: After _spawn got called self._actor is still None!")

        # init traffic manager if vehicle will be controlled by it
        self._init_tm__agent_actor_path()

    def _init_tm__agent_actor_path(self):
        """Initializes traffic manager for path if corresponding control type is used"""
        if self._cr_obstacle.obstacle_role is ObstacleRole.DYNAMIC:
            if self._config.carla_controller_type == VehicleControlType.PATH_TM:
                self._tm.set_path(self._actor, self._get_path())

            elif self._config.carla_controller_type == VehicleControlType.PATH_AGENT:
                self._controller.set_path(self._get_path())

            else:
                # needed to avoid throttle delay (see https://github.com/carla-simulator/carla/issues/1640)
                self._actor.apply_control(VehicleControl(manual_gear_shift=True, gear=1))
                self._actor.apply_control(VehicleControl(manual_gear_shift=False))

    def _create_cr_actor(self):
        """Creates CARLA vehicle given CommonRoad dynamic obstacle."""
        obstacle_blueprint = self._match_blueprint()
        transform = create_carla_transform(self._cr_obstacle.initial_state)
        actor = self._world.try_spawn_actor(obstacle_blueprint, transform)
        if not actor:
            logger.error("Error while spawning CR obstacle: %s", self.cr_obstacle.obstacle_id)
            spawn_points = self._world.get_map().get_spawn_points()
            closest = transform
            best_dist = math.inf
            for point in spawn_points:
                dist = point.location.distance(transform.location)
                if dist < best_dist:
                    best_dist = dist
                    closest = point
            actor = self._world.try_spawn_actor(obstacle_blueprint, closest)
            logger.info(
                "Obstacle %s spawned %s m away from original position",
                self.cr_obstacle.obstacle_id,
                best_dist,
            )
        actor.set_simulate_physics(self._config.physics)
        logger.debug("Spawn successful: CR-ID %s CARLA-ID %s", self._cr_obstacle.obstacle_id, actor.id)
        # Set up the lights to initial states:
        vehicle = self._world.get_actor(actor.id)
        if self._cr_obstacle.initial_signal_state:
            if vehicle:
                sig = self._cr_obstacle.initial_signal_state
                self._set_light(sig=sig)
        if self._config.carla_controller_type is VehicleControlType.TRANSFORM:
            actor.set_target_velocity(carla.Vector3D(0, 0, 0))
        else:
            yaw = transform.rotation.yaw * (math.pi / 180)
            vx = self._cr_obstacle.initial_state.velocity * math.cos(yaw)
            vy = self._cr_obstacle.initial_state.velocity * math.sin(yaw)
            actor.set_target_velocity(carla.Vector3D(vx, vy, 0))
        return actor

    def _create_random_actor(self, obs_id: int):
        """Creates a random actor"""
        # similar function exists for traffic generation but cannot be combined

        # randomly select a blueprint and check, if the config settings are matching with the bp.
        lib = self._world.get_blueprint_library().filter(self._config.simulation.filter_vehicle)
        while True:
            blueprint = random.choice(lib)
            # skip bikes
            if (
                self._config.simulation.filter_attribute_number_of_wheels is not None
                and blueprint.has_attribute("number_of_wheels")
                and blueprint.get_attribute("number_of_wheels").as_int()
                == self._config.simulation.filter_attribute_number_of_wheels
            ):
                break  # vehicle passed through filter

        if blueprint.has_attribute("color"):
            color = random.choice(blueprint.get_attribute("color").recommended_values)
            blueprint.set_attribute("color", color)

        ego_actor = None
        while ego_actor is None:
            spawn_points = self._world.get_map().get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            ego_actor = self._world.try_spawn_actor(blueprint, spawn_point)

        self._cr_obstacle = create_cr_vehicle_from_actor(ego_actor, obs_id, initial_time_step=0)
        self._actor = ego_actor

    def _match_blueprint(self):
        """Matches actor dimensions to available CARLA actor blueprint based on length, with, or area."""
        nearest_vehicle_type = None
        if self._config.approximation_type == ApproximationType.LENGTH:
            nearest_vehicle_type = similar_by_length(
                self._cr_obstacle.obstacle_shape.length, self._cr_obstacle.obstacle_shape.width, 0
            )
        if self._config.approximation_type == ApproximationType.WIDTH:
            nearest_vehicle_type = similar_by_width(
                self._cr_obstacle.obstacle_shape.length, self._cr_obstacle.obstacle_shape.width, 0
            )
        if self._config.approximation_type == ApproximationType.AREA:
            nearest_vehicle_type = similar_by_area(
                self._cr_obstacle.obstacle_shape.length, self._cr_obstacle.obstacle_shape.width, 0
            )
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

    def _get_path(self) -> List[carla.Location]:
        """
        Computes path which will be followed by CARLA traffic manager given CommonRoad trajectory.

        :return: List of CARLA locations.
        """
        if self._cr_obstacle.obstacle_role is not ObstacleRole.DYNAMIC:
            return [
                carla.Location(
                    x=self._cr_obstacle.initial_state.position[0],
                    y=-self._cr_obstacle.initial_state.position[1],
                    z=0.5,
                )
            ]
        path = []
        for time_step in range(0, len(self._cr_obstacle.prediction.trajectory.state_list), self._config.path_sampling):
            state = self._cr_obstacle.prediction.trajectory.state_list[time_step]
            path.append(carla.Location(x=state.position[0], y=-state.position[1], z=0.5))
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
        elif self._cr_obstacle.obstacle_role is ObstacleRole.DYNAMIC:
            self._controller.control(self.cr_obstacle.state_at_time(time_step))
            self._set_light(self.cr_obstacle.signal_state_at_time_step(time_step))
