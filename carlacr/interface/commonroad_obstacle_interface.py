from enum import Enum
import logging
import carla
import numpy as np
from commonroad.scenario.obstacle import Obstacle, ObstacleRole, ObstacleType
from commonroad.scenario.trajectory import State
import pygame
from carlacr.helper.vehicle_dict import (similar_by_area, similar_by_length, similar_by_width)

from carlacr.configurations.set_configs import set_configs
from agents.navigation.controller import VehiclePIDController
# pycharm might suggest "unresolved reference" here, false alarm

config = set_configs()

logger = logging.getLogger(__name__)

SpawnActor = carla.command.SpawnActor


class ApproximationType(Enum):
    LENGTH = 0
    WIDTH = 1
    AREA = 2


class CommonRoadObstacleInterface:
    """
    One to one representation of a CommonRoad obstacle to be worked with in CARLA
    (Caution: only vehicles are currently supported, basic setup for pedestrians in comments)
    """

    def __init__(self, cr_obstacle: Obstacle):
        """

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        self.commonroad_id = cr_obstacle.obstacle_id
        self.carla_id = None
        self.is_spawned = False
        self.spawn_timestep = cr_obstacle.initial_state.time_step
        self.init_state = cr_obstacle.initial_state
        self.init_signal_state = cr_obstacle.initial_signal_state
        self.trajectory = cr_obstacle.prediction.trajectory if cr_obstacle.obstacle_role == ObstacleRole.DYNAMIC else\
            None
        self.signal_series = cr_obstacle.signal_series
        self.size = (cr_obstacle.obstacle_shape.length, cr_obstacle.obstacle_shape.width, 0)  # (x, y, z)
        self.role = cr_obstacle.obstacle_role
        self.type = cr_obstacle.obstacle_type
        self.cr_base = cr_obstacle
        # merge
        self.clock = pygame.time.Clock()
        self.client = carla.Client

        # uncomment to use Ackermann controller
        # _args = config.config_carla_obstacle
        # self.ackermann_settings = carla.AckermannControllerSettings(speed_kp=_args.speed_kp, speed_ki=_args.speed_ki,
        #                                                             speed_kd=_args.speed_kd,accel_kp=_args.accel_kp,
        #                                                             accel_ki=_args.accel_ki, accel_kd=_args.accel_kd)

    def spawn(self, world: carla.World, approx_type=ApproximationType.LENGTH, physics=True) -> carla.Actor:
        """
        Tries to spawn the vehicle (incl. lights if supported) in the given CARLA world and returns the spawned vehicle.

        :param world: the CARLA world object
        :param approx_type: based on what approximation of the vehicle size the blueprint should be selected
        :param physics: if physics should be enabled for the vehicle
        :return: if spawn successful the according CARLA actor else None
        """
        transform = carla.Transform(
            carla.Location(x=self.init_state.position[0], y=-self.init_state.position[1], z=0.5),
            carla.Rotation(yaw=(-(180 * self.init_state.orientation) / np.pi)))
        # # PEDESTRIAN
        if self.type == ObstacleType.PEDESTRIAN:
            obstacle_blueprint_walker = world.get_blueprint_library().find('walker.pedestrian.0002')
            try:
                obstacle = world.spawn_actor(obstacle_blueprint_walker, transform)  # parent_walker
                if obstacle:
                    obstacle.set_simulate_physics(physics)
                    logger.debug("Spawn successful: CR-ID %s CARLA-ID %s", self.commonroad_id, obstacle.id)
                    self.carla_id = obstacle.id
                    self.is_spawned = True
            except Exception as e:
                logger.error("Error while spawning PEDESTRIAN:")
                raise e

        # VEHICLE
        if self.type in [ObstacleType.CAR, ObstacleType.TRUCK, ObstacleType.BUS, ObstacleType.PRIORITY_VEHICLE,
                         ObstacleType.PARKED_VEHICLE, ObstacleType.MOTORCYCLE, ObstacleType.TAXI]:
            if approx_type == ApproximationType.LENGTH:
                nearest_vehicle_type = similar_by_length(self.size[0], self.size[1], 0)
            if approx_type == ApproximationType.WIDTH:
                nearest_vehicle_type = similar_by_width(self.size[0], self.size[1], 0)
            if approx_type == ApproximationType.AREA:
                nearest_vehicle_type = similar_by_area(self.size[0], self.size[1], 0)
            obstacle_blueprint = world.get_blueprint_library().filter(nearest_vehicle_type[0])[0]

            try:
                obstacle = world.try_spawn_actor(obstacle_blueprint, transform)
                if obstacle:
                    obstacle.set_simulate_physics(physics)
                    logger.debug("Spawn successful: CR-ID %s CARLA-ID %s", self.commonroad_id, obstacle.id)

                    # Set up the lights to initial states:
                    vehicle = world.get_actor(obstacle.id)
                    if self.init_signal_state:
                        if vehicle:
                            sig = self.init_signal_state
                            self._set_up_lights(vehicle=vehicle, sig=sig)
                    self.carla_id = obstacle.id
                    self.is_spawned = True
            except Exception as e:
                logger.error("Error while spawning VEHICLE:")
                raise e

    def update_position_by_time(self, world: carla.World, state: State):
        """
        Tries to update the position of the obstacle and sets lights
        :param world: the CARLA world object
        :param state:state at the time step
        """

        try:
            if self.is_spawned & (self.role == ObstacleRole.DYNAMIC) & (self.trajectory is not None):
                actor = world.get_actor(self.carla_id)
                if actor:
                    new_orientation = state.orientation
                    new_position = state.position
                    transform = carla.Transform(

                        carla.Location(x=new_position[0], y=-new_position[1], z=actor.get_location().z),
                        carla.Rotation(yaw=(-(180 * new_orientation) / np.pi)))

                    actor.set_transform(transform)

                else:
                    logger.debug("Could not find actor")
        except Exception as e:
            logger.error("Error while updating position")
            raise e

    def update_position_by_control(self, world: carla.World, state: State):
        """
        This function controls the obstacle vehicle to drive along the planned route (for one step) and sets the lights.
        From the motion planner of the autonomous vehicle a CommonRoad trajectory is expected. From the trajectory the
        desired velocity and the waypoint(position) can be extracted.

        :param world: the CARLA world object
        :param state: state at the time step
        """

        try:
            if self.is_spawned & (self.role == ObstacleRole.DYNAMIC) & (self.trajectory is not None):
                vehicle = world.get_actor(self.carla_id)

                if vehicle:
                    _pid = VehiclePIDController(vehicle,
                                                config.config_carla_obstacle.args_lateral_dict,
                                                config.config_carla_obstacle.args_long_dict)

                    _target = self.trajectory.position(state.time_step)
                    _speed = self.trajectory.velocity(state.time_step)

                    control = _pid.run_step(_speed, _target)
                    vehicle.apply_control(control)

                    # Do the lights:
                    self._set_up_lights(vehicle, state)

                else:
                    logger.debug("Could not find vehicle (obstacle)")
        except Exception as e:
            logger.error("Error while updating position")
            raise e

    def update_with_ackermann_control(self, world: carla.World, state: State):
        """"
        Prepare to update the position with ackermann controller (version 0.9.14+ required)

        :param world: the CARLA world object
        :param state:state at the time step
        """

        vehicle = world.get_actor(self.carla_id)
        try:
            _steering_angle = self.trajectory.steering_angle(state.time_step)
            _steering_angle_speed = self.trajectory.steering_angle_speed(state.time_step)
            _speed = self.trajectory.velocity(state.time_step)
            _acceleration = _speed = self.trajectory.acceleration(state.time_step)
            _jerk = self.trajectory.jerk(state.time_step)

            # Define the Ackermann control
            ackermann_control = carla.VehicleAckermannControl(
                steer=_steering_angle,
                steer_speed=_steering_angle_speed,
                speed=_speed,
                acceleration=_acceleration,
                jerk=_jerk
            )

            # Set the parameters of the PID
            vehicle.apply_ackermann_controller_settings(self.ackermann_settings)

            # Apply the Ackermann control to the vehicle
            vehicle.apply_ackermann_control(ackermann_control)

            # Do the lights:
            self._set_up_lights(vehicle, state)

        except Exception as e:
            logger.error("Error while updating position")
            raise e

    def destroy_carla_obstacle(self, world):
        """
        Destroys vehicle in CARLA

        :param world: the CARLA world object
        """
        if self.is_spawned:
            actor = world.get_actor(self.carla_id)
            if actor:
                actor.destroy()

    def _set_up_lights(self, vehicle, state: State = None, sig=None):
        # vehicle = world.get_actor(self.carla_id)
        z = carla.VehicleLightState.NONE
        vehicle.set_light_state(z)
        if sig is None and state is not None:
            sig = self.cr_base.signal_state_at_time_step(state.time_step)
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

    def __str__(self):
        resp = f"commonroad_id: {self.commonroad_id}\n"
        resp += f"carla_id: {self.carla_id}\n"
        resp += f"is_spawned: {self.is_spawned}\n"
        resp += f"spawn_timestep: {self.spawn_timestep}\n"
        resp += f"next_update_time: {self.next_update_time}\n"
        resp += f"trajectory: {{{self.trajectory}\n}}\n"
        resp += f"size: {self.size}\n"
        resp += f"role: {self.role}"
        resp += f"type: {self.type}\n"
        return resp
