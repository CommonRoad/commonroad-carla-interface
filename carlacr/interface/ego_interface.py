import logging

import carla
from agents.navigation.controller import VehiclePIDController

from commonroad.scenario.obstacle import Obstacle, ObstacleRole, ObstacleType
from commonroad.scenario.trajectory import State

from carlacr.interface.vehicle_interface import VehicleInterface

logger = logging.getLogger(__name__)


class EgoInterface(VehicleInterface):
    """One to one representation of a CommonRoad obstacle to be worked with in CARLA."""

    def __init__(self, cr_obstacle: Obstacle):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        super().__init__(cr_obstacle)
        # _args = config.config_carla_obstacle
        # self.ackermann_settings = carla.AckermannControllerSettings(speed_kp=_args.speed_kp, speed_ki=_args.speed_ki,
        #                                                             speed_kd=_args.speed_kd,accel_kp=_args.accel_kp,
        #                                                             accel_ki=_args.accel_ki, accel_kd=_args.accel_kd)

    def update_position_by_control(self, world: carla.World, state: State):
        """
        Function controls the obstacle vehicle to drive along the planned route (for one step) and sets the lights.

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
        """
        Prepare to update the position with ackermann controller (version 0.9.14+ required).

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
