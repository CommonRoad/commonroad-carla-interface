import logging
from typing import Optional
from carla import World
try:
    from carla import VehicleAckermannControl, AckermannControllerSettings
except ImportError:
    print("TODO logging")

from commonroad.scenario.obstacle import DynamicObstacle, ObstacleRole
from commonroad.scenario.trajectory import State

from carlacr.interface.obstacle.vehicle_interface import VehicleInterface
from carlacr.helper.config import ObstacleParams

logger = logging.getLogger(__name__)


class EgoInterface(VehicleInterface):
    """One to one representation of a CommonRoad obstacle to be worked with in CARLA."""
    def __init__(self, cr_obstacle: Optional[DynamicObstacle] = None, config: ObstacleParams = ObstacleParams()):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        super().__init__(cr_obstacle, config)

    def control(self, state: Optional[State] = None):
        pass

    def start(self, hud, world):
        pass

    def tick(self, clock):
        pass

class AckermannEgoInterface(EgoInterface):
    """One to one representation of a CommonRoad obstacle to be worked with in CARLA."""

    def __init__(self, cr_obstacle: DynamicObstacle, config: ObstacleParams = ObstacleParams()):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        super().__init__(cr_obstacle, config)
        self.ackermann_settings = AckermannControllerSettings(speed_kp=config.control.ackermann_pid_speed_kp,
                                                                    speed_ki=config.control.ackermann_pid_speed_ki,
                                                                    speed_kd=config.control.ackermann_pid_speed_kd,
                                                                    accel_kp=config.control.ackermann_pid_accel_kp,
                                                                    accel_ki=config.control.ackermann_pid_accel_ki,
                                                                    accel_kd=config.control.ackermann_pid_accel_kd)

    def control(self, state: Optional[State] = None):
        """
        Prepare to update the position with ackermann controller (version 0.9.14+ required).

        :param world: the CARLA world object
        :param state:state at the time step
        """
        vehicle = self._world.get_actor(self.carla_id)
        try:
            _steering_angle = self.trajectory.steering_angle(state.time_step)
            _steering_angle_speed = self.trajectory.steering_angle_speed(state.time_step)
            _speed = self.trajectory.velocity(state.time_step)
            _acceleration = _speed = self.trajectory.acceleration(state.time_step)
            _jerk = self.trajectory.jerk(state.time_step)

            # Define the Ackermann control
            ackermann_control = VehicleAckermannControl(
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

class PIDEgoInterface(EgoInterface):
    """One to one representation of a CommonRoad obstacle to be worked with in CARLA."""

    def __init__(self, cr_obstacle: DynamicObstacle, config: ObstacleParams = ObstacleParams()):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        super().__init__(cr_obstacle, config)
        from agents.navigation.controller import VehiclePIDController
        self._pid = VehiclePIDController(vehicle, self._config.control.args_lateral_dict,
                                    self._config.config_carla_obstacle.args_long_dict)

    def control(self, state: Optional[State] = None):
        """
        Function controls the obstacle vehicle to drive along the planned route (for one step) and sets the lights.

        From the motion planner of the autonomous vehicle a CommonRoad trajectory is expected. From the trajectory the
        desired velocity and the waypoint(position) can be extracted.

        :param world: the CARLA world object
        :param state: state at the time step
        """
        try:
            if self._is_spawned and self._cr_base.obstacle_role == ObstacleRole.DYNAMIC and \
                    self._cr_base.prediction.trajectory is not None:
                vehicle = self._world.get_actor(self._carla_id)

                if vehicle:

                    _target = self._cr_base.state_at_time(state.time_step).position
                    _speed = self._cr_base.state_at_time(state.time_step).velocity

                    control = self._pid.run_step(_speed, _target)
                    vehicle.apply_control(control)

                    # Do the lights:
                    self._set_up_lights(vehicle, state)

                else:
                    logger.debug("Could not find vehicle (obstacle)")
        except Exception as e:
            logger.error("Error while updating position")
            raise e

class WheelEgoInterface(EgoInterface):
    """One to one representation of a CommonRoad obstacle to be worked with in CARLA."""

    def __init__(self, cr_obstacle: DynamicObstacle, config: ObstacleParams = ObstacleParams()):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        super().__init__(cr_obstacle, config)

    def control(self, state: Optional[State] = None):
        pass


class KeyboardEgoInterface(EgoInterface):
    """One to one representation of a CommonRoad obstacle to be worked with in CARLA."""

    def __init__(self, cr_obstacle: DynamicObstacle, config: ObstacleParams = ObstacleParams()):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        super().__init__(cr_obstacle, config)

    def control(self, state: Optional[State] = None):
        pass