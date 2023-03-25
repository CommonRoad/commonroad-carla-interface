from dataclasses import dataclass
import logging
from typing import Optional, List
import carla
import math

from carlacr.helper.config import ControlParams
from carlacr.controller.controller import CarlaController, create_carla_transform
from agents.navigation.controller import VehiclePIDController
from agents.navigation.behavior_agent import BehaviorAgent

from commonroad.scenario.state import TraceState

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

try:
    from carla import VehicleAckermannControl, AckermannControllerSettings
except ImportError:
    logger.info("AckermannControl not available! Please upgrade your CARLA version!")


@dataclass
class CarlaCRWaypoint:
    """Wrapper class for CARLA waypoint."""

    # CARLA transform
    transform: carla.Transform


class VehicleTMPathFollowingControl(CarlaController):
    """Controller which uses the CARLA traffic manager to follow a path."""

    def control(self, state: Optional[TraceState] = None, tm: Optional[carla.TrafficManager] = None):
        """
        Applies CARLA traffic manager path following control. Only adapts current speed.
        This feature is only available for CARLA versions >= 0.9.14.

        :param state: State which should be reached at next time step.
        :param tm: CARLA traffic manager.
        """
        if hasattr(tm, "set_desired_speed"):
            if hasattr(state, "velocity_y"):
                vel = math.sqrt(state.velocity ** 2 + state.velocity_y ** 2)
                tm.set_desired_speed(self._actor, vel)
            else:
                tm.set_desired_speed(self._actor, state.velocity)


class VehicleBehaviorAgentPathFollowingControl(CarlaController):
    """Controller which uses the CARLA agent models to follow a path."""

    def __init__(self, actor: carla.Actor):
        """
        Initialization of CARLA agent path following controller.

        :param actor: CARLA actor which will be controlled.
        """
        super().__init__(actor)
        self._agent = BehaviorAgent(actor)

    def control(self, state: Optional[TraceState] = None):
        """
        Applies CARLA agent path following control. Only adapts current speed.

        :param state: State which should be reached at next time step.
        """
        self._agent.set_target_speed(state.velocity)

    def set_path(self, path: List[carla.Location]):
        """
        Sets path which should be followed by CARLA agent.

        :param path: List of CARLA locations.
        """
        self._agent.set_global_plan([(CarlaCRWaypoint(elem), None) for elem in path],
                                    stop_waypoint_creation=True, clean_queue=True)


class PIDController(CarlaController):
    """Controller which uses CARLA's PID controller to control the vehicle."""

    def __init__(self, actor: carla.Actor, config: ControlParams = ControlParams(), dt: float = 0.1):
        """
        Initialization of PID controller.

        :param actor: CARLA actor which will be controlled.
        :param config: Controller configuration.
        :param dt: Time step size.
        """
        super().__init__(actor)
        self._pid = VehiclePIDController(actor, config.pid_lat_dict(dt), config.pid_lon_dict(dt))

    def control(self, state: Optional[TraceState] = None):
        """
        Computes and applies CARLA PID control for one time step.

        :param state: State which should be reached at next time step.
        """
        target = CarlaCRWaypoint(create_carla_transform(state))
        speed = state.velocity

        control = self._pid.run_step(speed, target)
        self._actor.apply_control(control)


class AckermannController(CarlaController):
    """Controller which uses CARLA's Ackermann controller to control the vehicle."""

    def __init__(self, actor: carla.Actor, config: ControlParams = ControlParams()):
        """
        Initialization of Ackermann controller.

        :param actor: CARLA actor which will be controlled.
        :param config: Controller configuration.
        """
        super().__init__(actor)
        ackermann_settings = AckermannControllerSettings(speed_kp=config.ackermann_pid_speed_kp,
                                                         speed_ki=config.ackermann_pid_speed_ki,
                                                         speed_kd=config.ackermann_pid_speed_kd,
                                                         accel_kp=config.ackermann_pid_accel_kp,
                                                         accel_ki=config.ackermann_pid_accel_ki,
                                                         accel_kd=config.ackermann_pid_accel_kd)
        self._actor.apply_ackermann_controller_settings(ackermann_settings)

    def control(self, state: Optional[TraceState] = None):
        """
        Computes and applies CARLA Ackermann control for one time step.

        :param state: State which should be reached at next time step.
        """
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

            # Apply the Ackermann control to the vehicle
            self._actor.apply_ackermann_control(ackermann_control)

        except Exception as e:
            logger.error("Error while updating position")
            raise e


class WheelController(CarlaController):
    """Controller which uses steering wheel as input."""

    def control(self, state: Optional[TraceState] = None):
        """
        Computes and applies CARLA steering wheel control.

        :param state: State which should be reached at next time step.
        """


class CommonRoadPlannerController(CarlaController):
    """Controller which uses trajectory generated by CommonRoad planner as input."""
