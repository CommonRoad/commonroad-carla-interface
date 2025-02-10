import logging
import math
from dataclasses import dataclass
from typing import List, Optional

import carla
from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.controller import VehiclePIDController
from commonroad.scenario.state import InputState, KSState, TraceState

from crcarla.controller.controller import CarlaController, create_carla_transform
from crcarla.helper.config import ControlParams

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

try:
    from carla import AckermannControllerSettings, VehicleAckermannControl, VehicleControl
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
                vel = math.sqrt(state.velocity**2 + state.velocity_y**2)
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
        self._agent.set_global_plan(
            [(CarlaCRWaypoint(elem), None) for elem in path],
            stop_waypoint_creation=True,
            clean_queue=True,
        )


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

    def control(self, state: Optional[TraceState] = None) -> VehicleControl:
        """
        Computes and applies CARLA PID control for one time step.

        :param state: State which should be reached at next time step.
        """
        target = CarlaCRWaypoint(create_carla_transform(state))
        speed = state.velocity * 3.6

        control = self._pid.run_step(speed, target)
        self._actor.apply_control(control)

        return control


class AckermannController(CarlaController):
    """Controller which uses CARLA's Ackermann controller to control the vehicle."""

    def __init__(self, actor: carla.Actor, dt: float, config: ControlParams = ControlParams()):
        """
        Initialization of Ackermann controller.

        :param actor: CARLA actor which will be controlled.
        :param config: Controller configuration.
        """
        super().__init__(actor)
        ackermann_settings = AckermannControllerSettings(
            speed_kp=config.ackermann_pid_speed_kp,
            speed_ki=config.ackermann_pid_speed_ki,
            speed_kd=config.ackermann_pid_speed_kd,
            accel_kp=config.ackermann_pid_accel_kp,
            accel_ki=config.ackermann_pid_accel_ki,
            accel_kd=config.ackermann_pid_accel_kd,
        )
        self._actor.apply_ackermann_controller_settings(ackermann_settings)
        self._previous_state: Optional[KSState] = None
        self._dt = dt

    def control(self, state: Optional[KSState] = None, input: Optional[InputState] = None):
        """
        Computes and applies CARLA Ackermann control for one time step.

        :param state: State which should be reached at next time step.
        """
        if not isinstance(state, KSState):
            logger.error("AckermannController::control: state must be of type KSState.")
            raise RuntimeError("AckermannController::control: state must be of type KSState.")
        acc_vec = self._actor.get_acceleration()
        acc = math.sqrt(acc_vec.x**2 + acc_vec.y**2)
        if input is None and self._previous_state is not None:
            input_state = InputState(
                acceleration=(state.velocity - self._previous_state.velocity) / self._dt,
                steering_angle_speed=(state.steering_angle - self._previous_state.steering_angle) / self._dt,
            )
            jerk = (((state.velocity - self._previous_state.velocity) / self._dt) - acc) / self._dt
        elif input is None and self._previous_state is None:
            vel_vec = self._actor.get_velocity()
            vel = math.sqrt(vel_vec.x**2 + vel_vec.y**2)
            steer = self._actor.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
            input_state = InputState(
                acceleration=(state.velocity - vel) / self._dt,
                steering_angle_speed=(state.steering_angle - steer) / self._dt,
            )
            jerk = (((state.velocity - vel) / self._dt) - acc) / self._dt
        else:
            input_state = input
            jerk = (input.acceleration - acc) / self._dt

        ackermann_control = VehicleAckermannControl(
            steer=-state.steering_angle,
            steer_speed=input_state.steering_angle_speed,
            speed=state.velocity,
            acceleration=input_state.acceleration,
            jerk=jerk,
        )

        self._actor.apply_ackermann_control(ackermann_control)
