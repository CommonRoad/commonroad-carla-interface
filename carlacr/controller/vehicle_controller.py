import dataclasses
import logging
from typing import Optional
import carla
import math

from carlacr.helper.config import ControlParams
from carlacr.controller.controller import CarlaController, create_carla_transform
from carlacr.agents.navigation.controller import VehiclePIDController

from commonroad.scenario.state import TraceState

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

try:
    from carla import VehicleAckermannControl, AckermannControllerSettings
except ImportError:
    logger.info("AckermannControl not available! Please upgrade your CARLA version!")

@dataclasses.dataclass
class CarlaCRWaypoint:
    transform: carla.Transform


class VehiclePathFollowingControl(CarlaController):
    def __init__(self, actor: carla.Actor):
        super().__init__(actor)

    def control(self, actor: Optional[carla.Actor] = None, state: Optional[TraceState] = None,
                tm: Optional[carla.TrafficManager] = None):
        if hasattr(tm, "set_desired_speed"):
            if hasattr(state, "velocity_y"):
                vel = math.sqrt(state.velocity ** 2 + state.velocity_y ** 2)
                tm.set_desired_speed(actor, vel)
            else:
                tm.set_desired_speed(actor, state.velocity)

class PIDController(CarlaController):
    def __init__(self, actor: carla.Actor, config: ControlParams = ControlParams()):
        super().__init__(actor)

        self._pid = VehiclePIDController(actor, config.pid_lat_dict(), config.pid_lon_dict())

    def control(self, state: Optional[TraceState] = None):
        """
        Function controls the obstacle vehicle to drive along the planned route (for one step) and sets the lights.

        From the motion planner of the autonomous vehicle a CommonRoad trajectory is expected. From the trajectory the
        desired velocity and the waypoint(position) can be extracted.

        :param world: the CARLA world object
        :param state: state at the time step
        """

        target = CarlaCRWaypoint(create_carla_transform(state))
        speed = state.velocity

        control = self._pid.run_step(speed, target)
        self._actor.apply_control(control)


class AckermannController(CarlaController):
    def __init__(self, actor: carla.Actor, config: ControlParams = ControlParams()):
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
        Prepare to update the position with ackermann controller (version 0.9.14+ required).

        :param world: the CARLA world object
        :param state:state at the time step
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

            # Set the parameters of the PID


            # Apply the Ackermann control to the vehicle
            self._actor.apply_ackermann_control(ackermann_control)

        except Exception as e:
            logger.error("Error while updating position")
            raise e

class WheelController(CarlaController):
    def __init__(self, actor: carla.Actor, ):
        super().__init__(actor)

    def control(self, state: Optional[TraceState] = None):
        pass