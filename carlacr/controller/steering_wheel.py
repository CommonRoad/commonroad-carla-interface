import math
from typing import Optional

import carla
import pygame
from commonroad.scenario.state import TraceState

from carlacr.controller.controller import CarlaController


class SteeringWheelController(CarlaController):
    """Controller which uses steering wheel as input."""

    def __init__(self, actor: carla.Actor):
        """
        Initializes input member variables when instance is created.

        :param actor:
        """
        super().__init__(actor)

        self._steer_cache = 0.0

        # initialize steering wheel
        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            raise ValueError("Please Connect Just One Joystick")

        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()

        # values for our logitech gaming wheel TODO to in config
        self._steer_idx = 0
        self._throttle_idx = 2
        self._brake_idx = 3
        self._reverse_idx = 6  # R2 on wheel
        self._handbrake_idx = 7  # L2 on wheel
        self._reverse_activated = False

    def control(self, state: Optional[TraceState] = None):
        """
        Computes and applies CARLA steering wheel control. Parses input and computes vehicle inputs.

        :param state: CommonRoad state which should be reached at next time step. Not used for steering wheel control.
        """
        pygame.event.get()  # required to get steering wheel information
        self._parse_vehicle_wheel()

    def _parse_vehicle_wheel(self):
        control = carla.VehicleControl()
        num_axes = self._joystick.get_numaxes()
        js_inputs = [float(self._joystick.get_axis(i)) for i in range(num_axes)]
        js_buttons = [float(self._joystick.get_button(i)) for i in range(self._joystick.get_numbuttons())]
        k1 = 1.0
        control.steer = k1 * math.tan(1.1 * js_inputs[self._steer_idx])

        if bool(js_buttons[self._reverse_idx]) is True and self._reverse_activated:
            self._reverse_activated = False
        elif bool(js_buttons[self._reverse_idx]) is True and not self._reverse_activated:
            self._reverse_activated = True

        if self._reverse_activated:
            control.reverse = True

        k2 = 1.6
        throttle_cmd = k2 + (2.05 * math.log10(-0.7 * js_inputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        if throttle_cmd <= 0:
            control.throttle = 0
        elif throttle_cmd > 1:
            control.throttle = 1
        else:
            control.throttle = throttle_cmd

        brake_cmd = 1.6 + (2.05 * math.log10(-0.7 * js_inputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        if brake_cmd <= 0:
            control.brake = 0
        elif brake_cmd > 1:
            control.brake = 1
        else:
            control.brake = brake_cmd

        control.hand_brake = bool(js_buttons[self._handbrake_idx])

        self._actor.apply_control(control)
