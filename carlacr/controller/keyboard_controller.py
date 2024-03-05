"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot (not implemented yet)

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    O            : open/close all doors of vehicle
    T            : toggle vehicle's telemetry

    V            : toogle visualization tools

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from typing import Optional

import carla
import pygame
import pygame.locals as keys
from commonroad.scenario.state import TraceState

from carlacr.controller.controller import CarlaController
from carlacr.visualization.visualization_base import VisualizationBase


class KeyboardVehicleController(CarlaController):
    """Class that handles input received such as keyboard and mouse."""

    def __init__(self, actor: carla.Actor, dt: float = 0.1):
        """
        Initializes input member variables when instance is created.

        :param actor:
        """
        super().__init__(actor)
        self._dt = dt
        self._control = carla.VehicleControl()
        self._lights = carla.VehicleLightState.NONE
        self._steer_cache = 0.0

    def control(self, state: Optional[TraceState] = None):
        """
        Applies keyboard control. Parses input and computes vehicle inputs.

        :param state: CommonRoad state which should be reached at next time step. Not used for keyboard control.
        """
        self._parse_input()

    def _parse_events(self):
        """Parses general events related to vehicle."""
        current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break
            if event.type == pygame.KEYUP:
                if event.key == keys.K_l and pygame.key.get_mods() & keys.KMOD_CTRL:
                    current_lights ^= carla.VehicleLightState.Special1
                elif event.key == keys.K_l and pygame.key.get_mods() & keys.KMOD_SHIFT:
                    current_lights ^= carla.VehicleLightState.HighBeam
                elif event.key == keys.K_l:
                    # Use 'L' key to switch between lights:
                    # closed -> position -> low beam -> fog
                    if not self._lights & carla.VehicleLightState.Position:
                        #  world.hud.notification("Position lights")
                        current_lights |= carla.VehicleLightState.Position
                    else:
                        #  world.hud.notification("Low beam lights")
                        current_lights |= carla.VehicleLightState.LowBeam
                    if self._lights & carla.VehicleLightState.LowBeam:
                        #  world.hud.notification("Fog lights")
                        current_lights |= carla.VehicleLightState.Fog
                    if self._lights & carla.VehicleLightState.Fog:
                        #  world.hud.notification("Lights off")
                        current_lights ^= carla.VehicleLightState.Position
                        current_lights ^= carla.VehicleLightState.LowBeam
                        current_lights ^= carla.VehicleLightState.Fog
                elif event.key == keys.K_i:
                    current_lights ^= carla.VehicleLightState.Interior
                elif event.key == keys.K_z:
                    current_lights ^= carla.VehicleLightState.LeftBlinker
                elif event.key == keys.K_x:
                    current_lights ^= carla.VehicleLightState.RightBlinker
                elif event.key == keys.K_v:
                    VisualizationBase.is_visible = not VisualizationBase.is_visible
                elif event.key == keys.K_h:
                    print(__doc__)

    def _parse_vehicle_keys(self):
        """Parses control-related key inputs (steering, acceleration)."""
        pressed_keys = pygame.key.get_pressed()
        self._control.throttle = (
            min(self._control.throttle + 0.01, 1.00) if pressed_keys[keys.K_UP] or pressed_keys[keys.K_w] else 0.0
        )
        #  self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * self._dt * 1000
        if pressed_keys[keys.K_LEFT] or pressed_keys[keys.K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif pressed_keys[keys.K_RIGHT] or pressed_keys[keys.K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = (
            min(self._control.brake + 0.2, 1) if pressed_keys[keys.K_DOWN] or pressed_keys[keys.K_s] else 0.0
        )
        # self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = pressed_keys[keys.K_SPACE]
        self._control.reverse = pressed_keys[keys.K_q]

    def _parse_input(self):
        """Parses the input, which is classified in keyboard events and mouse"""
        self._parse_events()
        if not self._autopilot_enabled:
            self._parse_vehicle_keys()
            self._actor.apply_control(self._control)
