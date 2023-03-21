from typing import Optional
import carla

from carlacr.visualization.ego_view import HUD3D, World3D
from carlacr.visualization.birds_eye_view import HUD2D, World2D, is_quit_shortcut
from carlacr.controller.controller import CarlaController

from commonroad.scenario.state import TraceState

import pygame
from pygame.locals import K_COMMA  # pylint: disable= no-name-in-module
from pygame.locals import K_DOWN  # pylint: disable= no-name-in-module
from pygame.locals import K_LEFT  # pylint: disable= no-name-in-module
from pygame.locals import K_PERIOD  # pylint: disable= no-name-in-module
from pygame.locals import K_RIGHT  # pylint: disable= no-name-in-module
from pygame.locals import K_SPACE  # pylint: disable= no-name-in-module
from pygame.locals import K_UP  # pylint: disable= no-name-in-module
from pygame.locals import K_a  # pylint: disable= no-name-in-module
from pygame.locals import K_d  # pylint: disable= no-name-in-module
from pygame.locals import K_m  # pylint: disable= no-name-in-module
from pygame.locals import K_q  # pylint: disable= no-name-in-module
from pygame.locals import K_s  # pylint: disable= no-name-in-module
from pygame.locals import K_w  # pylint: disable= no-name-in-module

from pygame.locals import KMOD_CTRL  # pylint: disable= no-name-in-module
from pygame.locals import KMOD_SHIFT  # pylint: disable= no-name-in-module
from pygame.locals import K_l  # pylint: disable= no-name-in-module
from pygame.locals import K_x  # pylint: disable= no-name-in-module
from pygame.locals import K_z  # pylint: disable= no-name-in-module
from pygame.locals import K_i  # pylint: disable= no-name-in-module


class KeyboardVehicleController(CarlaController):
    """Class that handles input received such as keyboard and mouse."""

    def __init__(self, actor: carla.Actor):
        """
        Initializes input member variables when instance is created.

        :param actor:
        """
        super().__init__(actor)
        self._clock = None
        self._hud = None
        self._vis_world = None
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
        """

        :return:
        """
        current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            if event.type == pygame.KEYUP:
                if is_quit_shortcut(event.key):
                    return True
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = self._vis_world.player.get_control().gear
                        self._hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                        current_lights ^= carla.VehicleLightState.Special1
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                        current_lights ^= carla.VehicleLightState.HighBeam
                    elif event.key == K_l:
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
                    elif event.key == K_i:
                        current_lights ^= carla.VehicleLightState.Interior
                    elif event.key == K_z:
                        current_lights ^= carla.VehicleLightState.LeftBlinker
                    elif event.key == K_x:
                        current_lights ^= carla.VehicleLightState.RightBlinker

    def _parse_vehicle_keys(self):
        """Parses control-related key inputs (steering, acceleration)."""
        keys = pygame.key.get_pressed()
        self._control.throttle = min(self._control.throttle + 0.01, 1.00) if keys[K_UP] or keys[K_w] else 0.0
         #   self.control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * self._clock.get_time()
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = min(self._control.brake + 0.2, 1) if keys[K_DOWN] or keys[K_s] else 0.0
        # self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    def _parse_input(self):
        """Parses the input, which is classified in keyboard events and mouse"""
        self._parse_events()
        if not self._autopilot_enabled:
            self._parse_vehicle_keys()
            self._actor.apply_control(self._control)
