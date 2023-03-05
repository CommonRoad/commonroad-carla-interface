#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import carla

try:
    import pygame
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

MAP_DEFAULT_SCALE = 0.1
HERO_DEFAULT_SCALE = 1.0

class KeyBoardControl2D(object):
    """Class that handles input received such as keyboard and mouse."""

    def __init__(self, name):
        """Initializes input member variables when instance is created."""
        self.name = name
        self._steer_cache = 0.0
        self.control = carla.VehicleControl()
        self._autopilot_enabled = False

        # Modules that input will depend on
        self._hud = None
        self._world = None

    def start(self, hud, world):
        """Assigns other initialized modules that input module needs."""
        self._hud = hud
        self._world = world

        self._hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def render(self, display):
        """Does nothing. Input module does not need render anything."""

    def tick(self, clock):
        """Executed each frame. Calls method for parsing input."""
        self.parse_input(clock)

    def _parse_events(self):
        """Parses input events. These events are executed only once when pressing a key."""
        for event in pygame.event.get():
                if event.key == K_TAB:
                    # Toggle between hero and map mode
                    if self._world.hero_actor is None:
                        self._world.select_hero_actor()
                        self._world.wheel_offset = HERO_DEFAULT_SCALE
                        self.control = carla.VehicleControl()
                        self._hud.notification('Hero Mode')
                    else:
                        self._world.wheel_offset = MAP_DEFAULT_SCALE
                        self._world.mouse_offset = [0, 0]
                        self._world.mouse_pos = [0, 0]
                        self._world.scale_offset = [0, 0]
                        self._world.hero_actor = None
                        self._hud.notification('Map Mode')
                if isinstance(self.control, carla.VehicleControl):
                    if event.key == K_q:
                        self.control.gear = 1 if self.control.reverse else -1
                    elif event.key == K_m:
                        self.control.manual_gear_shift = not self.control.manual_gear_shift
                        self.control.gear = self._world.hero_actor.get_control().gear
                        self._hud.notification('%s Transmission' % (
                            'Manual' if self.control.manual_gear_shift else 'Automatic'))
                    elif self.control.manual_gear_shift and event.key == K_COMMA:
                        self.control.gear = max(-1, self.control.gear - 1)
                    elif self.control.manual_gear_shift and event.key == K_PERIOD:
                        self.control.gear = self.control.gear + 1
                    elif event.key == K_p:
                        # Toggle autopilot
                        if self._world.hero_actor is not None:
                            self._autopilot_enabled = not self._autopilot_enabled
                            self._world.hero_actor.set_autopilot(self._autopilot_enabled)
                            self._hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))

    def _parse_keys(self, milliseconds):
        """Parses keyboard input when keys are pressed"""
        keys = pygame.key.get_pressed()
        self.control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self.control.steer = round(self._steer_cache, 1)
        self.control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self.control.hand_brake = keys[K_SPACE]

    def parse_input(self, clock):
        """Parses the input, which is classified in keyboard events and mouse"""
        self._parse_events()
        if not self._autopilot_enabled:
            if isinstance(self.control, carla.VehicleControl):
                self._parse_keys(clock.get_time())
                self.control.reverse = self.control.gear < 0
            if (self._world.hero_actor is not None):
                self._world.hero_actor.apply_control(self.control)
