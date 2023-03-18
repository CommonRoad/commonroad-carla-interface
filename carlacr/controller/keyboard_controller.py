#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================

from typing import Union, Optional
import sys
import carla

from carlacr.visualization.ego_view import HUD3D, World3D
from carlacr.visualization.birds_eye_view import HUD2D, World2D
from carlacr.controller.controller import CarlaController

from commonroad.scenario.state import TraceState


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
  #  from pygame.locals import K_ESCAPE

    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
 #   from pygame.locals import K_b
    from pygame.locals import K_l
 #   from pygame.locals import K_c
 #   from pygame.locals import K_BACKQUOTE
 #   from pygame.locals import K_BACKSPACE
 #   from pygame.locals import K_F1
 #   from pygame.locals import K_v
 #   from pygame.locals import K_h
 #   from pygame.locals import K_g
 #   from pygame.locals import K_n
 #   from pygame.locals import K_o
 #   from pygame.locals import K_0
 #   from pygame.locals import K_9
 #   from pygame.locals import K_SLASH
 #   from pygame.locals import K_r
    from pygame.locals import K_x
    from pygame.locals import K_z
    from pygame.locals import K_i
 #   from pygame.locals import K_MINUS
 #   from pygame.locals import K_EQUALS
 #   from pygame.locals import K_t

except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

MAP_DEFAULT_SCALE = 0.1


class KeyboardControllerInterface(CarlaController):
    """Class that handles input received such as keyboard and mouse."""

    def __init__(self, actor: carla.Actor):
        """Initializes input member variables when instance is created."""
        super().__init__(actor)
        self._clock = None
        self._hud = None
        self._vis_world = None

    def control(self, state: Optional[TraceState] = None):
        """Executed each frame. Calls method for parsing input."""
        pass

    def register(self, clock: pygame.time.Clock, hud: Union[HUD2D, HUD3D], vis_world: Union[World2D, World3D]):
        self._clock = clock
        self._hud = hud
        self._vis_world = vis_world

    def _parse_events(self):
        pass
       # if isinstance(self._control, carla.VehicleControl):
       #     current_lights = self._lights
    #    for event in pygame.event.get():
            # if event.type == pygame.QUIT:
            #     return True
      #      if event.type == pygame.KEYUP:
            #     if is_quit_shortcut(event.key):
            #         return True
            #     elif event.key == K_BACKSPACE:
            #         if self._autopilot_enabled:
            #             world.player.set_autopilot(False)
            #             world.restart()
            #             world.player.set_autopilot(True)
            #         else:
            #             world.restart()
                # elif event.key == K_F1:
                #     world.hud.toggle_info()
                # elif event.key == K_v and pygame.key.get_mods() & KMOD_SHIFT:
                #     world.next_map_layer(reverse=True)
                # elif event.key == K_v:
                #     world.next_map_layer()
                # elif event.key == K_b and pygame.key.get_mods() & KMOD_SHIFT:
                #     world.load_map_layer(unload=True)
                # elif event.key == K_b:
                #     world.load_map_layer()
                # elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                #     world.hud.help.toggle()
                # elif event.key == K_TAB:
                #     world.camera_manager.toggle_camera()
                # elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                #     world.next_weather(reverse=True)
                # elif event.key == K_c:
                #     world.next_weather()
                # elif event.key == K_g:
                #     world.toggle_radar()
                # elif event.key == K_BACKQUOTE:
                #     world.camera_manager.next_sensor()
                # elif event.key == K_n:
                #     world.camera_manager.next_sensor()
                # elif event.key == K_w and (pygame.key.get_mods() & KMOD_CTRL):
                #     if world.constant_velocity_enabled:
                #         world.player.disable_constant_velocity()
                #         world.constant_velocity_enabled = False
                #         world.hud.notification("Disabled Constant Velocity Mode")
                #     else:
                #         world.player.enable_constant_velocity(carla.Vector3D(17, 0, 0))
                #         world.constant_velocity_enabled = True
                #         world.hud.notification("Enabled Constant Velocity Mode at 60 km/h")
                # elif event.key == K_o:
                #     try:
                #         if world.doors_are_open:
                #             world.hud.notification("Closing Doors")
                #             world.doors_are_open = False
                #             world.player.close_door(carla.VehicleDoor.All)
                #         else:
                #             world.hud.notification("Opening doors")
                #             world.doors_are_open = True
                #             world.player.open_door(carla.VehicleDoor.All)
                #     except Exception:
                #         pass
                # elif event.key == K_t:
                #     if world.show_vehicle_telemetry:
                #         world.player.show_debug_telemetry(False)
                #         world.show_vehicle_telemetry = False
                #         world.hud.notification("Disabled Vehicle Telemetry")
                #     else:
                #         try:
                #             world.player.show_debug_telemetry(True)
                #             world.show_vehicle_telemetry = True
                #             world.hud.notification("Enabled Vehicle Telemetry")
                #         except Exception:
                #             pass
                # elif event.key > K_0 and event.key <= K_9:
                #     index_ctrl = 0
                #     if pygame.key.get_mods() & KMOD_CTRL:
                #         index_ctrl = 9
                #     world.camera_manager.set_sensor(event.key - 1 - K_0 + index_ctrl)
                # elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                #     world.camera_manager.toggle_recording()
                # elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                #     if (world.recording_enabled):
                #         client.stop_recorder()
                #         world.recording_enabled = False
                #         world.hud.notification("Recorder is OFF")
                #     else:
                #         client.start_recorder("manual_recording.rec")
                #         world.recording_enabled = True
                #         world.hud.notification("Recorder is ON")
                # elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                #     # stop recorder
                #     client.stop_recorder()
                #     world.recording_enabled = False
                #     # work around to fix camera at start of replaying
                #     current_index = world.camera_manager.index
                #     world.destroy_sensors()
                #     # disable autopilot
                #     self._autopilot_enabled = False
                #     world.player.set_autopilot(self._autopilot_enabled)
                #     world.hud.notification("Replaying file 'manual_recording.rec'")
                #     # replayer
                #     client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                #     world.camera_manager.set_sensor(current_index)
                # elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                #     if pygame.key.get_mods() & KMOD_SHIFT:
                #         world.recording_start -= 10
                #     else:
                #         world.recording_start -= 1
                #     world.hud.notification("Recording start time is %d" % (world.recording_start))
                # elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                #     if pygame.key.get_mods() & KMOD_SHIFT:
                #         world.recording_start += 10
                #     else:
                #         world.recording_start += 1
                #     world.hud.notification("Recording start time is %d" % (world.recording_start))
                # if isinstance(self._control, carla.VehicleControl):
                #     if event.key == K_q:
                #         self._control.gear = 1 if self._control.reverse else -1
                #     elif event.key == K_m:
                #         self._control.manual_gear_shift = not self._control.manual_gear_shift
                #         self._control.gear = world.player.get_control().gear
                #         world.hud.notification('%s Transmission' %
                #                                ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                #     elif self._control.manual_gear_shift and event.key == K_COMMA:
                #         self._control.gear = max(-1, self._control.gear - 1)
                #     elif self._control.manual_gear_shift and event.key == K_PERIOD:
                #         self._control.gear = self._control.gear + 1
                    # elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                    #     if not self._autopilot_enabled and not self._config.sync:
                    #         print("WARNING: You are currently in asynchronous mode and could "
                    #               "experience some issues with the traffic simulation")
                    #     self._autopilot_enabled = not self._autopilot_enabled
                    #     world.player.set_autopilot(self._autopilot_enabled)
                    #     world.hud.notification(
                    #         'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                    # elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                    #     current_lights ^= carla.VehicleLightState.Special1
                    # elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                    #     current_lights ^= carla.VehicleLightState.HighBeam
                    # elif event.key == K_l:
                    #     # Use 'L' key to switch between lights:
                    #     # closed -> position -> low beam -> fog
                    #     if not self._lights & carla.VehicleLightState.Position:
                    #         world.hud.notification("Position lights")
                    #         current_lights |= carla.VehicleLightState.Position
                    #     else:
                    #         world.hud.notification("Low beam lights")
                    #         current_lights |= carla.VehicleLightState.LowBeam
                    #     if self._lights & carla.VehicleLightState.LowBeam:
                    #         world.hud.notification("Fog lights")
                    #         current_lights |= carla.VehicleLightState.Fog
                    #     if self._lights & carla.VehicleLightState.Fog:
                    #         world.hud.notification("Lights off")
                    #         current_lights ^= carla.VehicleLightState.Position
                    #         current_lights ^= carla.VehicleLightState.LowBeam
                    #         current_lights ^= carla.VehicleLightState.Fog
                    # elif event.key == K_i:
                    #     current_lights ^= carla.VehicleLightState.Interior
                    # elif event.key == K_z:
                    #     current_lights ^= carla.VehicleLightState.LeftBlinker
                    # elif event.key == K_x:
                    #     current_lights ^= carla.VehicleLightState.RightBlinker


    def _parse_input(self):
        pass


class KeyboardVehicleController(KeyboardControllerInterface):
    """Class that handles input received such as keyboard and mouse."""

    def __init__(self, actor: carla.Actor):
        """Initializes input member variables when instance is created."""
        super().__init__(actor)
        self._control = carla.VehicleControl()
        self._lights = carla.VehicleLightState.NONE
        self._steer_cache = 0.0

    def control(self, state: Optional[TraceState] = None):
        """Executed each frame. Calls method for parsing input."""
        self._parse_input()

    def _parse_events(self):
        current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            if event.type == pygame.KEYUP:
            #     if is_quit_shortcut(event.key):
            #         return True
            #     elif event.key == K_BACKSPACE:
            #         if self._autopilot_enabled:
            #             world.player.set_autopilot(False)
            #             world.restart()
            #             world.player.set_autopilot(True)
            #         else:
            #             world.restart()
                # elif event.key == K_F1:
                #     world.hud.toggle_info()
                # elif event.key == K_v and pygame.key.get_mods() & KMOD_SHIFT:
                #     world.next_map_layer(reverse=True)
                # elif event.key == K_v:
                #     world.next_map_layer()
                # elif event.key == K_b and pygame.key.get_mods() & KMOD_SHIFT:
                #     world.load_map_layer(unload=True)
                # elif event.key == K_b:
                #     world.load_map_layer()
                # elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                #     world.hud.help.toggle()
                # elif event.key == K_TAB:
                #     world.camera_manager.toggle_camera()
                # elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                #     world.next_weather(reverse=True)
                # elif event.key == K_c:
                #     world.next_weather()
                # elif event.key == K_g:
                #     world.toggle_radar()
                # elif event.key == K_BACKQUOTE:
                #     world.camera_manager.next_sensor()
                # elif event.key == K_n:
                #     world.camera_manager.next_sensor()
                # elif event.key == K_w and (pygame.key.get_mods() & KMOD_CTRL):
                #     if world.constant_velocity_enabled:
                #         world.player.disable_constant_velocity()
                #         world.constant_velocity_enabled = False
                #         world.hud.notification("Disabled Constant Velocity Mode")
                #     else:
                #         world.player.enable_constant_velocity(carla.Vector3D(17, 0, 0))
                #         world.constant_velocity_enabled = True
                #         world.hud.notification("Enabled Constant Velocity Mode at 60 km/h")
                # elif event.key == K_o:
                #     try:
                #         if world.doors_are_open:
                #             world.hud.notification("Closing Doors")
                #             world.doors_are_open = False
                #             world.player.close_door(carla.VehicleDoor.All)
                #         else:
                #             world.hud.notification("Opening doors")
                #             world.doors_are_open = True
                #             world.player.open_door(carla.VehicleDoor.All)
                #     except Exception:
                #         pass
                # elif event.key == K_t:
                #     if world.show_vehicle_telemetry:
                #         world.player.show_debug_telemetry(False)
                #         world.show_vehicle_telemetry = False
                #         world.hud.notification("Disabled Vehicle Telemetry")
                #     else:
                #         try:
                #             world.player.show_debug_telemetry(True)
                #             world.show_vehicle_telemetry = True
                #             world.hud.notification("Enabled Vehicle Telemetry")
                #         except Exception:
                #             pass
                # elif event.key > K_0 and event.key <= K_9:
                #     index_ctrl = 0
                #     if pygame.key.get_mods() & KMOD_CTRL:
                #         index_ctrl = 9
                #     world.camera_manager.set_sensor(event.key - 1 - K_0 + index_ctrl)
                # elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                #     world.camera_manager.toggle_recording()
                # elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                #     if (world.recording_enabled):
                #         client.stop_recorder()
                #         world.recording_enabled = False
                #         world.hud.notification("Recorder is OFF")
                #     else:
                #         client.start_recorder("manual_recording.rec")
                #         world.recording_enabled = True
                #         world.hud.notification("Recorder is ON")
                # elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                #     # stop recorder
                #     client.stop_recorder()
                #     world.recording_enabled = False
                #     # work around to fix camera at start of replaying
                #     current_index = world.camera_manager.index
                #     world.destroy_sensors()
                #     # disable autopilot
                #     self._autopilot_enabled = False
                #     world.player.set_autopilot(self._autopilot_enabled)
                #     world.hud.notification("Replaying file 'manual_recording.rec'")
                #     # replayer
                #     client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                #     world.camera_manager.set_sensor(current_index)
                # elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                #     if pygame.key.get_mods() & KMOD_SHIFT:
                #         world.recording_start -= 10
                #     else:
                #         world.recording_start -= 1
                #     world.hud.notification("Recording start time is %d" % (world.recording_start))
                # elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                #     if pygame.key.get_mods() & KMOD_SHIFT:
                #         world.recording_start += 10
                #     else:
                #         world.recording_start += 1
                #     world.hud.notification("Recording start time is %d" % (world.recording_start))
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
                    # elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                    #     if not self._autopilot_enabled and not self._config.sync:
                    #         print("WARNING: You are currently in asynchronous mode and could "
                    #               "experience some issues with the traffic simulation")
                    #     self._autopilot_enabled = not self._autopilot_enabled
                    #     world.player.set_autopilot(self._autopilot_enabled)
                    #     world.hud.notification(
                    #         'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                    # elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                    #     current_lights ^= carla.VehicleLightState.Special1
                    # elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                    #     current_lights ^= carla.VehicleLightState.HighBeam
                    # elif event.key == K_l:
                    #     # Use 'L' key to switch between lights:
                    #     # closed -> position -> low beam -> fog
                    #     if not self._lights & carla.VehicleLightState.Position:
                    #         world.hud.notification("Position lights")
                    #         current_lights |= carla.VehicleLightState.Position
                    #     else:
                    #         world.hud.notification("Low beam lights")
                    #         current_lights |= carla.VehicleLightState.LowBeam
                    #     if self._lights & carla.VehicleLightState.LowBeam:
                    #         world.hud.notification("Fog lights")
                    #         current_lights |= carla.VehicleLightState.Fog
                    #     if self._lights & carla.VehicleLightState.Fog:
                    #         world.hud.notification("Lights off")
                    #         current_lights ^= carla.VehicleLightState.Position
                    #         current_lights ^= carla.VehicleLightState.LowBeam
                    #         current_lights ^= carla.VehicleLightState.Fog
                    # elif event.key == K_i:
                    #     current_lights ^= carla.VehicleLightState.Interior
                    # elif event.key == K_z:
                    #     current_lights ^= carla.VehicleLightState.LeftBlinker
                    # elif event.key == K_x:
                    #     current_lights ^= carla.VehicleLightState.RightBlinker

    def _parse_vehicle_keys(self):
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
            self._control.reverse = self._control.gear < 0
            # Set automatic control-related vehicle lights
            # current_lights = self._lights
            # if self._control.brake:
            #     current_lights |= carla.VehicleLightState.Brake
            # else: # Remove the Brake flag
            #     current_lights &= ~carla.VehicleLightState.Brake
            # if self._control.reverse:
            #     current_lights |= carla.VehicleLightState.Reverse
            # else: # Remove the Reverse flag
            #     current_lights &= ~carla.VehicleLightState.Reverse
            # if current_lights != self._lights: # Change the light state only if necessary
            #     self._lights = current_lights
            #     world.player.set_light_state(carla.VehicleLightState(self._lights))
            self._actor.apply_control(self._control)

class KeyboardWalkerController(KeyboardControllerInterface):
    """Class that handles input received such as keyboard and mouse."""

    def __init__(self, actor: carla.Actor):
        """Initializes input member variables when instance is created."""
        super().__init__(actor)
        self._control = carla.WalkerControl()
        self._rotation = 0.0
        self._player_max_speed_fast = 0.0
        self._player_max_speed = 0.0

    def control(self, state: Optional[TraceState] = None):
        """Executed each frame. Calls method for parsing input."""
        self._parse_input()

    def _parse_walker_keys(self):
        milliseconds = self._clock.get_time()
        keys = pygame.key.get_pressed()
        self._rotation = self._actor.rotation
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = self._player_max_speed_fast if pygame.key.get_mods() & KMOD_SHIFT \
                else self._player_max_speed
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    def _parse_input(self):
        """Parses the input, which is classified in keyboard events and mouse"""
        self._parse_events()
        if not self._autopilot_enabled:
            self._parse_walker_keys()
            self._actor.apply_control(self._control)
