#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position

    O            : open/close all doors of vehicle
    T            : toggle vehicle's telemetry

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""


import os
import collections
import datetime
import math
import weakref
import numpy as np
import pygame
import carla
from typing import Dict
from carlacr.helper.config import EgoViewParams
from carlacr.visualization.common import FadingText, HelpText, get_actor_display_name


class HUD3D:
    """Head-up display for 3D visualization."""

    def __init__(self, world: carla.World, config: EgoViewParams = EgoViewParams()):
        """
        Initialization of 3D Head-up display.

        :param config: Ego view parameters.
        """
        self.dim = (config.width, config.height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (config.width, 40), (0, config.height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), config.width, config.height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()
        self._config = config
        self._world = world
        self._map = world.get_map()

    def on_world_tick(self, timestamp: carla.WorldSnapshot.timestamp):
        """
        Callback for CARLA world tick.

        :param timestamp: CARLA world snapshot timestamp.
        """
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world: 'World3D', clock: pygame.time.Clock):
        """
        Update of visualized information.

        :param world: Ego view 3D visualization world object.
        :param clock: Pygame clock.
        """
        self._notifications.tick(clock)
        if not self._show_info:
            return
        t = world.ego_vehicle.get_transform()
        v = world.ego_vehicle.get_velocity()
        c = world.ego_vehicle.get_control()
        compass = world.imu_sensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(collision)
        max_col = max(1.0, max_col)
        collision = [x / max_col for x in collision]
        vehicles = self._world.get_actors().filter(self._config.object_filter)
        map_name = self._map.name.split('/')[-1]
        self._info_text = [f'Server:  {self.server_fps:.2f} FPS',
                           f'Client:  {clock.get_fps():.2f} FPS',
                           '',
                           f'Vehicle: {get_actor_display_name(world.ego_vehicle, truncate=20)}',
                           f'Map:     {map_name}',
                           f'Simulation time: {datetime.timedelta(seconds=int(self.simulation_time))}',
                           '',
                           f'Speed:   {(3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)):.2f} km/h',
                           f'Compass: {compass:.2f}\N{DEGREE SIGN} {heading}',
                           f'Accelero: ({world.imu_sensor.accelerometer[0]:.2f}, '
                           f'{world.imu_sensor.accelerometer[1]:.2f}, {world.imu_sensor.accelerometer[2]:.2f})',
                           f'Gyroscop: ({world.imu_sensor.gyroscope[0]:.2f}, {world.imu_sensor.gyroscope[1]:.2f}, '
                           f'{world.imu_sensor.gyroscope[2]:.2f})',
                           f'Location: {t.location.x:.2f} {t.location.y:.2f}',
                           f'GNSS: {world.gnss_sensor.lat:.2f} {world.gnss_sensor.lon:.2f}',
                           f'Height:  {t.location.z:.2f} m',
                           '']
        if isinstance(c, carla.VehicleControl):
            gear = {-1: 'R', 0: 'N'}.get(c.gear, c.gear)
            self._info_text += [('Throttle:', c.throttle, 0.0, 1.0), ('Steer:', c.steer, -1.0, 1.0),
                                ('Brake:', c.brake, 0.0, 1.0), ('Reverse:', c.reverse), ('Hand brake:', c.hand_brake),
                                ('Manual:', c.manual_gear_shift), f'Gear:        {gear}']
        self._info_text += ['', 'Collision:', collision, '', f"Number of vehicles: {len(vehicles)}"]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']

            def distance(location: carla.Location):
                return math.sqrt((location.x - t.location.x) ** 2 +
                                 (location.y - t.location.y) ** 2 + (location.z - t.location.z) ** 2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.ego_vehicle.id]
            for d, vehicle in sorted(vehicles, key=lambda vehicles: vehicles[0]):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append(f"{d:.2f} {vehicle_type}")

    def notification(self, text: str, seconds: float = 2.0):
        """
        Sets notification text.

        :param text: Text which should be displayed.
        :param seconds: Time how long text is shown.
        """
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text: str):
        """
        Sets notification text in red as error message.

        :param text: Text which should be displayed.
        """
        self._notifications.set_text(f"Error: {text}", (255, 0, 0))

    def render(self, display: pygame.display):
        """
        Renders head-up display.

        :param display: CARLA 3D
        """
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


class CollisionSensor:
    """Manages GNSS sensor attached to ego vehicle."""

    def __init__(self, parent_actor: carla.Vehicle, hud: HUD3D):
        """
        Initialization of collision sensor.

        :param parent_actor: Parent CARLA actor.
        :param hud: Head-up display object.
        """
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self) -> Dict[int, float]:
        """
        Extracts collision intensity for each frame.

        :return: Mapping of frame ID to collision intensity.
        """
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self: weakref, event: carla.CollisionEvent):
        """
        Call back function to extract collision data.

        :param weak_self: Weak self-reference.
        :param event: CARLA CollisionEvent measurement.
        """
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification(f"Collision with {actor_type}")
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


class LaneInvasionSensor:
    """Manages lane invasion sensor attached to ego vehicle."""

    def __init__(self, parent_actor: carla.Vehicle, hud: HUD3D):
        """
        Initialization of lane invasion sensor.

        :param parent_actor: Parent CARLA actor.
        :param hud: Head-up display object.
        """
        self.sensor = None

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor
            self.hud = hud
            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid circular
            # reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self: weakref, event: carla.LaneInvasionSensor):
        """
        Call back function to extract GNSS data.

        :param weak_self: Weak self-reference.
        :param event: CARLA LaneInvasionSensor measurement.
        """
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = [f"{str(x).split()[-1]}" for x in lane_types]
        self.hud.notification(f"Crossed line and {text}")


class GnssSensor:
    """Manages GNSS sensor attached to ego vehicle."""

    def __init__(self, parent_actor: carla.Vehicle):
        """
        Initialization of gnss sensor.

        :param parent_actor: Parent CARLA actor.
        """
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self: weakref, event: carla.GnssMeasurement):
        """
        Call back function to extract GNSS data.

        :param weak_self: Weak self-reference.
        :param event: CARLA GNSS measurement.
        """
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


class IMUSensor:
    """Manages IMU sensor attached to ego vehicle."""

    def __init__(self, parent_actor: carla.Vehicle):
        """
        Initialization of IMU sensor.

        :param parent_actor: Parent CARLA actor.
        """
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda sensor_data: IMUSensor._imu_callback(weak_self, sensor_data))

    @staticmethod
    def _imu_callback(weak_self: weakref, sensor_data: carla.IMUMeasurement):
        """
        Call back function to extract IMU data.

        :param weak_self: Weak self-reference.
        :param sensor_data: CARLA IMU measurement.
        """
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
                              max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
                              max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
                          max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
                          max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


class CameraManager:
    """Manages camera sensor attached to ego vehicle."""

    def __init__(self, parent_actor: carla.Vehicle, hud: HUD3D, gamma_correction: float, path: str):
        """
        Initialization of camera manager.

        :param parent_actor: Parent CARLA actor.
        :param hud: Head-up display object.
        :param gamma_correction: Gamma correction value for camera lens.
        :param path: Path where video should be stored.
        """
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self._hud = hud
        self.recording = False
        self.path = path
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z

        if not self._parent.type_id.startswith("walker.pedestrian"):
            self._camera_transforms = [(
                carla.Transform(carla.Location(x=-2.0 * bound_x, y=+0.0 * bound_y, z=2.0 * bound_z),
                                carla.Rotation(pitch=8.0)), carla.AttachmentType.SpringArm),
                (carla.Transform(carla.Location(x=+0.8 * bound_x, y=+0.0 * bound_y, z=1.3 * bound_z)),
                 carla.AttachmentType.Rigid),
                (carla.Transform(carla.Location(x=+1.9 * bound_x, y=+1.0 * bound_y, z=1.2 * bound_z)),
                 carla.AttachmentType.SpringArm),
                (carla.Transform(carla.Location(x=-2.8 * bound_x, y=+0.0 * bound_y, z=4.6 * bound_z),
                                 carla.Rotation(pitch=6.0)), carla.AttachmentType.SpringArm),
                (carla.Transform(carla.Location(x=-1.0, y=-1.0 * bound_y, z=0.4 * bound_z)),
                 carla.AttachmentType.Rigid)]
        else:
            self._camera_transforms = [(carla.Transform(carla.Location(x=-2.5, z=0.0), carla.Rotation(pitch=-8.0)),
                                        carla.AttachmentType.SpringArm),
                                       (carla.Transform(carla.Location(x=1.6, z=1.7)), carla.AttachmentType.Rigid),
                                       (carla.Transform(carla.Location(x=2.5, y=0.5, z=0.0),
                                                        carla.Rotation(pitch=-8.0)),
                                        carla.AttachmentType.SpringArm),
                                       (carla.Transform(carla.Location(x=-4.0, z=2.0),
                                                        carla.Rotation(pitch=6.0)), carla.AttachmentType.SpringArm),
                                       (carla.Transform(carla.Location(x=0, y=-2.5, z=-0.0), carla.Rotation(yaw=90.0)),
                                        carla.AttachmentType.Rigid)]

        self.transform_index = 1
        self.sensors = [['sensor.camera.rgb', carla.ColorConverter.Raw, 'Camera RGB', {}],
                        ['sensor.camera.dvs', carla.ColorConverter.Raw, 'Dynamic Vision Sensor', {}],
                        ['sensor.camera.rgb', carla.ColorConverter.Raw, 'Camera RGB Distorted',
                         {'lens_circle_multiplier': '3.0', 'lens_circle_falloff': '3.0',
                          'chromatic_aberration_intensity': '0.5', 'chromatic_aberration_offset': '0'}], ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            item.append(bp)
        self.index = None

    def set_sensor(self, index: int, notify: bool = True, force_respawn: bool = False):
        """
        Sets camera sensor.

        :param index: Index of sensor.
        :param notify: Boolean whether notification on HUD should be shown.
        :param force_respawn: Boolean indicating whether the sensor has to be respawned.
        """
        index = index % len(self.sensors)
        needs_respawn = \
            True if self.index is None else (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = \
                self._parent.get_world().spawn_actor(self.sensors[index][-1],
                                                     self._camera_transforms[self.transform_index][0],
                                                     attach_to=self._parent,
                                                     attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self._hud.notification(self.sensors[index][2])
        self.index = index

    def toggle_recording(self):
        """Activates/deactivates camera recording."""
        self.recording = not self.recording

    def render(self, display: pygame.display):
        """
        Renders camera.

        :param display: Pygame display used for rendering.
        """
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self: weakref, image: carla.Image):
        """
        Parses camera image and stores it if recording is activated.

        :param weak_self: Weak self-reference.
        :param image: CARLA image.
        """
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = \
                np.frombuffer(image.raw_data,
                              dtype=np.dtype([('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk(f'{self.path}/_tmp/%08d' % image.frame)


class World3D:
    """
    Class for 3D ego view visualization that contains all the
    information of a CARLA world that is running on the server side
    """

    def __init__(self, carla_world: carla.World, hud: HUD3D, config: EgoViewParams, ego_vehicle: carla.Vehicle):
        """
        Initialization of 3D world visualization.

        :param carla_world: CARLA world.
        :param hud: 3D Head-up display.
        :param config: Simulation parameters.
        :param ego_vehicle: Ego vehicle actor.
        """
        self._world = carla_world
        self._config = config
        self._hud = hud
        self._ego_vehicle = ego_vehicle
        self._collision_sensor = None
        self._lane_invasion_sensor = None
        self._gnss_sensor = None
        self._imu_sensor = None
        self._camera_manager = None
        self._restart()
        self._world.on_tick(hud.on_world_tick)
        # self.doors_are_open = False

    @property
    def ego_vehicle(self) -> carla.Vehicle:
        """
        Getter for ego vehicle

        :return: CARLA ego vehicle.
        """
        return self._ego_vehicle

    @property
    def imu_sensor(self) -> IMUSensor:
        """
        Getter for ego vehicle IMU sensor.

        :return: IMUSensor object.
        """
        return self._imu_sensor

    @property
    def lane_invasion_sensor(self) -> LaneInvasionSensor:
        """
        Getter for ego vehicle's lane invasion sensor.

        :return: Lane invasion sensor object.
        """
        return self._lane_invasion_sensor

    @property
    def gnss_sensor(self) -> GnssSensor:
        """
        Getter for ego vehicle's GNSS sensor

        :return: GNSS sensor object.
        """
        return self._gnss_sensor

    @property
    def collision_sensor(self) -> CollisionSensor:
        """
        Getter for ego vehicle's collision sensor.

        :return: Collision sensor object.
        """
        return self._collision_sensor

    def _restart(self):
        """Starts world objects."""
        # Keep same camera config if the camera manager exists.
        cam_index = self._camera_manager.index if self._camera_manager is not None else 0
        cam_pos_index = self._camera_manager.transform_index if self._camera_manager is not None else 0

        # Set up the sensors.
        self._collision_sensor = CollisionSensor(self._ego_vehicle, self._hud)
        self._lane_invasion_sensor = LaneInvasionSensor(self._ego_vehicle, self._hud)
        self._gnss_sensor = GnssSensor(self._ego_vehicle)
        self._imu_sensor = IMUSensor(self._ego_vehicle)
        self._camera_manager = CameraManager(self._ego_vehicle, self._hud, self._config.gamma, self._config.video_path)
        if self._config.record_video:
            self._camera_manager.toggle_recording()
        self._camera_manager.transform_index = cam_pos_index
        self._camera_manager.set_sensor(cam_index, notify=False)
        self._hud.notification(get_actor_display_name(self._ego_vehicle))
        self._hud.notification("Press 'H' or '?' for help.", seconds=4.0)

        if self._config.sync:
            self._world.tick()
        else:
            self._world.wait_for_tick()

    def tick(self, clock: pygame.time.Clock):
        """
        Performs tick of HUD.

        :param clock: Pygame clock.
        """
        self._hud.tick(self, clock)

    def render(self, display: pygame.display):
        """
        Calls renderer of camera manager and HUD.

        :param display: Pygame display for rendering.
        """
        self._camera_manager.render(display)
        if self._config.vis_hud:
            self._hud.render(display)

    def destroy_sensors(self):
        """Destroys camera sensor."""
        self._camera_manager.sensor.destroy()
        self._camera_manager.sensor = None
        self._camera_manager.index = None

    def destroy(self):
        """Destroys sensors and ego vehicle."""
        sensors = [self._camera_manager.sensor, self._collision_sensor.sensor, self._lane_invasion_sensor.sensor,
                   self._gnss_sensor.sensor, self._imu_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self._ego_vehicle is not None:
            self._ego_vehicle.destroy()
