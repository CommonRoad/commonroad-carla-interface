import datetime
import math
from typing import TYPE_CHECKING

import carla
import pygame

from crcarla.visualization.canvas.ui_elements.fading_text import FadingText
from crcarla.visualization.canvas.ui_elements.text import COLOR_RED, COLOR_WHITE, Text
from crcarla.visualization.common import get_actor_display_name, sort_vehicles_by_dist
from crcarla.visualization.visualization_base import VisualizationBase

if TYPE_CHECKING:
    from crcarla.visualization.visualization3D import (  # from crcarla.visualization.visualization2D import World2D
        Visualization3D,
    )


class CanvasController(VisualizationBase):
    """Mananage the canvas ui elements."""

    def __init__(self, vis3d: "Visualization3D", z_axis: float = 100) -> None:
        """Must have for VisualizationBase inheriting."""
        super().__init__(z_axis)

        self._vis3d = vis3d

        # Create font for all text elements
        self._notification_text = FadingText(
            FadingText.font_mono(), (self._vis3d.config.width, 40), (0, self._vis3d.config.height - 40)
        )
        # self._help = HelpText(Text.font_mono_16(), self._config.width, self._config.height)

        self._info_text = []
        self._show_info = True

    # ==========================================
    # Render pipeline
    # ==========================================

    def tick(self, clock: pygame.time.Clock):
        super().tick(clock)

        if self._show_info:
            self._info_text = self._create_info_text()

    def render(self, display: pygame.surface.Surface):
        super().render(display)
        if self._show_info:
            self._render_info_text(display)

    # ==========================================
    # Public
    # ==========================================

    def notify(self, text: str, t_ms: int = 2000, color: pygame.color.Color = COLOR_WHITE):
        """
        Sets notification text.

        :param text: Text which should be displayed.
        :param seconds: Time how long text is shown.
        """
        self._notification_text.set_text(text, color, t_ms)

    def error(self, text: str):
        """
        Sets notification text in red as error message.

        :param text: Text which should be displayed.
        """
        self.notify(f"Error: {text}", color=COLOR_RED)

    # ==========================================
    # Private
    # ==========================================

    def _create_info_text(self):
        info_text = []

        t = self._vis3d.ego_vehicle.get_transform()
        v = self._vis3d.ego_vehicle.get_velocity()
        c = self._vis3d.ego_vehicle.get_control()
        compass = self._vis3d.sensor_controller.imu_sensor.compass
        heading = "N" if compass > 270.5 or compass < 89.5 else ""
        heading += "S" if 90.5 < compass < 269.5 else ""
        heading += "E" if 0.5 < compass < 179.5 else ""
        heading += "W" if 180.5 < compass < 359.5 else ""
        colhist = self._vis3d.sensor_controller.collision_sensor.get_collision_history()
        collision = [colhist[x + self._vis3d.frame - 200] for x in range(0, 200)]
        max_col = max(collision)
        max_col = max(1.0, max_col)
        collision = [x / max_col for x in collision]
        vehicles: carla.ActorList = self._vis3d.vehicles
        map_name = self._vis3d.carla_world.get_map().name.split("/")[-1]
        info_text = [
            f"Server:  {self._vis3d.server_fps:.2f} FPS",
            f"Client:  {self._vis3d.client_fps:.2f} FPS",
            "",
            f"Vehicle: {get_actor_display_name(self._vis3d.ego_vehicle, truncate=20)}",
            f"Map:     {map_name}",
            f"Simulation time: {datetime.timedelta(seconds=int(self._vis3d.simulation_time))}",
            "",
            f"Speed:   {(3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)):.2f} km/h",
            f"Compass: {compass:.2f}\N{DEGREE SIGN} {heading}",
            f"Accelero: ({self._vis3d.sensor_controller.imu_sensor.accelerometer[0]:.2f}, "
            f"{self._vis3d.sensor_controller.imu_sensor.accelerometer[1]:.2f}, "
            f"{self._vis3d.sensor_controller.imu_sensor.accelerometer[2]:.2f})",
            f"Gyroscop: ({self._vis3d.sensor_controller.imu_sensor.gyroscope[0]:.2f}, "
            f"{self._vis3d.sensor_controller.imu_sensor.gyroscope[1]:.2f}, "
            f"{self._vis3d.sensor_controller.imu_sensor.gyroscope[2]:.2f})",
            f"Location: {t.location.x:.2f} {t.location.y:.2f}",
            f"GNSS: {self._vis3d.sensor_controller.gnss_sensor.lat:.2f} "
            f"{self._vis3d.sensor_controller.gnss_sensor.lon:.2f}",
            f"Height:  {t.location.z:.2f} m",
            "",
        ]
        if isinstance(c, carla.VehicleControl):
            gear = {-1: "R", 0: "N"}.get(c.gear, c.gear)
            info_text += [
                ("Throttle:", c.throttle, 0.0, 1.0),
                ("Steer:", c.steer, -1.0, 1.0),
                ("Brake:", c.brake, 0.0, 1.0),
                ("Reverse:", c.reverse),
                ("Hand brake:", c.hand_brake),
                ("Manual:", c.manual_gear_shift),
                f"Gear:        {gear}",
            ]
        info_text += ["", "Collision:", collision, "", f"Number of vehicles: {len(vehicles)}"]

        if len(vehicles) > 1:
            near_vehicles = sort_vehicles_by_dist(vehicles, self._vis3d.ego_vehicle, max_dist=200.0)
            info_text += ["Nearby vehicles:"]

            for vehicle, d in near_vehicles:
                try:
                    vehicle_type = get_actor_display_name(vehicle, truncate=22)
                except BaseException:
                    print("No actor display name ")
                    return info_text
                info_text.append(f"{d:.2f} {vehicle_type}")
        return info_text

    def _render_info_text(self, display: pygame.surface.Surface):
        if self._show_info:
            info_surface = pygame.Surface((220, self._vis3d.config.height))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self._vis3d.config.height:
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
                    surface = Text.font_mono().render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
