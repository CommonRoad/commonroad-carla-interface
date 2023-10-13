from typing import TYPE_CHECKING, List, Tuple, Dict

import carla
import pygame
import shapely

from carlacr.visualization.visualization_base import VisualizationBase

if TYPE_CHECKING:
    from carlacr.visualization.visualization3D import Visualization3D


class LineToVehicle(VisualizationBase):
    """
    A class to visualize bounding boxes of objects in a 3D world.
    """

    def __init__(self, vis3d: 'Visualization3D', z_axis: float = 1) -> None:
        """
        Initializes an instance of BoundingBox3D.

        :param vis3D: A reference to the 3D visualization instance.
        :type vis3D: Visualization3D
        :param z_axis: The z-coordinate of the visualization reference point.
        :type z_axis: float
        """
        super().__init__(z_axis)

        self._vis3d = vis3d  # The 3D visualization instance

        self._targets: Dict[int, dict] = {}
        self._lines: List[dict] = []
        self._min_distance = 1.2

    def set_connection(self, vehicle: carla.Vehicle, max_dist, color: Tuple[int, int, int] = (255, 0, 0)):
        """
        Enables the display of vehicles.

        :param vehicle: The vehicle to visualize.
        :type vehicle: carla.Vehicle
        :param max_dist: Maximum distance to display vehicles.
        :type max_dist: float
        :param color: Color of the bounding boxes (RGB format).
        :type color: Tuple[int, int, int]
        """
        target = self._targets.setdefault(vehicle.id, {})
        target['vehicle'] = vehicle
        target['color'] = color
        target['max_dist'] = max_dist

    def remove_connection(self, vehicle: carla.Vehicle):
        """
        Disables the display of vehicles.

        :param vehicle: The vehicle to remove.
        :type vehicle: carla.Vehicle
        """
        if vehicle.id in self._targets:
            del self._targets[vehicle.id]

    def tick(self, clock: pygame.time.Clock):
        """
        Called to update the current position of the bounding boxes.

        :param clock: The game clock.
        :type clock: pygame.time.Clock
        """
        super().tick(clock)
        if not VisualizationBase.is_visible:
            return
        ego_vehicle = self._vis3d.ego_vehicle
        ego_location = ego_vehicle.get_location()
        ego_image_point = self._vis3d.vis_tool_controller.get_image_point(ego_location)
        ego_forward_vec = ego_vehicle.get_transform().get_forward_vector()

        for _, target in self._targets.items():
            vehicle: carla.Vehicle = target['vehicle']
            max_dist = target['max_dist']
            color = target['color']
            loc = vehicle.get_location()

            ray = loc - ego_location
            distance = ego_forward_vec.dot(ray)

            if self._min_distance < distance < max_dist:
                self._lines.append(
                    {'start': ego_image_point, 'end': self._vis3d.vis_tool_controller.get_image_point(loc),
                        'color': color, })

    def render(self, display: pygame.display):
        """
        Draws the bounding boxes on the display.

        :param display: The display to draw onto.
        :type display: pygame.display
        """
        super().render(display)
        if not VisualizationBase.is_visible:
            return
        for target in self._lines:
            color = target['color']
            start = target['start']
            end = target['end']

            line = shapely.LineString([start, end])
            pygame_linie = list(line.coords)
            pygame.draw.lines(display, color, False, pygame_linie, 3)

        self._lines.clear()
