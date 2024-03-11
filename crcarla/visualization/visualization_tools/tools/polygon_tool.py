from typing import TYPE_CHECKING, Dict, Tuple

import carla
import pygame
import shapely

from crcarla.visualization.visualization_base import VisualizationBase

if TYPE_CHECKING:
    from crcarla.visualization.visualization3D import Visualization3D


class PolygonTool(VisualizationBase):
    """
    A class to visualize bounding boxes of objects in a 3D world.
    """

    def __init__(self, vis3d: "Visualization3D", z_axis: float = 1) -> None:
        """
        Initializes an instance of BoundingBox3D.

        :param vis3D: A reference to the 3D visualization instance.
        :type vis3D: Visualization3D
        :param z_axis: The z-coordinate of the visualization reference point.
        :type z_axis: float
        """
        super().__init__(z_axis)

        self._vis3d = vis3d  # The 3D visualization instance

        self._polygons: Dict[int, dict] = {}
        self._id_counter = 1
        self._min_distance = 1.2

    def set_arrow(self, vehicle: carla.Vehicle, max_dist=200, size=40, color: Tuple[int, int, int] = (255, 0, 0)):
        """
        Enables the display of vehicles as arrows.

        :param vehicle: The vehicle to visualize.
        :type vehicle: carla.Vehicle
        :param max_dist: Maximum distance to display vehicles.
        :type max_dist: float
        :param size: Size of the arrow.
        :type size: int
        :param color: Color of the arrow (RGB format).
        :type color: Tuple[int, int, int]
        """
        verts = [[0, 0], [2, 2], [1, 2], [1, 4], [-1, 4], [-1, 2], [-2, 2], [0, 0]]
        scaled_verts = [[size * x, -size * y] for x, y in verts]
        poly = shapely.Polygon(scaled_verts)
        self.set_polygon(vehicle, poly, max_dist, color)

    def set_polygon(
        self, vehicle: carla.Vehicle, polygon: shapely.Polygon, max_dist=200, color: Tuple[int, int, int] = (255, 0, 0)
    ) -> int:
        """
        Enables the display of vehicles as polygons.

        :param vehicle: The vehicle to visualize.
        :type vehicle: carla.Vehicle
        :param polygon: The polygon representing the shape of the vehicle.
        :type polygon: shapely.geometry.Polygon
        :param max_dist: Maximum distance to display vehicles.
        :type max_dist: float
        :param color: Color of the polygon (RGB format).
        :type color: Tuple[int, int, int]

        :return: The index of the added polygon.
        :rtype: int
        """
        self._id_counter += 1
        self._polygons.setdefault(
            self._id_counter,
            {
                "vehicle": vehicle,
                "polygon": polygon,
                "color": color,
                "max_dist": max_dist,
            },
        )

        return self._id_counter

    def remove_polygon(self, index):
        """
        Disables the display of vehicles.

        :param index: The index of the polygon to remove.
        :type index: int
        """
        if index in self._polygons:
            del self._polygons[index]

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
        ego_forward_vec = ego_vehicle.get_transform().get_forward_vector()

        for _, target in self._polygons.items():
            vehicle: carla.Vehicle = target["vehicle"]
            max_dist = target["max_dist"]
            loc = vehicle.get_location()

            ray = loc - ego_location
            distance = ego_forward_vec.dot(ray)

            if distance < self._min_distance or distance >= max_dist:
                target["render"] = False
                continue

            bb = vehicle.bounding_box
            verts = list(bb.get_world_vertices(vehicle.get_transform()))
            x_max = -10000
            x_min = 10000
            y_max = -10000
            y_min = 10000

            # get max and min vertices
            for vert in verts:
                p = self._vis3d.vis_tool_controller.get_image_point(vert)

                # Find the rightmost vertex
                if p[0] > x_max:
                    x_max = p[0]
                # Find the leftmost vertex
                if p[0] < x_min:
                    x_min = p[0]
                # Find the highest vertex
                if p[1] > y_max:
                    y_max = p[1]
                # Find the lowest vertex
                if p[1] < y_min:
                    y_min = p[1]

            target["render"] = True
            image_point = self._vis3d.vis_tool_controller.get_image_point(loc)

            scale = distance / max_dist
            image_point[0] = (x_max + x_min) // 2
            image_point[1] = y_min - 40 + 40 * scale

            poly: shapely.Polygon = target["polygon"]
            pygame_polygon = [
                (x - x * scale + image_point[0], y - y * scale + image_point[1]) for x, y in poly.exterior.coords
            ]
            target["pygame_polygon"] = pygame_polygon

            color = target["color"]
            color_scaled = [0, 0, 0]
            color_scaled[0] = int(color[0] - color[0] * scale)
            color_scaled[1] = int(color[1] - color[1] * scale)
            color_scaled[2] = int(color[2] - color[2] * scale)
            target["color_scaled"] = tuple(color_scaled)

    def render(self, display: pygame.display):
        """
        Draws the bounding boxes on the display.

        :param display: The display to draw onto.
        :type display: pygame.display
        """
        super().render(display)
        if not VisualizationBase.is_visible:
            return
        for _, target in self._polygons.items():

            if target["render"] is True:
                color = target["color_scaled"]
                pygame_polygon = target["pygame_polygon"]
                pygame.draw.polygon(display, color, pygame_polygon, 3)
