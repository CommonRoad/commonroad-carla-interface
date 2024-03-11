from typing import TYPE_CHECKING, Dict, List, Tuple

import carla
import pygame

from crcarla.visualization.visualization_base import VisualizationBase

if TYPE_CHECKING:
    from crcarla.visualization.visualization3D import Visualization3D


class BoundingBoxTool(VisualizationBase):
    """
    A class to visualize bounding boxes of objects in a 3D world.
    """

    def __init__(self, vis3d: "Visualization3D", z_axis: float = 1) -> None:
        """
        Initializes an instance of BoundingBox3D.

        :param vis3d: A reference to the 3D visualization instance.
        :type vis3d: Visualization3D
        :param z_axis: The z-coordinate of the visualization reference point.
        :type z_axis: float
        """
        super().__init__(z_axis)

        self._vis3D = vis3d  # The 3D visualization instance

        # Initialize the vertices and edges of the bounding boxes
        self.render_dicts = []

        # Settings for displaying vehicles
        self._show_vehicles_dict = {
            "enable": True,
            "color": (255, 0, 0),
            "show_as_3d": False,
            "max_dist": 200.0,
            "print_distance": True,
        }
        self._forward_delta_distance = 1.0

        # Settings for displaying city object labels
        self._active_city_object_label: Dict[carla.CityObjectLabel, dict] = {}

    def show_vehicles(
        self, max_dist, color: Tuple[int, int, int] = (255, 0, 0), print_distance=True, show_as_3d: bool = False
    ):
        """
        Enables the display of vehicles.

        :param max_dist: Maximum distance to display vehicles.
        :type max_dist: float
        :param color: Color of the bounding boxes (RGB format).
        :type color: Tuple[int, int, int]
        :param print_distance: Whether to print the distance or not.
        :type print_distance: bool
        :param show_as_3d: Whether to show the bounding boxes in 3D.
        :type show_as_3d: bool
        """
        self._show_vehicles_dict = {
            "enable": True,
            "color": color,
            "show_as_3d": show_as_3d,
            "max_dist": max_dist,
            "print_distance": print_distance,
        }

    def hide_vehicles(self):
        """Disables the display of vehicles."""
        self._show_vehicles = False

    def show_city_object_label(
        self, label: carla.CityObjectLabel, max_dist: float, color: Tuple[int, int, int] = (0, 255, 0), show_as_3d=False
    ):
        """
        Activates a label to display the respective bounding boxes.

        :param label: The label of the objects to visualize.
        :type label: carla.CityObjectLabel
        :param max_dist: Maximum distance to display objects of this label.
        :type max_dist: float
        :param color: Color of the bounding boxes (RGB format).
        :type color: Tuple[int, int, int]
        :param show_as_3d: Whether to show the bounding boxes in 3D.
        :type show_as_3d: bool
        """
        data = self._active_city_object_label.setdefault(label, {})
        if "bbs" not in data:
            data["bbs"] = self._vis3D.carla_world.get_level_bbs(label)

        self._active_city_object_label[label]["max_dist"] = max_dist
        self._active_city_object_label[label]["color"] = color
        self._active_city_object_label[label]["show_as_3d"] = show_as_3d
        self._active_city_object_label[label]["enable"] = True

    def hide_city_object_label(self, label: carla.CityObjectLabel):
        """
        Removes an activated label.

        :param label: The label to remove.
        :type label: carla.CityObjectLabel
        """
        if label in self._active_city_object_label:
            self._active_city_object_label[label]["enable"] = False

    def tick(self, clock: pygame.time.Clock):
        """
        Called to update the current position of the bounding boxes.

        :param clock: The game clock.
        :type clock: pygame.time.Clock
        """
        super().tick(clock)
        if not VisualizationBase.is_visible:
            return

        self._update_city_object_labels()

        self._update_vehicles()

    def render(self, display: pygame.display):
        """
        Draws the bounding boxes on the display.

        :param clock: The game clock.
        :type clock: pygame.time.Clock
        """
        super().render(display)
        if not VisualizationBase.is_visible:
            return
        for item in self.render_dicts:
            lines = item["lines"]
            color = item["color"]
            for line in lines:
                pygame.draw.line(display, color, line[0], line[1], 1)

        # Clear the vertices after drawing to calculate them anew for the next frame
        self.render_dicts.clear()

    def _update_city_object_labels(self):
        # Determine the current position and orientation of the ego vehicle
        ego_vehicle = self._vis3D.ego_vehicle
        ego_vehicle_location = ego_vehicle.get_transform().location
        forward_vec = ego_vehicle.get_transform().get_forward_vector()

        # Update the bounding boxes for the activated city object labels
        for label in self._active_city_object_label:  # pylint: disable=consider-using-dict-items
            dict_tmp = self._active_city_object_label[label]
            if not dict_tmp["enable"]:
                continue
            color = dict_tmp["color"]
            max_dist = dict_tmp["max_dist"]
            show_as_3d = dict_tmp["show_as_3d"]

            # if object is dynam object, update them every tick (to update location)
            if label is carla.CityObjectLabel.Pedestrians:
                dict_tmp["bbs"] = self._vis3D.carla_world.get_level_bbs(label)

            # get over all bounding boxes
            for bb in dict_tmp["bbs"]:
                bb: carla.BoundingBox
                distance = bb.location.distance(ego_vehicle_location)

                if distance > max_dist:
                    continue

                # Filter bounding boxes based on their distance from the ego vehicle
                ray = bb.location - ego_vehicle_location
                if forward_vec.dot(ray) > self._forward_delta_distance:
                    verts = list(bb.get_world_vertices(carla.Transform()))
                    linedata = self.vertices_to_linedata(verts, color, show_as_3d)
                    self.render_dicts.append(linedata)

    def _update_vehicles(self):
        # Determine the current position and orientation of the ego vehicle
        ego_vehicle = self._vis3D.ego_vehicle
        ego_vehicle_location = ego_vehicle.get_transform().location
        forward_vec = ego_vehicle.get_transform().get_forward_vector()
        if self._show_vehicles_dict["enable"] is False:
            return
        color = self._show_vehicles_dict["color"]
        max_dist = self._show_vehicles_dict["max_dist"]
        show_as_3d = self._show_vehicles_dict["show_as_3d"]
        print_distance = self._show_vehicles_dict["print_distance"]

        # Get vehicles in front up to a specific distance
        vehicles: List[carla.Vehicle] = []

        for vehicle, dist in self._vis3D.vehicles_by_dist:
            ray = vehicle.get_location() - ego_vehicle_location
            if dist <= max_dist and forward_vec.dot(ray) > self._forward_delta_distance:
                vehicles.append([vehicle, dist])

        # get the render information for the vehicle bounding boxes
        for vehicle, dist in vehicles:

            color2 = [0, 0, 0]
            color2[0] = int(color[0] - color[0] * dist / max_dist)
            color2[1] = int(color[1] - color[1] * dist / max_dist)
            color2[2] = int(color[2] - color[2] * dist / max_dist)
            color2 = tuple(color2)

            vehicle: carla.Vehicle
            bb = vehicle.bounding_box
            verts = list(bb.get_world_vertices(vehicle.get_transform()))

            linedata = self.vertices_to_linedata(verts, color2, show_as_3d)
            self.render_dicts.append(linedata)

            if print_distance:
                x = (linedata["x_max"] + linedata["x_min"]) // 2
                y = linedata["y_min"]
                size = int(40 - 39 * dist / max_dist)
                self._vis3D.vis_tool_controller.text.add_static_text_2d(str(int(dist)), x, y, size, 1, color2)

    def vertices_to_linedata(self, verts, color, show_as_3d: bool) -> dict:
        """
        Create lines from vertices and store corresponding information for render().

        :param verts: List of vertices as tuples (x, y).
        :type verts: List[Tuple[float, float]]
        :param color: Color of the lines in RGB format.
        :type color: Tuple[int, int, int]
        :param show_as_3d: Choose between 2D and 3D bounding box.
        :type show_as_3d: bool

        :return: A dictionary with all corresponding information to draw the lines.
        :rtype: dict
        Contains the bounding box coordinates (x_min, y_min, x_max, y_max),
        the lines to draw (lines), and the color information (color).
        """
        lines = []
        edges = [[0, 1], [1, 3], [3, 2], [2, 0], [0, 4], [4, 5], [5, 1], [5, 7], [7, 6], [6, 4], [6, 2], [7, 3]]

        x_max = -10000
        x_min = 10000
        y_max = -10000
        y_min = 10000

        # get max and min vertices
        for vert in verts:
            p = self._vis3D.vis_tool_controller.get_image_point(vert)

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

        if show_as_3d:
            for edge in edges:
                p1 = self._vis3D.vis_tool_controller.get_image_point(verts[edge[0]])
                p2 = self._vis3D.vis_tool_controller.get_image_point(verts[edge[1]])
                lines.append([(int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1]))])
        else:

            # Draw a bounding box around the object
            lines.append([(int(x_min), int(y_min)), (int(x_max), int(y_min))])
            lines.append([(int(x_min), int(y_max)), (int(x_max), int(y_max))])
            lines.append([(int(x_min), int(y_min)), (int(x_min), int(y_max))])
            lines.append([(int(x_max), int(y_min)), (int(x_max), int(y_max))])

        return {
            "x_min": x_min,
            "y_min": y_min,
            "x_max": x_max,
            "y_max": y_max,
            "lines": lines,
            "color": color,
        }
