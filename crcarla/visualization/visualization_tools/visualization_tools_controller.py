from typing import TYPE_CHECKING

import carla
import numpy as np
import pygame

from crcarla.visualization.sensors.sensor_types.camera_sensor import CameraSensor
from crcarla.visualization.visualization_base import VisualizationBase
from crcarla.visualization.visualization_tools.tools.bounding_box_tool import BoundingBoxTool
from crcarla.visualization.visualization_tools.tools.line_to_vehicle import LineToVehicle
from crcarla.visualization.visualization_tools.tools.polygon_tool import PolygonTool
from crcarla.visualization.visualization_tools.tools.text import Text

if TYPE_CHECKING:
    from crcarla.visualization.visualization3D import Visualization3D


class VisualizationToolsController(VisualizationBase):
    """
    Managing the visualization tools.
    """

    def __init__(self, vis3d: "Visualization3D", z_axis: float = 1) -> None:
        """
        Initializes an instance of VisualizationToolsController.

        :param vis3d: Base Visualization3D instance.
        :type vis3d: Visualization3D
        :param z_axis: The z-coordinate of the visualization reference point. Defaults to 1.
        :type z_axis: float
        """
        super().__init__(z_axis)

        self._vis3d = vis3d
        self.__K: np.ndarray = None
        self.__T_CW: np.ndarray = None

        self._bb_tool: BoundingBoxTool = None
        self._line_to_vehicle: LineToVehicle = None
        self.text: Text = None
        self._poly_tool: PolygonTool = None

    @property
    def _camera_intriniscs(self) -> np.ndarray:
        """
        Get the camera intrinsics.

        :return: Camera intrinsics.
        :rtype: np.ndarray
        """
        if None is self.__K:
            self.__K = self._vis3d.sensor_controller.camera_sensor.get_camera_intrinsics()
        return self.__K

    @property
    def _transform_cw(self) -> np.ndarray:
        """
        Get the camera extrinsic matrix.

        :return: Camera extrinsic matrix.
        :rtype: np.ndarray
        """
        if None is self.__T_CW:
            self.update_transform_cw()
        return self.__T_CW

    def restart(self):
        """
        Restart the visualization tools.

        """
        super().restart()
        self.__K = None
        self.__T_CW = None

        self._bb_tool = BoundingBoxTool(self._vis3d)
        self._line_to_vehicle = LineToVehicle(self._vis3d)
        self.text = Text(self._vis3d)
        self._poly_tool = PolygonTool(self._vis3d)

        self._bb_tool.show_vehicles(200, show_as_3d=True)
        self._bb_tool.show_city_object_label(
            label=carla.CityObjectLabel.Pedestrians,
            max_dist=200,
            color=(0, 0, 255),
            show_as_3d=True,
        )
        self._bb_tool.show_city_object_label(label=carla.CityObjectLabel.TrafficSigns, max_dist=200, show_as_3d=False)

        for vehicle in self._vis3d.vehicles:
            self._line_to_vehicle.set_connection(vehicle, 50)
            self._poly_tool.set_arrow(vehicle, max_dist=150)

    def tick(self, clock: pygame.time.Clock):
        """
        Called to update the current position of the bounding boxes.

        :param clock: The game clock.
        :type clock: pygame.time.Clock
        """
        super().tick(clock)
        self.update_transform_cw()

    def get_image_point(self, loc: carla.Location) -> np.ndarray:
        """
        Project a any 3D carla location into the camera-sensor frame.

        :param loc: 3D carla world target location.
        :type loc: carla.Location
        :return: Projected 2D point in the camera sensor frame.
        :rtype: np.ndarray
        """
        # Calculate 2D projection of 3D coordinate

        # Format the input coordinate (loc is a carla.Position object)
        point = np.array([loc.x, loc.y, loc.z, 1])
        # transform to camera coordinates
        point_camera = np.dot(self._transform_cw, point)

        # Now we must change from UE4's coordinate system to an "standard"
        # (x, y ,z) -> (y, -z, x)
        # and we remove the fourth component also
        point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

        # Now project 3D->2D using the camera matrix
        point_img = np.dot(self._camera_intriniscs, point_camera)
        # Normalize
        point_img[0] /= point_img[2]
        point_img[1] /= point_img[2]

        return point_img[0:2]

    def update_transform_cw(self):
        """
        Update the camera extrinsic matrix.
        """
        camera: CameraSensor = self._vis3d.sensor_controller.camera_sensor
        inv_carla_transform = camera.sensor.get_transform().get_inverse_matrix()
        self.__T_CW = np.array(inv_carla_transform)
