from typing import TYPE_CHECKING, Optional

import carla
import numpy as np
import pygame

from crcarla.helper.config import VisActivation
from crcarla.visualization.sensors.sensor_types.camera_sensor import CameraSensor
from crcarla.visualization.visualization_base import VisualizationBase
from crcarla.visualization.visualization_tools.tools.bounding_box_tool import BoundingBoxTool
from crcarla.visualization.visualization_tools.tools.line_to_vehicle import LineToVehicle
from crcarla.visualization.visualization_tools.tools.polygon_tool import PolygonTool
from crcarla.visualization.visualization_tools.tools.text import Text
from crcarla.visualization.visualization_tools.tools.trajectories import TrajectoryTool

if TYPE_CHECKING:
    from crcarla.visualization.visualization3D import Visualization3D


class VisualizationToolsController(VisualizationBase):
    """
    Managing the visualization tools.
    """

    def __init__(self, vis3d: "Visualization3D", vis_activation: VisActivation, z_axis: float = 1) -> None:
        """
        Initializes an instance of VisualizationToolsController.

        :param vis3d: Base Visualization3D instance.
        :param z_axis: The z-coordinate of the visualization reference point. Defaults to 1.
        """
        super().__init__(z_axis)

        self._vis3d = vis3d
        self.__K: Optional[np.ndarray] = None
        self.__T_CW: Optional[np.ndarray] = None

        self._bb_tool: Optional[BoundingBoxTool] = None
        self._line_to_vehicle: Optional[LineToVehicle] = None
        self.text: Optional[Text] = None
        self._poly_tool: Optional[PolygonTool] = None
        self.trajectory_tool: Optional[TrajectoryTool] = None

        self._vis_activation = vis_activation

    @property
    def _camera_intrinsics(self) -> np.ndarray:
        """
        Get the camera intrinsics.

        :return: Camera intrinsics.
        """
        if None is self.__K:
            self.__K = self._vis3d.sensor_controller.camera_sensor.get_camera_intrinsics()
        return self.__K

    @property
    def _transform_cw(self) -> np.ndarray:
        """
        Get the camera extrinsic matrix.

        :return: Camera extrinsic matrix.
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

        if self._vis_activation.bounding_boxes:
            self._bb_tool = BoundingBoxTool(self._vis3d, print_distance=self._vis_activation.text)
            self._bb_tool.show_vehicles(200, show_as_3d=True, print_distance=self._vis_activation.text)
            self._bb_tool.show_city_object_label(
                label=carla.CityObjectLabel.Pedestrians,
                max_dist=200,
                color=(0, 0, 255),
                show_as_3d=True,
            )
            self._bb_tool.show_city_object_label(
                label=carla.CityObjectLabel.TrafficSigns, max_dist=200, show_as_3d=False
            )
        if self._vis_activation.line:
            self._line_to_vehicle = LineToVehicle(self._vis3d)
            for vehicle in self._vis3d.vehicles:
                self._line_to_vehicle.set_connection(vehicle, 50)
        if self._vis_activation.text:
            self.text = Text(self._vis3d)
        if self._vis_activation.polygon:
            self._poly_tool = PolygonTool(self._vis3d)
            for vehicle in self._vis3d.vehicles:
                self._poly_tool.set_arrow(vehicle, max_dist=150)
        if self._vis_activation.trajectory:
            self.trajectory_tool = TrajectoryTool(self._vis3d)

    def tick(self, clock: pygame.time.Clock):
        """
        Called to update the current position of the bounding boxes.

        :param clock: The game clock.
        """
        super().tick(clock)
        self.update_transform_cw()

    def get_image_point(self, loc: carla.Location) -> np.ndarray:
        """
        Project a any 3D carla location into the camera-sensor frame.

        :param loc: 3D carla world target location.
        :return: Projected 2D point in the camera sensor frame.
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
        point_img = np.dot(self._camera_intrinsics, point_camera)
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
