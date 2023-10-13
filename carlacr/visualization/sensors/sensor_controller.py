from carlacr.visualization.visualization_base import VisualizationBase
from carlacr.visualization.sensors.sensor_types.camera_sensor import CameraSensor
from carlacr.visualization.sensors.sensor_types.collision_sensor import CollisionSensor
from carlacr.visualization.sensors.sensor_types.gnss_sensor import GnssSensor
from carlacr.visualization.sensors.sensor_types.lane_invasion_sensor import LaneInvasionSensor
from carlacr.visualization.sensors.sensor_types.imu_sensor import IMUSensor
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from carlacr.visualization.visualization3D import \
        Visualization3D  # from carlacr.visualization.visualization2D import World2D


class SensorController(VisualizationBase):
    """
    Sensor controller is managing and accessing the different sensors for the ego-vehicle.
    """

    def __init__(self, vis3d: 'Visualization3D', z_axis: float = 1) -> None:
        """
        Initialize the sensor controller

        :param vis3d: visualization3D base class
        :type vis3d: Visualization3D
        :param z_axis: z-axis for pygame window. Defaults to 1.
        :type z_axis: float
        """
        super().__init__(z_axis)  # must have for VisualizationBase inheriting __init__()

        self._vis3d = vis3d

        self._camera_sensor: CameraSensor = None
        self._collision_sensor: CollisionSensor = None
        self._lane_invasion_sensor: LaneInvasionSensor = None
        self._gnss_sensor: GnssSensor = None
        self._imu_sensor: IMUSensor = None

    def restart(self):
        """
        Set up the sensors.
        """
        self._camera_sensor = CameraSensor(self._vis3d.ego_vehicle, self._vis3d.config, self._vis3d.canvas_controller)
        self._collision_sensor = CollisionSensor(self._vis3d.ego_vehicle, self._vis3d.canvas_controller)
        self._lane_invasion_sensor = LaneInvasionSensor(self._vis3d.ego_vehicle, self._vis3d.canvas_controller)
        self._gnss_sensor = GnssSensor(self._vis3d.ego_vehicle)
        self._imu_sensor = IMUSensor(self._vis3d.ego_vehicle)

    @property
    def camera_sensor(self) -> CameraSensor:
        """
        Getter for the ego vehicle's camera manager.

        :return: Camera manager object.
        :rtype: CameraSensor
        """
        return self._camera_sensor

    @property
    def imu_sensor(self) -> IMUSensor:
        """
        Getter for the ego vehicle IMU sensor.

        :return: IMUSensor object.
        :rtype: IMUSensor
        """
        return self._imu_sensor

    @property
    def lane_invasion_sensor(self) -> LaneInvasionSensor:
        """
        Getter for the ego vehicle's lane invasion sensor.

        :return: Lane invasion sensor object.
        :rtype: LaneInvasionSensor
        """
        return self._lane_invasion_sensor

    @property
    def gnss_sensor(self) -> GnssSensor:
        """
        Getter for the ego vehicle's GNSS sensor.

        :return: GNSS sensor object.
        :rtype: GnssSensor
        """
        return self._gnss_sensor

    @property
    def collision_sensor(self) -> CollisionSensor:
        """
        Getter for the ego vehicle's collision sensor.

        :return: Collision sensor object.
        :rtype: CollisionSensor
        """
        return self._collision_sensor
