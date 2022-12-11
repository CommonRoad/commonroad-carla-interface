from typing import Union

from omegaconf import ListConfig, DictConfig
import os


class Configuration:
    """Base class holding all relevant configurations."""

    def __init__(self, config: Union[ListConfig, DictConfig]):
        """Constructs all necessary configurations."""
        self.config_carla = CarlaConfiguration(config)
        self.config_carla_pedestrian = CarlaPedestrianConConfiguration(config)
        self.config_carla_2d = CarlaConfig2d(config)
        self.config_general = GeneralConfiguration(config)
        self.config_carla_obstacle = ObstacleConfiguration(config)

    @property
    def carla_config(self):
        """Get Carla Configuration."""
        return self.config_carla

    @property
    def general(self):
        """Get General Configuration."""
        return self.config_general

    @property
    def config_carla_2d(self):
        """Get Carla 2D Configuration."""
        return self._config_carla_2d

    @config_carla_2d.setter
    def config_carla_2d(self, value):
        """Set Carla 2D Configuration."""
        self._config_carla_2d = value

    @property
    def config_carla_pedestrian(self):
        """Get Carla Pedestrian Configuration."""
        return self._config_carla_pedestrian

    @config_carla_pedestrian.setter
    def config_carla_pedestrian(self, value):
        """Set Carla Pedestrian Configuration."""
        self._config_carla_pedestrian = value

    @property
    def config_carla_obstacle(self):
        """Get Carla Obstacle Configuration."""
        return self._config_carla_obstacle

    @config_carla_obstacle.setter
    def config_carla_obstacle(self, value):
        """Set Carla Obstacle Configuration."""
        self._config_carla_obstacle = value


class CarlaConfiguration:
    """Class holding carla configuration."""

    def __init__(self, config: Union[ListConfig, DictConfig]):
        """Initializes Carla configuration attributes."""
        config_relevant = config.carla_config

        self.host = config_relevant.carla_parameters.host
        self.port = config_relevant.carla_parameters.port
        self.sleep_time = config_relevant.carla_parameters.sleep_time
        self.carla_root_path = config_relevant.carla_root_path
        if os.sep == "/":
            self.carla_path = os.path.join(self.carla_root_path, "CarlaUE4.sh")

        if os.sep == "\\":
            self.carla_path = os.path.join(self.carla_root_path, "CarlaUE4.exe")
        self.carla_agent_path = os.path.join(self.carla_root_path, "PythonAPI/carla")
        self.carla_examples_path = os.path.join(self.carla_root_path, "PythonAPI")


class GeneralConfiguration:
    """Class holding general configuration (root path, output path, ...)."""

    def __init__(self):
        """Initializes general configuration attributes."""
        self.path_root = "." + os.sep + ".." + os.sep
        self.scenario_path = self.path_root + "scenarios/"
        self.map_path = self.path_root + "maps/"


class CarlaPedestrianConConfiguration:
    """Class holding Carla Pedestrian configuration."""

    def __init__(self, config: Union[ListConfig, DictConfig]):
        """Initializes Carla Pedestrians configuration attributes."""
        config_relevant = config.carla_pedestrians

        self.percentage_pedestrians_running = config_relevant.percentage_pedestrians_running
        self.percentage_pedestrians_crossing = config_relevant.percentage_pedestrians_crossing


class CarlaConfig2d:
    """Class holding Carla 2D configuration."""

    def __init__(self, config: Union[ListConfig, DictConfig]):
        """Initializes Carla 2D configuration attributes."""
        config_relevant = config.carla_2D_mode

        self.description = config_relevant.description
        self.res = config_relevant.res
        self.filter = config_relevant.filter
        self.map = config_relevant.map


class ObstacleConfiguration:
    """Class holding carla Obstacle configuration."""

    def __init__(self, config: Union[ListConfig, DictConfig]):
        """Initializes Carla Obstacle configuration attributes."""
        config_relevant = config.obstacle_interface.obstacle_parameters

        self.args_lateral_dict = config_relevant.args_lateral_dict
        self.args_long_dict = config_relevant.args_long_dict

        config_relevant = config.obstacle_interface.obstacle_parameters.args_ackermann_PID
        self.speed_kp = config_relevant.speed_kp
        self.speed_ki = config_relevant.speed_ki
        self.speed_kd = config_relevant.speed_kd
        self.accel_kp = config_relevant.accel_kp
        self.accel_ki = config_relevant.accel_ki
        self.accel_kd = config_relevant.accel_kd
