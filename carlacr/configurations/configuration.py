from typing import Union

from omegaconf import ListConfig, DictConfig


class Configuration:
    """Base class holding all relevant configurations"""

    def __init__(self, config: Union[ListConfig, DictConfig]):

        self.config_carla = CarlaConfiguration(config)
        self.config_carla_pedestrian = CarlaPedestrianConConfiguration(config)
        self.config_carla_2d = CarlaConfig2d(config)
        self.config_general = GeneralConfiguration(config)

    @property
    def carla_config(self):
        return self.config_carla

    @property
    def general(self):
        return self.config_general

    @property
    def config_carla_2d(self):
        return self._config_carla_2d

    @config_carla_2d.setter
    def config_carla_2d(self, value):
        self._config_carla_2d = value

    @property
    def config_carla_pedestrian(self):
        return self._config_carla_pedestrian

    @config_carla_pedestrian.setter
    def config_carla_pedestrian(self, value):
        self._config_carla_pedestrian = value


class CarlaConfiguration:
    """
    Class holding carla configuration.
    """

    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.carla_config

        self.host = config_relevant.carla_parameters.host
        self.port = config_relevant.carla_parameters.port
        self.sleep_time = config_relevant.carla_parameters.sleep_time
        self.carla_root_path = config_relevant.carla_root_path
        self.carla_path = config_relevant.carla_path_parameters.carla_path
        self.carla_agent_path = config_relevant.carla_path_parameters.carla_agent_path
        self.carla_examples_path = config_relevant.carla_path_parameters.carla_examples_path


class GeneralConfiguration:
    """
    Class holding general configuration (root path, output path, ...).
    """

    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.general

        self.path_root = config_relevant.path_root
        self.map_path = config_relevant.map_path
        self.scenario_path = config_relevant.scenario_path


class CarlaPedestrianConConfiguration:
    """
    Class holding Carla Pedestrian configuration.
    """

    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.carla_pedestrians

        self.percentage_pedestrians_running = config_relevant.percentage_pedestrians_running
        self.percentage_pedestrians_crossing = config_relevant.percentage_pedestrians_crossing


class CarlaConfig2d:
    """
    Class holding Carla Pedestrian configuration.
    """

    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.carla_2D_mode

        self.description = config_relevant.description
        self.res = config_relevant.res
        self.filter = config_relevant.filter
        self.map = config_relevant.map
