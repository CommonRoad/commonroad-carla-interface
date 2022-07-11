from typing import Union

from omegaconf import ListConfig, DictConfig


class Configuration:
    """Base class holding all relevant configurations"""

    def __init__(self, config: Union[ListConfig, DictConfig]):

        self.config_carla = CarlaConfiguration(config)
        self.config_carla_pedestrian = CarlaPedestrianConConfiguration(config)

    @property
    def carla_config(self):
        return self.config_carla

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
        config_relevant = config.carla_config.carla_parameters

        self.host = config_relevant.host
        self.port = config_relevant.port
        self.sleep_time = config_relevant.sleep_time
        self.carla_path = config_relevant.carla_path


class CarlaPedestrianConConfiguration:
    """
    Class holding Carla Pedestrian configuration.
    """

    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.carla_pedestrians

        self.percentage_pedestrians_running = config_relevant.percentage_pedestrians_running
        self.percentage_pedestrians_crossing = config_relevant.percentage_pedestrians_crossing
