import os
from carlacr.configurations.configuration_builder import ConfigurationBuilder


def set_configs():

    """
    :parameter:
        path_config: path of the config files
        config: accumulating all default config files
    :return:
        config:  config containing all relevant information
    """

    path_config = os.path.dirname(os.path.abspath(__file__))
    ConfigurationBuilder.set_path_to_config(path_config)
    config = ConfigurationBuilder.build_configuration()

    return config
