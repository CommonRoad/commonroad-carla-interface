import glob
import logging
import os
from typing import Union

from omegaconf import OmegaConf, ListConfig, DictConfig

from carlacr.configurations.configuration import Configuration


class ConfigurationBuilder:
    """
    Class which is used to build a Configuration instance.
    """
    path_config: str = ""
    path_config_default: str = ""

    @classmethod
    def set_path_to_config(cls, path_config: str, dir_configs_default: str = "defaults"):
        """Sets the path to the directory containing configurations.

        Args:
            path_config (str): root folder of configurations
            dir_configs_default (str): directory under root folder containing
            default carla interface config files.

        """
        cls.path_config = path_config
        cls.path_config_default = os.path.join(path_config, dir_configs_default)

    @classmethod
    def build_configuration(cls) -> Configuration:

        """Builds configuration from default files.

        Steps:
            1. Load default files
            2. Build Configuration object

        Returns:
            Configuration: configuration containing all relevant information
        """

        # default configurations
        config_default = cls.construct_default_config()

        config_combined = OmegaConf.merge(config_default)
        print(OmegaConf.to_yaml(config_combined))

        config = Configuration(config_combined)

        return config

    @classmethod
    def construct_default_config(cls) -> Union[ListConfig, DictConfig]:
        """
        Constructs default configuration by accumulating yaml files.
        Collects all default config files ending with .yaml under path_config_default.
        """

        config_default = OmegaConf.create()
        ConfigurationBuilder.register_join_paths_resolver()
        for path_file in glob.glob(cls.path_config_default + "/*.yaml"):
            with open(path_file, encoding="utf-8") as file_config:
                try:
                    config_partial = OmegaConf.load(file_config)
                    OmegaConf.resolve(config_partial)
                    name_file = path_file.split("/")[-1].split(".")[0]

                except Exception as e:
                    print(e)

                else:
                    config_default[name_file] = config_partial

        return config_default

    @classmethod
    def register_join_paths_resolver(cls):
        """
        Registers Python functions to yaml configuration files using omega.conf's functionality.
        """
        try:
            OmegaConf.register_new_resolver("join_paths",
                                            lambda base_path, additional_path: os.path.join(base_path, additional_path))
        except ValueError:
            logging.debug("Re-attempting to register join_paths resolver exception is suppressed.")
