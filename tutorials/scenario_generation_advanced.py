import copy
import traceback
from pathlib import Path
from typing import List, Dict, Tuple

from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.common.util import FileFormat
from commonroad.scenario.scenario import Tag
from commonroad.visualization.mp_renderer import MPRenderer
from omegaconf import OmegaConf

from crcarla.carla_interface import CarlaInterface
from crcarla.helper.config import CarlaParams, SimulationParams

import matplotlib

matplotlib.use("TkAgg")

config_carla = CarlaParams.load(Path(__file__).parent / "carla_config.yaml")
config_sim = OmegaConf.load(Path(__file__).parent / "scenario_gen_config.yaml")

settings = list(config_sim.keys())
settings.remove("default_params_simulation")
default_config_sim = config_sim["default_params_simulation"]


def generate_params() -> Dict[str, List[Tuple[str, SimulationParams]]]:
    config_carla.logger.info("Generate param set.")
    list_param = {}
    tmp_param = copy.deepcopy(config_carla.simulation)

    def param_iteration(parameter_name: str, parameter_configs: Dict) -> List[Tuple[str, SimulationParams]]:
        new_params = []
        if parameter_name != "distances_leading":
            values_to_iterate = parameter_configs["percentages"]
        else:
            values_to_iterate = parameter_configs["distances_leading"]
        for value in values_to_iterate:
            tmp_param2 = copy.deepcopy(tmp_param)
            tmp_param2.tm.__setattr__(parameter_name, value)
            if parameter_configs["update_seed"]:
                param_iteration.seed_counter += 1
                param_iteration.seed_walker_counter += 1
                tmp_param2.tm.seed = param_iteration.seed_counter
                tmp_param2.seed_walker = param_iteration.seed_walker_counter
            new_params.append((parameter_name, copy.deepcopy(tmp_param2)))
        return new_params

    param_iteration.seed_counter = 0
    param_iteration.seed_walker_counter = 0

    for carla_param in settings:
        param_config = copy.deepcopy(default_config_sim)
        param_config.update(config_sim[carla_param])
        for idx in param_config.maps:
            tmp_param.map = f"Town{idx}"
            tmp_param.number_vehicles = param_config.num_vehicles
            tmp_param.number_walkers = param_config.num_pedestrians
            new_params = param_iteration(carla_param, param_config)

            if tmp_param.map not in list_param:
                list_param[tmp_param.map] = new_params
            else:
                list_param[tmp_param.map].extend(new_params)

    return list_param


param_sets = generate_params()

failing_maps = []

config_carla.logger.info("Generate scenarios.")
ci = None
scenario_mapping = ""
for map_name, map_list_with_key in param_sets.items():
    prediction_idx = 1
    config_carla.map = map_name
    config_carla.logger.info("use map {}".format(config_carla.map))
    if ci is None:
        ci = CarlaInterface(config_carla)
    for param_name, tmp_param in map_list_with_key:
        config_carla.logger.info("Prediction idx {}".format(prediction_idx))
        update_map = True if config_carla.map != tmp_param.map else False
        config_carla.map = tmp_param.map
        if config_carla.map in failing_maps:
            continue
        config_carla.simulation = tmp_param

        ci.update_config(config_carla, update_map)

        try:
            sc_base = ci.create_cr_map()
        except AttributeError:
            config_carla.logger.error(
                "Skip scenario generation for config. Traceback: {}".format(traceback.format_exc())
            )
            failing_maps.append(config_carla.map)
            continue
        sc_base.scenario_id.map_name = f"CARLA{tmp_param.map}"

        new_scenario = copy.deepcopy(sc_base)
        new_scenario.scenario_id.configuration_id = prediction_idx
        prediction_idx += 1

        try:
            sc, pps = ci.scenario_generation(new_scenario)
            scenario_mapping += (
                f"success - {param_name} - percentage: {tmp_param.tm[param_name]} - "
                f"{str(new_scenario.scenario_id)}\n"
            )
        except (RuntimeError, AttributeError):
            scenario_mapping += (
                f"failed - {param_name} - percentage: {tmp_param.tm[param_name]} - "
                f"{str(new_scenario.scenario_id)}\n"
            )
            config_carla.logger.error(
                "Skip scenario generation for config. Traceback: {}".format(traceback.format_exc())
            )
            config_carla.save("./error.yaml")  # TODO proper naming
            continue
        sc.tags = {Tag.URBAN}

        scenario_path = Path(__file__).parent / "generated_scenarios" / param_name
        if not scenario_path.exists():
            scenario_path.mkdir(parents=True, exist_ok=True)

        CommonRoadFileWriter(
            sc,
            pps,
            author="Sebastian Maierhofer",
            affiliation="Technical University of Munich",
            source="CommonRoad-CARLA-Interface",
            tags={Tag.URBAN},
            file_format=FileFormat.PROTOBUF,
        ).write_to_file(str(scenario_path / (str(sc.scenario_id) + ".xml")), OverwriteExistingFile.ALWAYS)

        rnd = MPRenderer()
        rnd.create_video([sc], str(scenario_path / str(sc.scenario_id)) + ".mp4")
    config_carla.start_carla_server = False

with open(f'{Path(__file__).parent / "generated_scenarios/scenario_mapping.txt"}', "w") as f:
    f.write(scenario_mapping)
