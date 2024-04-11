import copy
import traceback
from pathlib import Path
from typing import List

from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.common.util import FileFormat
from commonroad.scenario.scenario import Tag

from crcarla.carla_interface import CarlaInterface
from crcarla.helper.config import CarlaParams, CustomVis, SimulationParams

param = CarlaParams(log_level="INFO")
param.vehicle.vehicle_ks_state = False
param.simulation.max_time_step = 3000
param.offscreen_mode = True
param.offscreen_mode = True
param.vis_type = CustomVis.NONE

percentages = [25]
num_actors = [(75, 20)]
distances_leading = [1]


def generate_params() -> List[List[SimulationParams]]:

    param.logger.info("Generate param set.")
    list_param = []
    tmp_param = copy.deepcopy(param.simulation)

    for idx in ["01", "02", "04", "07", "10HD"]:
        new_map = []
        tmp_param.map = f"Town{idx}"
        for num_vehicles, number_walkers in num_actors:
            tmp_param.number_vehicles = num_vehicles
            tmp_param.number_walkers = number_walkers
            tmp_param2 = copy.deepcopy(tmp_param)

            tmp_param2.tm.global_distance_to_leading_vehicle = distances_leading[0]
            tmp_param2.tm.global_percentage_speed_difference = percentages[0]
            tmp_param2.tm.ignore_signs_percentage = percentages[0]
            tmp_param2.tm.ignore_lights_percentage = percentages[0]
            tmp_param2.tm.keep_right_rule_percentage = percentages[0]
            new_map.append(copy.deepcopy(tmp_param2))

        list_param.append(new_map)

    return list_param


param_sets = generate_params()

failing_maps = []

param.logger.info("Generate scenarios.")
for map_list in param_sets:
    prediction_idx = 1
    param.map = map_list[0].map
    param.logger.info("use map {}".format(param.map))
    ci = CarlaInterface(param)
    for tmp_param in map_list:
        param.logger.info("Prediction idx {}".format(prediction_idx))
        update_map = True if param.map != tmp_param.map else False
        param.map = tmp_param.map
        if param.map in failing_maps:
            continue
        param.simulation = tmp_param

        ci.update_config(param, update_map)

        try:
            sc_base = ci.create_cr_map()
        except AttributeError:
            param.logger.error("Skip scenario generation for config. Traceback: {}".format(traceback.format_exc()))
            failing_maps.append(param.map)
            continue
        sc_base.scenario_id.map_name = f"CARLA{tmp_param.map}"

        new_scenario = copy.deepcopy(sc_base)
        new_scenario.scenario_id.configuration_id = prediction_idx
        prediction_idx += 1

        try:
            sc, pps = ci.scenario_generation(new_scenario)
        except (RuntimeError, AttributeError):
            param.logger.error("Skip scenario generation for config. Traceback: {}".format(traceback.format_exc()))
            continue
        sc.tags = {Tag.URBAN}

        scenario_path = Path(__file__).parent / "generated_scenarios"
        if not scenario_path.exists():
            scenario_path.mkdir(parents=True, exist_ok=True)

        CommonRoadFileWriter(
            sc,
            pps,
            author="Sebastian Maierhofer",
            affiliation="Technical University of Munich",
            source="CARLA",
            tags={Tag.URBAN},
            file_format=FileFormat.PROTOBUF,
        ).write_to_file(str(scenario_path / (str(sc.scenario_id) + ".pb")), OverwriteExistingFile.ALWAYS)
