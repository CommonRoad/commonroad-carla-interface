#!/usr/bin/env python

from conversion.dataloader import DataLoader
from conversion.converter import Converter

import time
import glob
import os
import sys

try:
    sys.path.append(
        glob.glob(
            "./carla_interface/dist/carla-*%d.%d-%s.egg"
            % (
                sys.version_info.major,
                sys.version_info.minor,
                "win-amd64" if os.name == "nt" else "linux-x86_64",
            )
        )[0]
    )
except Exception as e:
    print(e)
    pass

import carla
from src.carla_interface import CarlaInterface


if __name__ == "__main__":

    ## Convert commonroad-scenario to OpenDRIVE map

    scenario_name = "DEU_Test-1_1_T-1"

    file_path_in = f"scenarios/{scenario_name}.xml"  # relative path for input
    file_path_out = f"maps/converted/{scenario_name}.xodr"  # relative path for output

    # load the xml file and preprocess it
    data = DataLoader(file_path_in)
    print(data)

    scenario, successors, ids = data.initialize()
    converter = Converter(file_path_in, scenario, successors, ids)
    converter.convert(file_path_out)

    # Simulate the scenario in Carla
    client = carla.Client("localhost", 2000)
    ci = CarlaInterface(
        cr_scenario_file_path=file_path_in,
        open_drive_map=file_path_out,
        carla_client=client,
        motion_planner=None,
    )

    ci.load_map()
    time.sleep(2)  # time to move your view in carla-window
    ci.setup_carla(hybrid_physics_mode=False)

    startTime = time.time()
    ci.run_scenario(clean_up=True, carla_vehicles=0, carla_pedestrians=0)
    executionTime = time.time() - startTime
    print("Execution time in seconds: " + str(executionTime))
