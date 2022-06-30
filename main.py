import time
import glob
import os
import sys
import logging
import carla
from carlacr.interface.carla_interface import CarlaInterface

logger = logging.getLogger(__name__)
try:
    if os.name == "nt":
        name = "win-amd64"
    else:
        name = "linux-x86_64"
    sys.path.append(
        glob.glob(
            f"./carla_interface/dist/carla-*{sys.version_info.major}" 
            f".{sys.version_info.minor}" 
            f"-{name}.egg"
        )[0]
    )
except (Exception, ) as e:  
    logger.error(e, exc_info=sys.exc_info())
    # pass

if __name__ == "__main__":
    # Convert commonroad-scenario to OpenDRIVE map


    scenario_name = "DEU_Test-1_1_T-1"

    file_path_in = f"scenarios/{scenario_name}.xml"  # relative path for input
    file_path_out = f"maps/converted/{scenario_name}.xodr"  # relative path for output

    # load the xml file and preprocess it
    # data = DataLoader(file_path_in)
    # logger.debug(data)

    # scenario, successors, ids = data.initialize()
    # converter = Converter(file_path_in, scenario, successors, ids)
    # converter.convert(file_path_out)

    # Simulate the scenario in Carla
    client = carla.Client("localhost", 2000)
    ci = CarlaInterface(
        cr_scenario_file_path=file_path_in,
        open_drive_map_path=file_path_out,
        carla_client=client,
        motion_planner=None,
    )

    ci.load_map()
    time.sleep(2)  # time to move your view in carla-window
    ci.setup_carla(hybrid_physics_mode=False)

    ci.run_scenario(clean_up=True, carla_vehicles=0, carla_pedestrians=0)
    StartTime = time.time()
    ExecutionTime = time.time() - StartTime
    logger.debug("Execution time in seconds: %e", str(ExecutionTime))
