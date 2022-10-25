import time
import carla
import subprocess
import logging
from carlacr.interface.carla_interface import CarlaInterface
from carlacr.configurations.set_configs import set_configs


def carlacr_starter(set_map_name: str = None,
                    set_scenario_name: str = None,
                    set_hybrid_physics_mode: bool = False,
                    set_clean_up: bool = True,
                    set_carla_vehicles: int = 0,
                    set_carla_pedestrians: int = 0,
                    set_motionplaner: str = None,
                    set_offscreen_mode: bool = False):
    """
    start carla interface with 3D mode or offscreen mode
    :param set_map_name: name of map file
    :param set_scenario_name: name of scenario name
    :param set_hybrid_physics_mode: Parameter for ci.setup_carla(hybrid_physics_mode=False)
    :param set_clean_up: parameter for ci.run_scenario(clean_up=True,
    carla_vehicles=0, carla_pedestrians=0)
    :param set_carla_vehicles: parameter for ci.run_scenario(clean_up=True,
    carla_vehicles=0, carla_pedestrians=0)
    :param set_carla_pedestrians: parameter for ci.run_scenario(clean_up=True,
    carla_vehicles=0, carla_pedestrians=0)
    :param set_motionplaner: parameter for ci.run_scenario(clean_up=True,
    carla_vehicles=0, carla_pedestrians=0, MotionPlaner)
    :param set_offscreen_mode: False for 3D mode and True for offscreen mode
    :return: None
    """
    # Load config files
    config = set_configs()
    sleep_time = config.config_carla.sleep_time

    if set_offscreen_mode is True:
        offscreen = '-RenderOffScreen'
    else:
        offscreen = ''

    with subprocess.Popen([config.config_carla.carla_path, offscreen]):
        time.sleep(sleep_time)

        logging.basicConfig(level=logging.DEBUG)

        map_path = config.general.map_path
        scenario_path = config.general.scenario_path

        map_name = set_map_name
        scenario_name = set_scenario_name

        client = carla.Client(config.carla_config.host, config.carla_config.port)

        motionplaner_idx = set_motionplaner

        ci = CarlaInterface(map_path + map_name + ".xodr", client,
                            scenario_path + scenario_name + ".xml", motionplaner_idx)

        ci.load_map()

        time.sleep(sleep_time)  # time to move your view in carla-window

        hybrid_physics_mode = set_hybrid_physics_mode
        clean_up = set_clean_up
        carla_vehicles = set_carla_vehicles
        carla_pedestrians = set_carla_pedestrians

        ci.setup_carla(hybrid_physics_mode=hybrid_physics_mode)

        start_time = time.time()
        ci.run_scenario(clean_up=clean_up, carla_vehicles=carla_vehicles, carla_pedestrians=carla_pedestrians)
        execution_time = (time.time() - start_time)

        print('Execution time in seconds: ' + str(execution_time))
