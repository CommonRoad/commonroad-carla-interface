import time

import carla

from commonroad.scenario.scenario import Scenario
from carlacr.interface.carla_interface import CarlaInterface
from carlacr.mode.carla_mode import CarlaMode
from carlacr.mode.carla_replay_mode import CarlaReplayMode

from commonroad.scenario.scenario import Tag
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.file_writer import OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet

# to convert map to commonroad scenario
# from crdesigner.map_conversion.map_conversion_interface import opendrive_to_commonroad


class CarlaTrafficGenerationMode(CarlaMode):
    """
    Traffic Generation mode used to auto create new scenario from auto generated vehicles by carla
    """

    def __init__(self, open_drive_map_path: str):
        """
        Create Traffic Generation mode. This mode allows carla traffic auto generation on a map and create a scenario

        :param open_drive_map_path: full path & filename to the according OpenDRIVE map for the scenario
        """
        self.carla_client = carla.Client("localhost", 2000)
        super().__init__(open_drive_map_path=open_drive_map_path)
        # load map to scenario
        self.scenario= Scenario(dt=0.1)

        # # load map to commonroad using cdesigner
        # self.scenario = opendrive_to_commonroad(open_drive_map_path)

        self.carla_interface = CarlaInterface(cr_scenario=self.scenario,
                                              open_drive_map_path=open_drive_map_path,
                                              carla_client=self.carla_client
                                              )

    def saving_video(self, create_video: bool = True, video_path: str = None, video_name: str = None,
                     video_asMP4: bool = False):
        raise RuntimeError("this function is not available on this mode")

    def visualize(self, sleep_time: int = 10, time_step_delta_real=None):
        self.carla_interface.load_map()
        time.sleep(sleep_time)
        self.carla_interface.setup_carla(self.time_step_delta)
        self._run_scenario(time_step_delta_real)

    def _run_scenario(self, time_step_delta_real):
        """
        run scenario with current setting

        :param time_step_delta_real: sets the time that will be waited in real time between the timesteps, if None the dt of the scenario will be used
        """
        self.carla_interface.run_scenario(time_step_delta_real=time_step_delta_real)

    def traffic_generate(self, time_steps, carla_vehicles, sleep_time: int = 0):
        self.carla_interface.load_map()
        time.sleep(sleep_time)
        self.carla_interface.setup_carla(self.time_step_delta)
        self.carla_interface.run_scenario(carla_vehicles=carla_vehicles, scenario_time_steps=time_steps)

    def saving_scenario(self, file_path, planning_problem_set=PlanningProblemSet(), author=None, affiliation=None,
                        source=None, tags={Tag.CRITICAL, Tag.INTERSTATE}):
        fw = CommonRoadFileWriter(self.scenario, planning_problem_set, author, affiliation, source, tags)
        fw.write_to_file(file_path, OverwriteExistingFile.ALWAYS)

    def switch_to_replay_mode(self) -> CarlaReplayMode:
        replay = CarlaReplayMode(open_drive_map_path=self.carla_interface.map,
                                 cr_scenario=self.scenario)
        return replay
