import os.path
import time

import carla

from commonroad.scenario.scenario import Scenario
from carlacr.interface.carla_interface import CarlaInterface
from carlacr.mode.carla_mode import CarlaMode


class CarlaReplayMode(CarlaMode):
    """
    Replay Mode used to easily create visualization and video from scenario and map
    """

    def __init__(self, open_drive_map_path: str, cr_scenario_path: str = None, cr_scenario: Scenario = None,
                 vehicle_id: id = -1):
        """
        Create Replay mode Interface

        :param open_drive_map_path: full path & filename to the according OpenDRIVE map for the scenario
        :param cr_scenario_path: full path & filename to a CommonRoad XML-file
        :param cr_scenario: Scenario object
        :param time_step_delta: time_step_delta within the simulation (how much time is between two timesteps for CARLA)
        if None using dt from CommonRoad scenario
        :param vehicle_id: id of the vehicle
        """
        self.carla_client = carla.Client("localhost", 2000)
        if cr_scenario:
            self.carla_interface = CarlaInterface(cr_scenario=cr_scenario,
                                                  open_drive_map_path=open_drive_map_path,
                                                  carla_client=self.carla_client,
                                                  )
        elif cr_scenario_path:
            self.carla_interface = CarlaInterface(cr_scenario_file_path=cr_scenario_path,
                                                  open_drive_map_path=open_drive_map_path,
                                                  carla_client=self.carla_client
                                                  )
        else:
            raise AttributeError("Missing scenario and scenario path, one of them muss be provided")
        if cr_scenario is None and not os.path.isfile(cr_scenario_path) and not os.path.isfile(open_drive_map_path):
            raise AttributeError("Can not find scenario file or map file")
        super().__init__(open_drive_map_path=open_drive_map_path, cr_scenario_file_path=cr_scenario_path,
                         vehicle_id=vehicle_id)

    def visualize(self, sleep_time: int = 10, time_step_delta_real=None):
        """

        visualize scenario with ego vehicle view if ego vehicle != None else run scenario without ego vehicle

        :param sleep_time: time to move your view in carla-window
        :param time_step_delta_real: sets the time that will be waited in real time between the timesteps, if None the dt of the scenario will be used
        """
        self.carla_interface.load_map()
        time.sleep(sleep_time)
        self.carla_interface.setup_carla(self.time_step_delta)
        self._run_scenario(time_step_delta_real)

    def _run_scenario(self, time_step_delta_real):
        """
        run scenario with current setting

        :param time_step_delta_real: sets the time that will be waited in real time between the timesteps, if None the dt of the scenario will be used
        """
        if not self.ego_vehicle:
            self.carla_interface.run_scenario(time_step_delta_real=time_step_delta_real)
        else:
            # run scenario with custom setting
            self.carla_interface.run_scenario(time_step_delta_real=time_step_delta_real,
                                              ego_vehicle=self.ego_vehicle)
