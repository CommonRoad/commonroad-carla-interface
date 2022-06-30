import os.path
import time

import carla

from commonroad.scenario.scenario import Scenario
from carlacr.interface.carla_interface import CarlaInterface, MotionPlanner
from carlacr.mode.carla_mode import CarlaMode


class CarlaMotionPlannerMode(CarlaMode):
    def __init__(self, open_drive_map_path: str, cr_scenario_path=None,
                 cr_scenario: Scenario = None,
                 motion_planner: MotionPlanner = None,
                 vehicle_id: int = -1):
        """
            Create Carla Motion Planner mode Interface.
            This allows user to put in a motion planner,
            scenario and a map then have a visualization in carla.
            This API can set up map, scenario.
            Trigger the motion planning at the beginning of the simulation.

            :param open_drive_map_path: full path & filename
            to the according OpenDRIVE map for the scenario
            :param cr_scenario_path: full path & filename to a CommonRoad XML-file
            :param cr_scenario: Scenario object
            :param motion_planner: motion planner library
            :param time_step_delta: time_step_delta within the simulation
            (how much time is between two timesteps for CARLA),
            if None using dt from CommonRoad scenario
            :param vehicle_id: id of the vehicle
            """
        self.carla_client = carla.Client("localhost", 2000)
        if motion_planner is None:
            raise AttributeError(
                "Missing motion planner , motion planner muss "
                "be provided for this mode muss be provided")
        if cr_scenario:
            self.carla_interface = CarlaInterface(cr_scenario=cr_scenario,
                                                  open_drive_map_path=open_drive_map_path,
                                                  carla_client=self.carla_client,
                                                  motion_planner=motion_planner)
        elif cr_scenario_path:
            self.carla_interface = CarlaInterface(cr_scenario_file_path=cr_scenario_path,
                                                  open_drive_map_path=open_drive_map_path,
                                                  carla_client=self.carla_client,
                                                  motion_planner=motion_planner
                                                  )
        else:
            raise AttributeError("Missing scenario and scenario path, one of them muss be provided")
        if not os.path.isfile(cr_scenario_path) and not os.path.isfile(open_drive_map_path):
            raise AttributeError("Can not find scenario file or map file")
        super().__init__(open_drive_map_path=open_drive_map_path, vehicle_id=vehicle_id)

    def visualize(self, sleep_time: int = 10, time_step_delta_real=None, carla_vehicles: int = 0):
        """

        visualize scenario with ego vehicle view if
        ego vehicle != None else run scenario without ego vehicle

        :param sleep_time: time to move your view in carla-window
        :param time_step_delta_real: sets the time
        that will be waited in real time between the timesteps,
        if None the dt of the scenario will be used
        """
        self.carla_interface.load_map()
        time.sleep(sleep_time)
        self.carla_interface.setup_carla(self.time_step_delta)
        self._run_scenario(time_step_delta_real, carla_vehicles=carla_vehicles)

    def _run_scenario(self, time_step_delta_real, carla_vehicles):
        """
        run scenario with current setting

        :param time_step_delta_real: sets the time
        that will be waited in real time between the time steps,
        if None the dt of the scenario will be used
        """
        self.carla_interface.run_scenario(time_step_delta_real=time_step_delta_real,
                                          ego_vehicle=self.ego_vehicle, clean_up=True,
                                          carla_vehicles=carla_vehicles)
