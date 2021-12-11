import os.path
import time
from datetime import datetime, date
from typing import List

import carla

from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle, ObstacleRole
from commonroad.scenario.trajectory import State, Trajectory
from commonroad.scenario.scenario import Scenario
from carlacr.carla_interface import CarlaInterface


class CarlaReplayMode:
    """
    Replay Mode used to easily create visualization and video from scenerio and map
    """

    def __init__(self, open_drive_map_path: str, cr_scenario_path: str = None, cr_scenario: Scenario = None,
                 vehicle_id: id = -1):
        """
        Create Replay mode Interface

        :param open_drive_map_path: full path & filename to the according OpenDRIVE map for the scenario
        :param cr_scenario_path: full path & filename to a CommonRoad XML-file
        :param cr_scenario: Scenario object
        :param time_step_delta: time_step_delta within the simulation (how much time is between two timesteps for CARLA), if None using dt from CommonRoad scenario
        :param vehicle_id: id of the vehicle
        """
        self.carla_client = carla.Client("localhost", 2000)
        if cr_scenario:
            self.carla_interface = CarlaInterface(cr_scenario=cr_scenario,
                                                  open_drive_map=open_drive_map_path,
                                                  carla_client=self.carla_client
                                                  )
        elif cr_scenario_path:
            self.carla_interface = CarlaInterface(cr_scenario_file_path=cr_scenario_path,
                                                  open_drive_map=open_drive_map_path,
                                                  carla_client=self.carla_client
                                                  )
        else:
            raise AttributeError("Missing scenario and scenario path, one of them muss be provided")
        if not os.path.isfile(cr_scenario_path) and not os.path.isfile(open_drive_map_path):
            raise AttributeError("Can not find scenario file or map file")
        self.time_step_delta = None
        self.ego_vehicle = None
        if vehicle_id!=-1 :
            self.set_ego_vehicle_by_id(vehicle_id)

    def set_carla_client(self, host: str, port: int):
        """
        set up carla client view

        :param host: host adresse
        :param port: port number
        """
        self.carla_client = carla.Client(host, port)
        self.carla_interface.client = self.carla_client

    def set_ego_vehicle_by_id(self, veh_id: int):
        vehicle = self.obstacle_by_id(veh_id)
        if not vehicle:
            raise ValueError("There are no vehicle with the given id")
        self.set_ego_vehicle(vehicle)

    def set_ego_vehicle(self, ego_vehicle: DynamicObstacle):
        """
        set up ego_vehicle view

        :param ego_vehicle: commonroad vehicle
        """
        if not ego_vehicle:
            raise ValueError("ego vehicle should not be null")
        else:
            if ego_vehicle.obstacle_role != ObstacleRole.DYNAMIC:
                raise AttributeError("ego vehicle muss be dynamic")
            self.ego_vehicle = ego_vehicle

    def obstacle_by_id(self, veh_id: int):
        """

        find vehicle in scenario with commonroad ID
        :param veh_id: commonroad id of the vehicle
        :return: the vehicle if found else return None
        """

        return self.carla_interface.scenario.obstacle_by_id(veh_id)

    def create_dynamic_obstacles_ego(self, initial_state: State,
                                     trajectory: Trajectory = None) -> DynamicObstacle:
        """
        create commonroad moving dynamic obstacle

        :param initial_state: initial state of the dynamic obstacle
        :param trajectory: trajectory of the dynamic obstacle
        :return: static obstacle object
        """
        new_id = self.carla_interface.scenario.generate_object_id()
        shape = Rectangle(2, 4.5)
        initial_state.time_step = int(self.carla_interface._calc_max_timestep())
        if not trajectory:
            trajectory = Trajectory(initial_time_step=1, state_list=[initial_state])
        prediction = TrajectoryPrediction(trajectory=trajectory, shape=shape)
        obj = DynamicObstacle(obstacle_id=new_id,
                              obstacle_type=ObstacleType.PARKED_VEHICLE,
                              obstacle_shape=Rectangle(2, 4.5),
                              initial_state=initial_state,
                              prediction=prediction)
        return obj

    def visualize(self, sleep_time: int = 10, time_step_delta_real=None,
                  saving_video: bool = False,
                  asMP4: bool = False,
                  video_path: str = None,
                  file_name: str = "replay_mode"):
        """

        visualize scenario with ego vehicle view if ego vehicle != None else run scenario without ego vehicle

        :param sleep_time: time to move your view in carla-window
        :param time_step_delta_real: sets the time that will be waited in real time between the timesteps, if None the dt of the scenario will be used
        :param saving_video: if True a gif will be created (only when a MotionPlanner is provided)
        :param video_path: path to a folder where the gif will be saved, additionally a folder at "gif_path"/img will be created in to save the images used for the gif
        :param file_name: filename for the gif
        :param asMP4: flag to save as mp4 or gif
        """
        self.carla_interface.load_map()
        time.sleep(sleep_time)
        self.carla_interface.setup_carla(self.time_step_delta)
        self._run_scenario(time_step_delta_real,
                           saving_video,
                           asMP4,
                           video_path,
                           file_name)

    def _run_scenario(self, time_step_delta_real,
                      saving_video: bool = False,
                      asMP4: bool = False,
                      video_path: str = None,
                      file_name: str = "replay_mode"):
        """
        run scenario with current setting

        :param time_step_delta_real: sets the time that will be waited in real time between the timesteps, if None the dt of the scenario will be used
        :param saving_video: if True a gif will be created (only when a MotionPlanner is provided)
        :param video_path: path to a folder where the gif will be saved, additionally a folder at "gif_path"/img will be created in to save the images used for the gif
        :param file_name: filename for the gif
        :param asMP4: flag to save as mp4 or gif
        """
        if saving_video:
            now = datetime.now()
            today = date.today()
            current_time = now.strftime("%H_%M_%S")
            current_date = today.strftime("%d_%m_%Y")
            video_path += f"/{self.carla_interface.scenario.scenario_id}_{current_date}_{current_time}"
        if not self.ego_vehicle:
            self.carla_interface.run_scenario(time_step_delta_real=time_step_delta_real)
        else:
            # run scenario with custom setting
            self.carla_interface.run_scenario_with_ego_vehicle(time_step_delta_real, self.ego_vehicle,
                                                               create_gif=saving_video,
                                                               gif_path=video_path, asMP4=asMP4)
