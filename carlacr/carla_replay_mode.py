import sys
import time
from datetime import datetime, date

import carla
import pygame
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.obstacle import Obstacle, ObstacleRole

from carlacr.carla_interface import CarlaInterface
from carlacr.commonroad_ego_interface import CommonRoadEgoInterface
from carlacr.commonroad_obstacle_interface import CommonRoadObstacleInterface
from carlacr.synchronous_mode import get_font


class CarlaReplayMode:
    """
    Replay Mode used to easily create visualization and video from scenerio and map
    """

    def __init__(self,
                 scenario_path: str,
                 open_drive_map_path: str):
        """

        :param scenario_path: full path & filename to a CommonRoad XML-file
        :param open_drive_map_path: full path & filename to the according OpenDRIVE map for the scenario

        """
        self.carla_client = carla.Client("localhost", 2000)
        self.ci = CarlaInterface(cr_scenario_file_path=scenario_path,
                                 open_drive_map=open_drive_map_path,
                                 carla_client=self.carla_client
                                 )
        self.time_step_delta = 240
        self.ego_vehicle = None

    def set_carla_client(self, host: str, port: int):
        """
        set up carla client view

        :param host : host adresse
        :param port: port number
        """
        self.carla_client = carla.Client(host, port)
        self.ci.client = self.carla_client

    def set_ego_vehicle(self, ego_vehicle: Obstacle = None):
        """
        set up ego_vehicle view

        :param veh_id: commonroad id of the vehicle
        """
        if not ego_vehicle:
            if self.ci.scenario.dynamic_obstacles:
                self.ego_vehicle = self.ci.scenario.dynamic_obstacles.pop()
            elif self.ci.scenario.static_obstacles:
                self.ego_vehicle = self.ci.scenario.static_obstacles.pop()
        else:
            self.ego_vehicle = ego_vehicle

    def find_car_with_CR_ID(self, veh_id: int = 0):
        """

        find vehicle in scenario with commonroad ID
        :param veh_id: commonroad id of the vehicle
        :return: the vehicle if found else return None
        """
        for vehicle in self.ci.scenario.dynamic_obstacles:
            if vehicle.obstacle_id == veh_id:
                return vehicle
        for vehicle in self.ci.scenario.static_obstacles:
            if vehicle.obstacle_id == veh_id:
                return vehicle
        return None

    def create_static_obstacles_ego(self):
        pass

    def visualize(self, sleep_time: int = 10, time_step_delta_real=None,
                  saving_video: bool = False,
                  asMP4: bool = True,
                  video_path: str = None,
                  file_name: str = "replay_mode"):
        """

        visualize scenario with ego vehicle view if ego vehicle != None else run scenario without ego vehicle

        :param time_step_delta_real: sets the time that will be waited in real time between the timesteps, if None the dt of the scenario will be used
        :param saving_video: if True a gif will be created (only when a MotionPlanner is provided)
        :param video_path: path to a folder where the gif will be saved, additionally a folder at "gif_path"/img will be created in to save the images used for the gif
        :param file_name: filename for the gif
        :param asMP4: flag to save as mp4 or gif
        """
        self.ci.load_map()
        time.sleep(sleep_time)  # time to move your view in carla-window
        self.ci.setup_carla(self.time_step_delta)
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
            video_path += f"/{self.ci.scenario.scenario_id}_{current_date}_{current_time}"
        if not self.ego_vehicle:
            self.ci.run_scenario(time_step_delta_real=time_step_delta_real)
        else:
            # run scenario with custom setting
            self.ci.run_scenario_with_ego_vehicle(time_step_delta_real, self.ego_vehicle, create_gif=saving_video,
                                                  gif_path=video_path, asMP4=asMP4)
