import argparse
import logging
import os
import sys
import time

import carla
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle, ObstacleRole
from commonroad.scenario.scenario import Scenario, Tag
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.file_writer import OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.trajectory import State, Trajectory
from carlacr.configurations.set_configs import set_configs
from carlacr.interface.carla_interface import CarlaInterface

sys.path.append(set_configs().carla_config.carla_examples_path)

logger = logging.getLogger(__name__)
try:
    from examples.no_rendering_mode import game_loop
except IndexError:
    logger.info("Can not find no_rendering_mode")
    raise


class CarlaMode:
    def __init__(self,
                 open_drive_map_path: str,
                 cr_scenario_path=None,
                 cr_scenario: Scenario = None,
                 vehicle_id: int = -1):

        """
        scenario and a map will have a visualization in carla.
        This API can set up map, scenario.

        :param open_drive_map_path: full path & filename
        to the according OpenDRIVE map for the scenario
        :param cr_scenario_path: full path & filename to a CommonRoad XML-file
        :param cr_scenario: Scenario object
        (how much time is between two timesteps for CARLA),
        if None using dt from CommonRoad scenario
        :param vehicle_id: id of the vehicle
        """

        self.carla_client = carla.Client("localhost", 2000)
        if cr_scenario:
            self.carla_interface = CarlaInterface(cr_scenario=cr_scenario,
                                                  open_drive_map_path=open_drive_map_path,
                                                  carla_client=self.carla_client)
        elif cr_scenario_path:
            self.carla_interface = CarlaInterface(cr_scenario_file_path=cr_scenario_path,
                                                  open_drive_map_path=open_drive_map_path,
                                                  carla_client=self.carla_client)
        else:
            raise AttributeError("Missing scenario and scenario path, one of them muss be provided")
        if not os.path.isfile(cr_scenario_path) and not os.path.isfile(open_drive_map_path):
            raise AttributeError("Can not find scenario file or map file")
        self.time_step_delta = None
        self.ego_vehicle = None
        if vehicle_id != -1:
            self.set_ego_vehicle_by_id(vehicle_id)

    def saving_video(self, create_video: bool = True,
                     video_path: str = None,
                     video_name: str = "test",
                     video_as_mp4: bool = False):
        """
        :param create_video: flag for creating video
        :param video_path: path to a folder where the gif will be saved, additionally a folder at "gif_path"/img will be
        created in to save the images used for the gif
        :param video_name: filename for the gif
        :param video_as_mp4: flag to save as mp4 or gif
        """
        self.carla_interface.saving_video(create_video=create_video,
                                          video_path=video_path,
                                          video_name=video_name,
                                          video_as_mp4=video_as_mp4)

    def set_carla_client(self, host: str, port: int):
        """
        set up carla client view

        :param host: host address
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

        :param ego_vehicle: CommonRoad vehicle
        """
        if not ego_vehicle:
            raise ValueError("ego vehicle should not be null")
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
        create CommonRoad moving dynamic obstacle

        :param initial_state: initial state of the dynamic obstacle
        :param trajectory: trajectory of the dynamic obstacle
        :return: static obstacle object
        """
        new_id = self.carla_interface.scenario.generate_object_id()
        shape = Rectangle(2, 4.5)
        initial_state.time_step = 1
        if not trajectory:
            trajectory = Trajectory(initial_time_step=1, state_list=[initial_state])
        prediction = TrajectoryPrediction(trajectory=trajectory, shape=shape)
        obj = DynamicObstacle(obstacle_id=new_id,
                              obstacle_type=ObstacleType.PARKED_VEHICLE,
                              obstacle_shape=Rectangle(2, 4.5),
                              initial_state=initial_state,
                              prediction=prediction)
        return obj

    def visualize(self, sleep_time: int = 10, time_step_delta_real=None, carla_vehicles: int = 0):
        """
        visualize scenario with ego vehicle view if
        ego vehicle != None else run scenario without ego vehicle

        :param carla_vehicles: number of carla vehicles
        :param sleep_time: time to move your view in carla-window
        :param time_step_delta_real: sets the time
        that will be waited in real time between the timesteps,
        if None the dt of the scenario will be used
        """
        self.carla_interface.load_map()
        time.sleep(sleep_time)
        self.carla_interface.setup_carla(self.time_step_delta)
        self._run_scenario(time_step_delta_real, carla_vehicles=carla_vehicles)

    def traffic_generate(self, time_steps, carla_vehicles, sleep_time: int = 0):
        """
        This function allows carla traffic auto generation on a map and create a scenario
        """
        self.carla_interface.load_map()
        time.sleep(sleep_time)
        self.carla_interface.setup_carla(self.time_step_delta)
        self.carla_interface.run_scenario(carla_vehicles=carla_vehicles,
                                          scenario_time_steps=time_steps)

    @staticmethod
    def saving_scenario(file_path, planning_problem_set=PlanningProblemSet(),
                        author=None, affiliation=None, source=None, tags=None):
        tags = {Tag.CRITICAL, Tag.INTERSTATE}
        fw = CommonRoadFileWriter(Scenario(dt=0.1), planning_problem_set,
                                  author, affiliation, source, tags)
        fw.write_to_file(file_path, OverwriteExistingFile.ALWAYS)

    @staticmethod
    def carla_2d_mode():
        """
        Set all necessary parameters for no_rendering_mode.py
        Details please see help
        :return: None
        """
        # Load config files
        config = set_configs()

        # Define arguments that will be received and parsed
        argparser = argparse.ArgumentParser(description=config.config_carla_2d.description)
        argparser.add_argument('-v', '--verbose', action='store_true', dest='debug', help='print debug information')
        argparser.add_argument('--host', metavar='H', default=config.carla_config.host,
                               help='IP of the host server (default: 127.0.0.1)')
        argparser.add_argument('-p', '--port', metavar='P', default=config.carla_config.port, type=int,
                               help='TCP port to listen to (default: 2000)')
        argparser.add_argument('--res', metavar='WIDTHxHEIGHT', default=config.config_carla_2d.res,
                               help='window resolution (default: 1280x720)')
        argparser.add_argument('--filter', metavar='PATTERN', default=config.config_carla_2d.filter,
                               help='actor filter (default: "vehicle.*")')
        argparser.add_argument('--map', metavar='TOWN', default=None, help='start a new episode at the given TOWN')
        argparser.add_argument('--no-rendering', action='store_true', help='switch off server rendering')
        argparser.add_argument('--show-triggers', action='store_true', help='show trigger boxes of traffic signs')
        argparser.add_argument('--show-connections', action='store_true', help='show waypoint connections')
        argparser.add_argument('--show-spawn-points', action='store_true', help='show recommended spawn points')

        # Parse arguments
        args = argparser.parse_args()
        args.description = argparser.description
        args.width, args.height = [int(x) for x in args.res.split('x')]

        # Print server information
        log_level = logging.DEBUG if args.debug else logging.INFO
        logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

        logging.info('listening to server %s:%s', args.host, args.port)
        print(__doc__)

        # Run game loop
        game_loop(args)

    def _run_scenario(self, time_step_delta_real, carla_vehicles):
        """
        run scenario with current setting

        :param time_step_delta_real: sets the time
        that will be waited in real time between the time steps,
        if None the dt of the scenario will be used
        """
        if not self.ego_vehicle:
            self.carla_interface.run_scenario(time_step_delta_real=time_step_delta_real)
        else:
            # run scenario with custom setting
            self.carla_interface.run_scenario(time_step_delta_real=time_step_delta_real,
                                              ego_vehicle=self.ego_vehicle,
                                              clean_up=True,
                                              carla_vehicles=carla_vehicles)
