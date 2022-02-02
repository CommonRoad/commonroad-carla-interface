#!/usr/bin/env python

import os
import sys
import time
from datetime import date, datetime
from typing import List
import logging
from copy import deepcopy

logger = logging.getLogger(__name__)

import carla
import commonroad.scenario.obstacle
import pygame
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.obstacle import ObstacleRole, DynamicObstacle
from carlacr.helper.carla_motion_planner_helper import calc_max_timestep
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory, State
from commonroad.prediction.prediction import TrajectoryPrediction

try:
    from motion_planner.motion_planner import MotionPlanner
except ModuleNotFoundError:
    class MotionPlanner:
        pass

from carlacr.interface.carla_pedestrian_handler import CarlaPedestrianHandler
from carlacr.interface.carla_vehicle_interface import CarlaVehicleInterface
from carlacr.interface.commonroad_ego_interface import CommonRoadEgoInterface
from carlacr.interface.commonroad_obstacle_interface import (ApproximationType, CommonRoadObstacleInterface)
from carlacr.helper.gif_creator import Gif_Creator
from carlacr.helper.synchronous_mode import (CarlaSyncMode, draw_image, get_font, should_quit)
from carlacr.helper.carla_motion_planner_helper import calc_max_timestep, divide_scenario


class CarlaInterface:
    """
    Main class of the CommonRoad-CARLA-Interface
    """

    def __init__(self, open_drive_map_path: str, carla_client: carla.Client, cr_scenario_file_path: str = None,
                 motion_planner: MotionPlanner = None, mpl_update_n: int = 5, cr_scenario: Scenario = None):
        """

        :param cr_scenario_file_path: full path & filename to a CommonRoad XML-file
        :param open_drive_map_path: full path & filename to the according OpenDRIVE map for the scenario
        :param carla_client: carla.Client() object connected to the simulation
        :param motion_planner: a MotionPlanner object from the commonroad-motion-planning-library
        :param mpl_update_n: (in DEV) update interval at which rate the motion planner receives updated CommonRoad
        dynamic obstacles of the CARLA generated vehicles & pedestrians
        :param cr_scenario: Scenario obj
        """
        self.map = open_drive_map_path
        if not cr_scenario:
            self.scenario, self.planning_problem_set = CommonRoadFileReader(cr_scenario_file_path).open()
        else:
            self.scenario = cr_scenario
        self.motion_planner = motion_planner
        self.client = carla_client
        self.mpl_update_n = mpl_update_n
        self.create_video: bool = False
        self.video_path: str = None
        self.video_name: str = None
        self.video_asMP4: bool = False

    def saving_video(self, create_video: bool = True, video_path: str = None, video_name: str = "test",
                     video_asMP4: bool = False):
        """
        :param create_video: flag for creating video
        :param video_path: path to a folder where the gif will be saved, additionally a folder at "gif_path"/img will
        be created in to save the images used for the gif
        :param video_name: filename for the gif
        :param video_asMP4: flag to save as mp4 or gif
        """
        # should add some checking
        if create_video:
            if not os.path.exists(video_path):
                raise AttributeError("video path not found")
        self.create_video = create_video
        self.video_path = video_path
        self.video_name = video_name
        self.video_asMP4 = video_asMP4

    def setup_carla(self, time_step_delta: int = None, tm_port=8000, hybrid_physics_mode=False):
        """
        Configures CARLA (self.client)

        :param time_step_delta: time_step_delta within the simulation (how much time is between two timesteps for
        CARLA), if None using dt from CommonRoad scenario
        :param tm_port: port of the CARLA traffic manager
        :param hybrid_physics_mode: sets hybrid_physics_mode in CARLA
        """
        self.client.set_timeout(2.0)
        world = self.client.get_world()
        # Synchrony:
        """
            Warning: If synchronous mode is enabled, and there is a Traffic Manager running, this must be set to sync 
            mode too. Read this to learn how to do it.
            See: https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/#setting-synchronous-mode
        """
        if time_step_delta:
            delta_seconds = 1.0 / time_step_delta
        else:
            delta_seconds = self.scenario.dt
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = delta_seconds
        world.apply_settings(settings)
        traffic_manager = self.client.get_trafficmanager(tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(1.0)
        traffic_manager.global_percentage_speed_difference(0)
        traffic_manager.set_hybrid_physics_mode(hybrid_physics_mode)
        traffic_manager.set_synchronous_mode(True)

    def load_map(self):
        """ Loads OpenDRIVE Map (self.map) into CARLA. Based on CARLAs program: PythonAPI/util/config.py
            # Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
            # Barcelona (UAB).
            #
            # This work is licensed under the terms of the MIT license.
            # For a copy, see <https://opensource.org/licenses/MIT>.
        """
        if os.path.exists(self.map):
            with open(self.map, encoding='utf-8') as od_file:
                try:
                    data = od_file.read()
                except OSError:
                    logger.error('file could not be readed.')
                    sys.exit()
            logger.debug('load opendrive map %r.' % os.path.basename(self.map))
            vertex_distance = 2.0  # in meters
            max_road_length = 500.0  # in meters
            wall_height = 1.0  # in meters
            extra_width = 0.6  # in meters
            world = self.client.generate_opendrive_world(data, carla.OpendriveGenerationParameters(
                    vertex_distance=vertex_distance, max_road_length=max_road_length, wall_height=wall_height,
                    additional_width=extra_width, smooth_junctions=True, enable_mesh_visibility=True))

        else:
            logging.error('file not found.')

    def _calc_max_timestep(self) -> int:
        """
        Calculates maximal time step of current scenario
        """
        self.max_timestep = calc_max_timestep(self.scenario)
        return self.max_timestep

    def _run_scenario_with_mpl(self, clean_up=True, time_step_delta_real=None, carla_vehicles=0, carla_pedestrians=0,
                               ego_vehicle=None):
        """
        Runs the CommonRoad Scenario in CARLA (with MPL & PyGame)

        :param clean_up: if True destroys all created actors in the CARLA simulation
        :param time_step_delta_real: sets the time that will be waited in real time between the timesteps,
        if None the dt of the scenario will be used
        :param carla_vehicles: maximum number of vehicles that should be created & controlled by CARLA additional to
        the objects defined in the scenario
        :param carla_pedestrians: maximum number of pedestrians that should be created & controlled by CARLA
        additional to the objects defined in the scenario
        """
        interface_obstacles, carla_interface_obstacles, carla_controlled_obstacles, carla_contr_obs_classes, \
        ego_interface_list, batch = [], [], [], [], [], []


        carla_controlled_imported = False

        pygame.init()

        display = pygame.display.set_mode((800, 600), pygame.HWSURFACE | pygame.DOUBLEBUF)
        font = get_font()
        clock = pygame.time.Clock()

        world = self.client.get_world()
        # Load all dynamic obstacles from scenario into CARLA:
        # dynamic_obstacles = scenario.obstacles_by_role_and_type(obstacle_role=ObstacleRole.DYNAMIC,
        # obstacle_type=ObstacleType.CAR)
        dynamic_obstacles = self.scenario.dynamic_obstacles
        dynamic_obstacles += self.scenario.static_obstacles

        # real world time step delta 
        if time_step_delta_real:
            time_between_ticks = time_step_delta_real
        else:
            time_between_ticks = self.scenario.dt

        if ego_vehicle in dynamic_obstacles:
            dynamic_obstacles.remove(ego_vehicle)

        # Create Interface
        for obstacle in dynamic_obstacles:
            obs = CommonRoadObstacleInterface(obstacle)
            interface_obstacles.append(obs)

        # Create Motion Planner Vehicle
        if self.motion_planner:
            motion_planner_vehicle = CommonRoadEgoInterface(client=self.client,
                                                            planning_problem=self.motion_planner.trajectory_planner.planning_problem,
                                                            trajectory=Trajectory(self.motion_planner.trajectory_planner.planning_problem.
                                                                                  initial_state.time_step,
                                                                       [self.motion_planner.trajectory_planner.planning_problem.initial_state]))
            self.motion_planner.initialize_plan()
            ego_interface_list.append(motion_planner_vehicle)

            try:
                actor = motion_planner_vehicle.spawn(world)
                if actor:
                    carla_interface_obstacles.append((motion_planner_vehicle, actor))
            except Exception as e:
                logger.error(e, exc_info=sys.exc_info())
        # Create ego
        if ego_vehicle:
            ego = self._create_ego_vehicle(ego_vehicle=ego_vehicle, ego_interface_list=ego_interface_list,
                                           carla_interface_obstacles=carla_interface_obstacles)
        else:
            raise RuntimeError("Can not create ego")

        i = 0  # time-step counter
        max_timesteps = self._calc_max_timestep()
        logger.debug(max_timesteps)
        with CarlaSyncMode(world, ego.actor_list[0], fps=30) as sync_mode:
            while i <= max_timesteps:
                if should_quit():
                    return
                clock.tick()
                snapshot, image_rgb = sync_mode.tick(timeout=2.0)
                if self.create_video and self.video_path:
                    image_rgb.save_to_disk('%s/img/%.6d.jpg' % (self.video_path, image_rgb.frame))
                try:  # Simulation
                    # ego vehicle
                    if ego.is_spawned and ego != motion_planner_vehicle:
                        try:
                            state = ego.trajectory.state_at_time_step(i)
                            if state:
                                ego.update_position_by_time(world, state)
                        except Exception as e:
                            logger.debug("Error when update vehicle")
                            logger.debug(ego)
                            logger.error(e, exc_info=sys.exc_info())


                    # CommonRoad controlled:
                    self._control_commonroad_obstacles(interface_obstacles, carla_interface_obstacles, i)

                    # CARLA controlled:
                    if not carla_controlled_imported:
                        pedestrian_handler = self._create_carla_obstacles(carla_vehicles, carla_contr_obs_classes,
                                                                          batch, carla_controlled_obstacles,
                                                                          carla_pedestrians)
                        carla_controlled_imported = True
                        self._wait_for_carla_vehicle(time_between_ticks)

                    else:
                        self._control_carla_obstacles(carla_contr_obs_classes, i)


                    # Motion planner Vehicle
                    # If n-th timestep update obstacles for mpl
                    if i % self.motion_planner.trajectory_planner.replanning_frequency == 0:
                        found_optimal_trajectory = self._update_scenario_motion_planner(carla_contr_obs_classes, motion_planner_vehicle)
                        if found_optimal_trajectory:
                            motion_planner_vehicle.set_trajectory(Trajectory(0,self.motion_planner.trajectory_planner.record_state_list))
                        else:
                            break

                    if motion_planner_vehicle.is_spawned:
                        try:
                            state = motion_planner_vehicle.trajectory.state_at_time_step(i)
                            if state:
                                motion_planner_vehicle.update_position_by_time(world, state)
                        except Exception as e:
                            logger.debug("Error when update vehicle")
                            logger.debug(motion_planner_vehicle)
                            logger.error(e, exc_info=sys.exc_info())
                    fps = round(1.0 / snapshot.timestamp.delta_seconds)
                    draw_image(display, image_rgb)
                    display.blit(font.render('% 5d FPS (real)' % clock.get_fps(), True, (255, 255, 255)), (8, 10))
                    display.blit(font.render('% 5d FPS (simulated)' % fps, True, (255, 255, 255)), (8, 28))
                    pygame.display.flip()

                    # advance time:
                    logger.debug("Timestep: " + str(i))
                    world.tick()
                    time.sleep(time_between_ticks)
                    i += 1
                except KeyboardInterrupt:
                    self._end_simmulation_error(clean_up=clean_up, ego_interface_list=ego_interface_list,
                                                interface_obstacles=interface_obstacles,
                                                carla_controlled_obstacles=carla_controlled_obstacles,
                                                pedestrian_handler=pedestrian_handler)
                except Exception as e:
                    logger.error(e, exc_info=sys.exc_info())
        # Create video
        self._create_video()
        settings = world.get_settings()
        # Stop synchronous mode to be able to investigate CARLA without a Script running
        settings.synchronous_mode = False
        world.apply_settings(settings)

        # add new obs to the scenario
        self.update_vehicle_to_scenario(carla_contr_obs_classes)

        # clean up
        if clean_up:
            self._clean_up_carla(ego_interface_list, interface_obstacles, carla_controlled_obstacles,
                                 pedestrian_handler)

    def _run_scenario_without_mpl(self, clean_up=True, time_step_delta_real=0.05, carla_vehicles=0, carla_pedestrians=0,
                                  scenario_time_steps: int = 0):
        """
        Runs the CommonRoad Scenario in CARLA without mpl

        :param clean_up: if True destroys all created actors in the CARLA simulation
        :param time_step_delta_real: sets the time that will be waited in real time between the timesteps,
        if None the dt of the scenario will be used
        :param carla_vehicles: maximum number of vehicles that should be created & controlled by CARLA additional to
        the objects defined in the scenario
        :param carla_pedestrians: maximum number of pedestrians that should be created & controlled by CARLA
        additional to the objects defined in the scenario
        """
        interface_obstacles, carla_interface_obstacles, carla_controlled_obstacles, carla_contr_obs_classes, \
        ego_interface_list, batch = [], [], [], [], [], []

        carla_controlled_imported = False

        world = self.client.get_world()
        # Load all dynamic obstacles from scenario into CARLA:
        # dynamic_obstacles = scenario.obstacles_by_role_and_type(obstacle_role=ObstacleRole.DYNAMIC,
        # obstacle_type=ObstacleType.CAR)
        dynamic_obstacles = self.scenario.dynamic_obstacles
        dynamic_obstacles += self.scenario.static_obstacles

        # real world time step delta
        if time_step_delta_real:
            time_between_ticks = time_step_delta_real
        else:
            time_between_ticks = self.scenario.dt

        # Create Interface
        for obstacle in dynamic_obstacles:
            obs = CommonRoadObstacleInterface(obstacle)
            interface_obstacles.append(obs)

        i = 0  # time-step counter
        max_timesteps = max(self._calc_max_timestep(), scenario_time_steps)
        while i <= max_timesteps:
            try:  # Simulation
                # CommonRoad controlled:
                self._control_commonroad_obstacles(interface_obstacles, carla_interface_obstacles, i)

                # CARLA controlled:
                if not carla_controlled_imported:
                    pedestrian_handler = self._create_carla_obstacles(carla_vehicles, carla_contr_obs_classes, batch,
                                                                      carla_controlled_obstacles, carla_pedestrians)
                    carla_controlled_imported = True
                    self._wait_for_carla_vehicle(time_between_ticks)

                else:
                    self._control_carla_obstacles(carla_contr_obs_classes, i)

                # advance time:
                logger.debug("Timestep: " + str(i))
                world.tick()
                time.sleep(time_between_ticks)
                i += 1
            except KeyboardInterrupt:
                # Create GIF
                settings = world.get_settings()
                # Stop synchronous mode to be able to investigate CARLA without a Script running
                settings.synchronous_mode = False
                world.apply_settings(settings)
                # clean up
                if clean_up:
                    self._clean_up_carla(ego_interface_list, interface_obstacles, carla_controlled_obstacles,
                                         pedestrian_handler)
                sys.exit(0)
            except Exception as e:
                logger.error(e, exc_info=sys.exc_info())

        settings = world.get_settings()
        # Stop synchronous mode to be able to investigate CARLA without a Script running
        settings.synchronous_mode = False
        world.apply_settings(settings)
        # add new obs to the scenario
        self.update_vehicle_to_scenario(carla_contr_obs_classes)
        # clean up
        if clean_up:
            self._clean_up_carla(ego_interface_list, interface_obstacles, carla_controlled_obstacles,
                                 pedestrian_handler)

    def run_scenario_with_ego_vehicle(self, time_step_delta_real, ego_vehicle: commonroad.scenario.obstacle.Obstacle,
                                      carla_vehicles=0, carla_pedestrians=0):
        """
        run scenario with ego vehicle setting

        :param time_step_delta_real: sets the time that will be waited in real time between the timesteps,
        if None the dt of the scenario will be used
        :param ego_vehicle: vehicle that will be used as ego_vehicle
        :param carla_vehicles: maximum number of vehicles that should be created & controlled by CARLA additional to
        the objects defined in the scenario
        :param carla_pedestrians: maximum number of pedestrians that should be created & controlled by CARLA
        additional to the objects defined in the scenario
        """
        interface_obstacles, carla_interface_obstacles, carla_controlled_obstacles, carla_contr_obs_classes, \
        ego_interface_list, batch = [], [], [], [], [], []

        carla_controlled_imported = False

        pygame.init()
        display = pygame.display.set_mode((800, 600), pygame.HWSURFACE | pygame.DOUBLEBUF)
        font = get_font()
        clock = pygame.time.Clock()
        world = self.client.get_world()
        # Load all dynamic obstacles from scenario into CARLA:
        dynamic_obstacles = self.scenario.dynamic_obstacles
        dynamic_obstacles += self.scenario.static_obstacles

        # real world time step delta
        if time_step_delta_real:
            time_between_ticks = time_step_delta_real
        else:
            time_between_ticks = self.scenario.dt

        if ego_vehicle in dynamic_obstacles:
            dynamic_obstacles.remove(ego_vehicle)

        # Create Interface
        for obstacle in dynamic_obstacles:
            obs = CommonRoadObstacleInterface(obstacle)
            interface_obstacles.append(obs)

        i = 0  # time-step counter
        max_timesteps = self._calc_max_timestep()
        logger.debug(max_timesteps)
        # Create ego
        ego = self._create_ego_vehicle(ego_vehicle=ego_vehicle, ego_interface_list=ego_interface_list,
                                       carla_interface_obstacles=carla_interface_obstacles)

        with CarlaSyncMode(world, ego.actor_list[0], fps=30) as sync_mode:
            while i <= max_timesteps:
                # set up gif
                if should_quit():
                    return
                clock.tick()
                snapshot, image_rgb = sync_mode.tick(timeout=2.0)
                if self.create_video and self.video_path:
                    image_rgb.save_to_disk('%s/img/%.6d.jpg' % (self.video_path, image_rgb.frame))
                try:  # Simulation
                    # Ego Vehicle
                    if ego.is_spawned:
                        try:
                            state = ego.trajectory.state_at_time_step(i)
                            if state:
                                ego.update_position_by_time(world, state)
                        except Exception as e:
                            logger.debug("Error when update vehicle")
                            logger.debug(ego)
                            logger.error(e, exc_info=sys.exc_info())

                    fps = round(1.0 / snapshot.timestamp.delta_seconds)
                    draw_image(display, image_rgb)
                    display.blit(font.render('% 5d FPS (real)' % clock.get_fps(), True, (255, 255, 255)), (8, 10))
                    display.blit(font.render('% 5d FPS (simulated)' % fps, True, (255, 255, 255)), (8, 28))
                    pygame.display.flip()

                    # CommonRoad controlled:
                    self._control_commonroad_obstacles(interface_obstacles, carla_interface_obstacles, i)

                    # CARLA controlled:
                    if not carla_controlled_imported:
                        pedestrian_handler = self._create_carla_obstacles(carla_vehicles, carla_contr_obs_classes,
                                                                          batch, carla_controlled_obstacles,
                                                                          carla_pedestrians)
                        carla_controlled_imported = True
                        self._wait_for_carla_vehicle(time_between_ticks)

                    # advance time:
                    logger.debug("Timestep: " + str(i))
                    world.tick()
                    time.sleep(time_between_ticks)
                    i += 1
                except KeyboardInterrupt:
                    self._end_simmulation_error()
                except Exception as e:
                    logger.error(e, exc_info=sys.exc_info())
        # Create video
        self._create_video()
        settings = world.get_settings()
        # Stop synchronous mode to be able to investigate CARLA without a Script running
        settings.synchronous_mode = False
        world.apply_settings(settings)

        # clean up
        self._clean_up_carla(ego_interface_list, interface_obstacles, carla_controlled_obstacles, pedestrian_handler)

    def run_scenario(self, clean_up=True, time_step_delta_real=None, carla_vehicles=0, carla_pedestrians=0,
                     ego_vehicle=None, scenario_time_steps: int = 0):
        """
        Runs the CommonRoad Scenario in CARLA. Splits up between a simulation with and without a motion planner

        :param clean_up: if True destroys all created actors in the CARLA simulation
        :param time_step_delta_real: sets the time that will be waited in real time between the timesteps,
        if None the dt of the scenario will be used
        :param carla_vehicles: maximum number of vehicles that should be created & controlled by CARLA additional to
        the objects defined in the scenario
        :param carla_pedestrians: maximum number of pedestrians that should be created & controlled by CARLA
        additional to the objects defined in the scenario
        :param ego_vehicle: the vehicle to be view
        :param scenario_time_steps: minimum scenario time steps, use to prolong a scenario with carla traffic generation
        """
        if self.video_path:
            now = datetime.now()
            today = date.today()
            current_time = now.strftime("%H_%M_%S")
            current_date = today.strftime("%d_%m_%Y")
            video_folder = f"/{self.scenario.scenario_id}_{current_date}_{current_time}"
            self.video_path += video_folder

        if self.motion_planner:
            self._run_scenario_with_mpl(clean_up=clean_up, time_step_delta_real=time_step_delta_real,
                                        ego_vehicle=ego_vehicle, carla_vehicles=carla_vehicles,
                                        carla_pedestrians=carla_pedestrians)

        else:
            if self.create_video and not ego_vehicle:
                logger.debug("GIFs can only be created when a ego_vehicle or motion planner is provided!")
            if ego_vehicle:
                self.run_scenario_with_ego_vehicle(time_step_delta_real=time_step_delta_real, ego_vehicle=ego_vehicle,
                                                   carla_pedestrians=carla_pedestrians, carla_vehicles=carla_vehicles)
            else:
                self._run_scenario_without_mpl(clean_up=clean_up, time_step_delta_real=time_step_delta_real,
                                               carla_pedestrians=carla_pedestrians, carla_vehicles=carla_vehicles,
                                               scenario_time_steps=scenario_time_steps)
        if self.video_path:
            self.video_path = self.video_path[:len(self.video_path) - len(video_folder)]

    def _clean_up_carla(self, ego_interface_list: List[CommonRoadEgoInterface],
                        interface_obstacles: List[CommonRoadObstacleInterface], carla_controlled_obstacles,
                        pedestrian_handler: CarlaPedestrianHandler):
        for ego in ego_interface_list:
            ego.destroy_carla_actor(self.client.get_world())
        for obs in interface_obstacles:
            obs.destroy_carla_obstacle(self.client.get_world())
        self.client.apply_batch([carla.command.DestroyActor(x) for x in carla_controlled_obstacles])
        if pedestrian_handler:
            pedestrian_handler.destroy()

    def _control_commonroad_obstacles(self, interface_obstacles: List[CommonRoadObstacleInterface],
                                      carla_interface_obstacles: List, curr_time_step: int):
        deleted_obs = []
        for obs in interface_obstacles:
            if not obs.is_spawned:
                try:
                    actor = obs.spawn(self.client.get_world(), ApproximationType.LENGTH)
                    if actor:
                        carla_interface_obstacles.append((obs, actor))
                except Exception as e:
                    logger.error(e, exc_info=sys.exc_info())
            else:
                try:
                    if obs.role == ObstacleRole.DYNAMIC and obs.trajectory.state_at_time_step(curr_time_step):
                        obs.update_position_by_time(self.client.get_world(),
                                                    obs.trajectory.state_at_time_step(curr_time_step))
                    elif obs.role != ObstacleRole.STATIC:
                        deleted_obs.append(obs)
                        obs.destroy_carla_obstacle(self.client.get_world())
                except Exception as e:
                    logger.error(e, exc_info=sys.exc_info())
        for obs in deleted_obs:
            interface_obstacles.remove(obs)

    def _create_carla_obstacles(self, carla_vehicles: int, carla_contr_obs_classes: List[CarlaVehicleInterface],
                                batch: List[carla.command.SpawnActor], carla_controlled_obstacles: List,
                                carla_pedestrians: int) -> List[DynamicObstacle]:
        for numb_veh in range(0, carla_vehicles):
            q = CarlaVehicleInterface(self.scenario, self.client)
            actor = q.get_spawnable()
            if actor:
                carla_contr_obs_classes.append(q)
                batch.append(actor)
        x = 0
        for response in self.client.apply_batch_sync(batch):
            if response.error:
                logger.debug(response.error)
            else:
                carla_controlled_obstacles.append(response.actor_id)
                carla_obs = carla_contr_obs_classes[x]
                carla_obs.update_after_spawn(cr_id=self.scenario.generate_object_id(), actor_id=response.actor_id)
                x += 1
        pedestrian_handler = CarlaPedestrianHandler(self.scenario, self.client, carla_pedestrians)
        pedestrian_handler.spawn()
        logger.debug(pedestrian_handler)
        return pedestrian_handler

    def _control_carla_obstacles(self, carla_contr_obs_classes: List[CarlaVehicleInterface], current_time_step: int):
        for obs in carla_contr_obs_classes:
            if obs.dynamic_obstacle:
                obs.dynamic_obstacle.prediction.trajectory.state_list.append(obs.get_cr_state(current_time_step))
            elif obs.commonroad_id:
                obs.create_cr_dynamic_obstacle()

    def _create_ego_vehicle(self, ego_vehicle: DynamicObstacle, ego_interface_list: List[DynamicObstacle],
                            carla_interface_obstacles: List):
        ego = CommonRoadEgoInterface(client=self.client,
                                     trajectory=ego_vehicle.prediction.trajectory,
                                     initial_state=ego_vehicle.initial_state,
                                     size=(ego_vehicle.obstacle_shape.length, ego_vehicle.obstacle_shape.width, 0))

        ego_interface_list.append(ego)
        try:
            actor = ego.spawn(self.client.get_world())
            if actor:
                carla_interface_obstacles.append((ego, actor))
        except Exception as e:
            logger.debug("Can not spawn ego vehicle")
            logger.debug(ego)
            logger.error(e, exc_info=sys.exc_info())
        return ego

    def _end_simmulation_error(self, clean_up, ego_interface_list: List[CommonRoadEgoInterface],
                               interface_obstacles: List[CommonRoadObstacleInterface], carla_controlled_obstacles,
                               pedestrian_handler: CarlaPedestrianHandler):
        self._create_video()

        settings = self.client.get_world().get_settings()
        # Stop synchronous mode to be able to investigate CARLA without a Script running
        settings.synchronous_mode = False
        self.client.get_world().apply_settings(settings)
        # clean up
        if clean_up:
            self._clean_up_carla(ego_interface_list, interface_obstacles, carla_controlled_obstacles,
                                 pedestrian_handler)
        sys.exit(0)

    def _create_video(self):
        # Create GIF
        if self.create_video:
            gc = Gif_Creator(self.video_path, self.video_name)
            if self.video_asMP4:
                gc.make_video()
            else:
                gc.make_gif()

    def _wait_for_carla_vehicle(self, time_between_ticks):
        dt = 0
        # wait for CARLA vehicles
        while dt < 80:
            self.client.get_world().tick()
            time.sleep(time_between_ticks)
            dt += 1

    def update_vehicle_to_scenario(self, carla_contr_obs_classes):
        new_dynamic_obs = []
        for obs in carla_contr_obs_classes:
            if obs.dynamic_obstacle and obs.commonroad_id:
                # rewrite prediction
                obs.dynamic_obstacle.prediction = TrajectoryPrediction(
                        trajectory=obs.dynamic_obstacle.prediction.trajectory,
                        shape=obs.dynamic_obstacle.obstacle_shape)
                new_dynamic_obs.append(obs.dynamic_obstacle)
        self.scenario.add_objects(new_dynamic_obs)

    def _update_scenario_motion_planner(self, carla_contr_obs_classes, motion_planner_vehicle):
        self.motion_planner.update_scenario_objects(carla_contr_obs_classes, None)
        new_planning_problem_state = motion_planner_vehicle.get_cr_state()
        current_state = self.motion_planner.trajectory_planner.planning_problem.initial_state
        current_state.position = new_planning_problem_state.position
        # verlocity is now not transform correctly
        current_state.velocity = self.motion_planner.trajectory_planner.record_state_list[-1].velocity
        current_state.orientation = new_planning_problem_state.orientation
        found = self.motion_planner.re_plan()
        return found
