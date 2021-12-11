#!/usr/bin/env python

import glob
import os
import sys
import time
from datetime import date, datetime
from enum import Enum
from typing import List

import carla
import commonroad.scenario.obstacle
import numpy as np
import pygame
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.obstacle import ObstacleRole, ObstacleType
from commonroad.scenario.trajectory import Trajectory
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.scenario.scenario import Scenario

try:
    from motion_planner.motion_planner import MotionPlanner
except ModuleNotFoundError:
    class MotionPlanner:
        pass

from carlacr.carla_pedestrian_handler import CarlaPedestrianHandler
from carlacr.carla_vehicle_interface import CarlaVehicleInterface
from carlacr.commonroad_ego_interface import CommonRoadEgoInterface
from carlacr.commonroad_obstacle_interface import (
    ApproximationType, CommonRoadObstacleInterface)
from carlacr.gif_creator import Gif_Creator
from carlacr.synchronous_mode import (CarlaSyncMode, draw_image,
                                      get_font, should_quit)
from carlacr.vehicle_dict import (similar_by_area, similar_by_length,
                                  similar_by_width)


class CarlaInterface:
    """
    Main class of the CommonRoad-CARLA-Interface
    """

    def __init__(
            self,
            cr_scenario_file_path: str,
            open_drive_map: str,
            carla_client: carla.Client,
            motion_planner: MotionPlanner = None,
            mpl_update_n: int = -1,
            cr_scenario: Scenario = None
    ):
        """

        :param cr_scenario_file_path: full path & filename to a CommonRoad XML-file
        :param open_drive_map: full path & filename to the according OpenDRIVE map for the scenario
        :param carla_client: carla.Client() object connected to the simulation
        :param motion_planner: a MotionPlanner object from the commonroad-motion-planning-library
        :param mpl_update_n: (in DEV) update interval at which rate the motion planner receives updated CommonRoad dynamic obstacles of the CARLA generated vehicles & pedestrians
        :param cr_scenario: Scenario obj
        """
        self.map = open_drive_map
        if not cr_scenario:
            self.scenario, self.planning_problem_set = CommonRoadFileReader(cr_scenario_file_path).open()
        else:
            self.scenario = cr_scenario
        self.motion_planner = motion_planner
        self.client = carla_client
        self.mpl_update_n = mpl_update_n

    def setup_carla(self, time_step_delta: int = None, tm_port=8000, hybrid_physics_mode=False):
        """
        Configures CARLA (self.client)

        :param time_step_delta: time_step_delta within the simulation (how much time is between two timesteps for CARLA), if None using dt from CommonRoad scenario
        :param tm_port: port of the CARLA traffic manager
        :param hybrid_physics_mode: sets hybrid_physics_mode in CARLA
        """
        self.client.set_timeout(2.0)
        world = self.client.get_world()
        # Synchrony:
        """
            Warning: If synchronous mode is enabled, and there is a Traffic Manager running, this must be set to sync mode too. Read this to learn how to do it.
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
                    print('file could not be readed.')
                    sys.exit()
            print('load opendrive map %r.' % os.path.basename(self.map))
            vertex_distance = 2.0  # in meters
            max_road_length = 500.0  # in meters
            wall_height = 1.0  # in meters
            extra_width = 0.6  # in meters
            world = self.client.generate_opendrive_world(
                data, carla.OpendriveGenerationParameters(
                    vertex_distance=vertex_distance,
                    max_road_length=max_road_length,
                    wall_height=wall_height,
                    additional_width=extra_width,
                    smooth_junctions=True,
                    enable_mesh_visibility=True))

        else:
            print('file not found.')

    def _calc_max_timestep(self) -> int:
        """
        Calculates maximal time step of current scenario
        """
        if self.scenario is None:
            return 0
        timesteps = [
            obstacle.prediction.occupancy_set[-1].time_step
            for obstacle in self.scenario.dynamic_obstacles
        ]
        self.max_timestep = np.max(timesteps) if timesteps else 0
        return self.max_timestep

    def _run_scenario_with_mpl(self, clean_up=True, time_step_delta_real=None, carla_vehicles=0, carla_pedestrians=0,
                               create_gif: bool = False, gif_path: str = None, gif_name: str = None,
                               asMp4: bool = True):
        """
        Runs the CommonRoad Scenario in CARLA (with MPL & PyGame)

        :param clean_up: if True destroys all created actors in the CARLA simulation
        :param time_step_delta_real: sets the time that will be waited in real time between the timesteps, if None the dt of the scenario will be used
        :param carla_vehicles: maximum number of vehicles that should be created & controlled by CARLA additional to the objects defined in the scenario
        :param carla_pedestrians: maximum number of pedestrians that should be created & controlled by CARLA additional to the objects defined in the scenario
        :param create_gif: if True a gif will be created (only when a MotionPlanner is provided)
        :param gif_path: path to a folder where the gif will be saved, additionally a folder at "gif_path"/img will be created in to save the images used for the gif
        :param gif_name: filename for the gif
        :param asMp4: flag to save as mp4 or gif
        """
        interface_obstacles = []
        carla_interface_obstacles = []
        carla_controlled_obstacles = []
        carla_contr_obs_classes = []
        ego_interface_list = []
        batch = []

        carla_controlled_imported = False

        pygame.init()

        display = pygame.display.set_mode(
            (800, 600),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        font = get_font()
        clock = pygame.time.Clock()

        world = self.client.get_world()
        # Load all dynamic obstacles from scenario into CARLA:
        # dynamic_obstacles = scenario.obstacles_by_role_and_type(obstacle_role=ObstacleRole.DYNAMIC, obstacle_type=ObstacleType.CAR)
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

        # Create Ego Vehicle
        if self.motion_planner:
            planning_problem_dict_values = self.planning_problem_set.planning_problem_dict.values()
            planning_problem_iter = iter(planning_problem_dict_values)
            planning_problem = next(planning_problem_iter)
            ego_trajectory = self.motion_planner.plan()
            ego = CommonRoadEgoInterface(planning_problem, ego_trajectory)
            ego_interface_list.append(ego)

            try:
                actor = ego.spawn(world)
                if actor:
                    carla_interface_obstacles.append((ego, actor))
            except Exception as e:
                print(e)

        i = 0  # time-step counter
        max_timesteps = self._calc_max_timestep()
        print(max_timesteps)
        with CarlaSyncMode(world, ego.actor_list[0], fps=30) as sync_mode:
            while i <= max_timesteps:
                if should_quit():
                    return
                clock.tick()
                snapshot, image_rgb = sync_mode.tick(timeout=2.0)
                if create_gif and gif_path:
                    image_rgb.save_to_disk('%s/img/%.6d.jpg' % (gif_path, image_rgb.frame))
                try:  # Simulation
                    # Ego Vehicle
                    if self.motion_planner and ego.is_spawned:
                        try:
                            ego.update_position_by_time(world, i)
                        except Exception as e:
                            print(e)

                    fps = round(1.0 / snapshot.timestamp.delta_seconds)
                    draw_image(display, image_rgb)
                    display.blit(
                        font.render('% 5d FPS (real)' % clock.get_fps(), True, (255, 255, 255)),
                        (8, 10))
                    display.blit(
                        font.render('% 5d FPS (simulated)' % fps, True, (255, 255, 255)),
                        (8, 28))
                    pygame.display.flip()

                    # CommonRoad controlled:
                    self._control_commonroad_obstacles(interface_obstacles,
                                                       carla_interface_obstacles,
                                                       world,
                                                       i)

                    # CARLA controlled:
                    if not carla_controlled_imported:
                        pedestrian_handler = self._control_carla_obstacles(carla_vehicles,
                                                                           carla_contr_obs_classes,
                                                                           batch,
                                                                           carla_controlled_obstacles,
                                                                           carla_pedestrians)
                        carla_controlled_imported = True
                        self._wait_for_carla_vehicle(time_between_ticks)
                    # Following comments print current cr-state of carla generated obstacles
                    # for obs in carla_contr_obs_classes:
                    #     print(obs.get_cr_state())
                    # print(pedestrian_handler.get_cr_state_list())

                    # If n-th timestep update obstacles for mpl
                    if (self.mpl_update_n > -1) & (i % self.mpl_update_n == 0):
                        carlaCRobsListToUpdate = []
                        for obs in carla_contr_obs_classes:
                            carlaCRobsListToUpdate.append(obs.get_cr_dynamic_obstacle())
                        carlaCRobsListToUpdate += pedestrian_handler.get_cr_dyn_obs_list()
                        # TODO: Following comment would update the cr-state for the mpl
                        # update_scenario_objects(carlaCRobsListToUpdate)

                    # advance time:
                    print("Timestep: " + str(i))
                    world.tick()
                    time.sleep(time_between_ticks)
                    i += 1
                except KeyboardInterrupt:
                    # Create GIF
                    if create_gif:
                        gc = Gif_Creator(gif_path, gif_name)
                        if asMp4:
                            gc.make_video()
                        else:
                            gc.make_gif()
                    settings = world.get_settings()
                    # Stop synchronous mode to be able to investigate CARLA without a Script running
                    settings.synchronous_mode = False
                    world.apply_settings(settings)
                    # clean up
                    if clean_up:
                        self._clean_up_carla(ego_interface_list, interface_obstacles,
                                             carla_controlled_obstacles, pedestrian_handler)
                    sys.exit(0)
                except Exception as e:
                    print(e)
        # Create GIF
        if create_gif:
            gc = Gif_Creator(gif_path, gif_name)
            if asMp4:
                gc.make_video()
            else:
                gc.make_gif()
        settings = world.get_settings()
        # Stop synchronous mode to be able to investigate CARLA without a Script running
        settings.synchronous_mode = False
        world.apply_settings(settings)

        # clean up
        if clean_up:
            self._clean_up_carla(ego_interface_list, interface_obstacles,
                                 carla_controlled_obstacles, pedestrian_handler)

    def _run_scenario_without_mpl(self, clean_up=True, time_step_delta_real=0.05, carla_vehicles=0,
                                  carla_pedestrians=0):
        """
        Runs the CommonRoad Scenario in CARLA without mpl

        :param clean_up: if True destroys all created actors in the CARLA simulation
        :param time_step_delta_real: sets the time that will be waited in real time between the timesteps, if None the dt of the scenario will be used
        :param carla_vehicles: maximum number of vehicles that should be created & controlled by CARLA additional to the objects defined in the scenario
        :param carla_pedestrians: maximum number of pedestrians that should be created & controlled by CARLA additional to the objects defined in the scenario
        """
        interface_obstacles = []
        carla_interface_obstacles = []
        carla_controlled_obstacles = []
        carla_contr_obs_classes = []
        ego_interface_list = []
        batch = []

        carla_controlled_imported = False

        world = self.client.get_world()
        # Load all dynamic obstacles from scenario into CARLA:
        # dynamic_obstacles = scenario.obstacles_by_role_and_type(obstacle_role=ObstacleRole.DYNAMIC, obstacle_type=ObstacleType.CAR)
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
        max_timesteps = self._calc_max_timestep()
        while i <= max_timesteps:
            try:  # Simulation
                # CommonRoad controlled:
                self._control_commonroad_obstacles(interface_obstacles,
                                                   carla_interface_obstacles,
                                                   world,
                                                   i)

                # CARLA controlled:
                if not carla_controlled_imported:
                    pedestrian_handler = self._control_carla_obstacles(carla_vehicles,
                                                                       carla_contr_obs_classes,
                                                                       batch,
                                                                       carla_controlled_obstacles,
                                                                       carla_pedestrians)
                    carla_controlled_imported = True
                    self._wait_for_carla_vehicle(time_between_ticks)

                # advance time:
                print("Timestep: " + str(i))
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
                    self._clean_up_carla(ego_interface_list, interface_obstacles,
                                         carla_controlled_obstacles, pedestrian_handler)
                sys.exit(0)
            except Exception as e:
                print(e)

        settings = world.get_settings()
        # Stop synchronous mode to be able to investigate CARLA without a Script running
        settings.synchronous_mode = False
        world.apply_settings(settings)
        # clean up
        if clean_up:
            self._clean_up_carla(ego_interface_list, interface_obstacles,
                                 carla_controlled_obstacles, pedestrian_handler)

    def run_scenario_with_ego_vehicle(self, time_step_delta_real, ego_vehicle: commonroad.scenario.obstacle.Obstacle,
                                      create_gif: bool = False, gif_path: str = None, gif_name: str = "ego",
                                      asMP4: bool = False):
        """
        run scenario with ego vehicle setting

        :param time_step_delta_real: sets the time that will be waited in real time between the timesteps, if None the dt of the scenario will be used
        :param ego_vehicle: vehicle that will be used as ego_vehicle
        :param create_gif: if True a gif will be created (only when a MotionPlanner is provided)
        :param gif_path: path to a folder where the gif will be saved, additionally a folder at "gif_path"/img will be created in to save the images used for the gif
        :param gif_name: filename for the gif
        :param asMP4: flag to save as mp4 or gif
        """
        interface_obstacles = []
        carla_interface_obstacles = []
        carla_controlled_obstacles = []
        carla_contr_obs_classes = []
        ego_interface_list = []
        batch = []

        carla_controlled_imported = False

        pygame.init()
        display = pygame.display.set_mode(
            (800, 600),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
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
        print(max_timesteps)
        # create ego

        ego = CommonRoadEgoInterface(trajectory=ego_vehicle.prediction.trajectory,
                                     initial_state=ego_vehicle.initial_state)

        ego_interface_list.append(ego)
        try:
            actor = ego.spawn(world)
            if actor:
                carla_interface_obstacles.append((ego, actor))
        except Exception as e:
            print(e)

        with CarlaSyncMode(world, ego.actor_list[0], fps=30) as sync_mode:
            while i <= max_timesteps:
                # set up gif
                if should_quit():
                    return
                clock.tick()
                snapshot, image_rgb = sync_mode.tick(timeout=2.0)
                if create_gif and gif_path:
                    image_rgb.save_to_disk('%s/img/%.6d.jpg' % (gif_path, image_rgb.frame))
                try:  # Simulation
                    # Ego Vehicle
                    if ego.is_spawned:
                        try:
                            ego.update_position_by_time(world, i)
                        except Exception as e:
                            print(e)

                    fps = round(1.0 / snapshot.timestamp.delta_seconds)
                    draw_image(display, image_rgb)
                    display.blit(
                        font.render('% 5d FPS (real)' % clock.get_fps(), True, (255, 255, 255)),
                        (8, 10))
                    display.blit(
                        font.render('% 5d FPS (simulated)' % fps, True, (255, 255, 255)),
                        (8, 28))
                    pygame.display.flip()

                    # CommonRoad controlled:
                    self._control_commonroad_obstacles(interface_obstacles,
                                                       carla_interface_obstacles,
                                                       world,
                                                       i)

                    # CARLA controlled:
                    if not carla_controlled_imported:
                        pedestrian_handler = self._control_carla_obstacles(0,
                                                                           carla_contr_obs_classes,
                                                                           batch,
                                                                           carla_controlled_obstacles,
                                                                           0)
                        carla_controlled_imported = True
                        self._wait_for_carla_vehicle(time_between_ticks)

                    # advance time:
                    print("Timestep: " + str(i))
                    world.tick()
                    time.sleep(time_between_ticks)
                    i += 1
                except KeyboardInterrupt:
                    # Create GIF
                    if create_gif:
                        gc = Gif_Creator(gif_path, gif_name)
                        if asMP4:
                            gc.make_video()
                        else:
                            gc.make_gif()
                    settings = world.get_settings()
                    # Stop synchronous mode to be able to investigate CARLA without a Script running
                    settings.synchronous_mode = False
                    world.apply_settings(settings)
                    # clean up
                    self._clean_up_carla(ego_interface_list, interface_obstacles,
                                         carla_controlled_obstacles, pedestrian_handler)
                    sys.exit(0)
                except Exception as e:
                    print(e)
        # Create GIF
        if create_gif:
            gc = Gif_Creator(gif_path, gif_name)
            if asMP4:
                gc.make_video()
            else:
                gc.make_gif()
        settings = world.get_settings()
        # Stop synchronous mode to be able to investigate CARLA without a Script running
        settings.synchronous_mode = False
        world.apply_settings(settings)

        # clean up
        self._clean_up_carla(ego_interface_list, interface_obstacles,
                             carla_controlled_obstacles, pedestrian_handler)

    def run_scenario(self, clean_up=True, time_step_delta_real=None, carla_vehicles=0, carla_pedestrians=0,
                     create_gif: bool = False, gif_path: str = None, gif_name: str = None, asMP4: bool = False):
        """
        Runs the CommonRoad Scenario in CARLA. Splits up between a simulation with and without a motion planner

        :param clean_up: if True destroys all created actors in the CARLA simulation
        :param time_step_delta_real: sets the time that will be waited in real time between the timesteps, if None the dt of the scenario will be used
        :param carla_vehicles: maximum number of vehicles that should be created & controlled by CARLA additional to the objects defined in the scenario
        :param carla_pedestrians: maximum number of pedestrians that should be created & controlled by CARLA additional to the objects defined in the scenario
        :param create_gif: if True a gif will be created (only when a MotionPlanner is provided)
        :param gif_path: path to a folder where the gif will be saved, additionally a folder at "gif_path"/img will be created in to save the images used for the gif
        :param gif_name: filename for the gif
        :param asMP4: flag to save as mp4 or gif
        """
        if gif_path:
            now = datetime.now()
            today = date.today()
            current_time = now.strftime("%H_%M_%S")
            current_date = today.strftime("%d_%m_%Y")
            gif_path += f"/{self.scenario.scenario_id}_{current_date}_{current_time}"

        if self.motion_planner:
            self._run_scenario_with_mpl(clean_up, time_step_delta_real, carla_vehicles, carla_pedestrians, create_gif,
                                        gif_path, gif_name, asMP4)
        else:
            if create_gif:
                print("GIFs can only be created when a Motion Planner is provided!")
            self._run_scenario_without_mpl(clean_up, time_step_delta_real, carla_vehicles, carla_pedestrians)

    def _clean_up_carla(self, ego_interface_list: List[CommonRoadEgoInterface],
                        interface_obstacles: List[CommonRoadObstacleInterface],
                        carla_controlled_obstacles,
                        pedestrian_handler: CarlaPedestrianHandler):
        for ego in ego_interface_list:
            ego.destroy_carla_actor(self.client.get_world())
        for obs in interface_obstacles:
            obs.destroy_carla_obstacle(self.client.get_world())
        self.client.apply_batch([carla.command.DestroyActor(x)
                                 for x in carla_controlled_obstacles])
        if pedestrian_handler:
            pedestrian_handler.destroy()

    def _control_commonroad_obstacles(self, interface_obstacles: List[CommonRoadObstacleInterface],
                                      carla_interface_obstacles: List,
                                      world: carla.World,
                                      curr_time_step: int):
        for obs in interface_obstacles:
            if not obs.is_spawned:
                try:
                    actor = obs.spawn(world, ApproximationType.LENGTH)
                    if actor:
                        carla_interface_obstacles.append((obs, actor))
                except Exception as e:
                    print(e)
            else:
                try:
                    obs.update_position_by_time(world, curr_time_step)
                except Exception as e:
                    print(e)

    def _control_carla_obstacles(self, carla_vehicles,
                                 carla_contr_obs_classes: List[CarlaVehicleInterface],
                                 batch: List[carla.command.SpawnActor],
                                 carla_controlled_obstacles: List,
                                 carla_pedestrians: int):
        for numb_veh in range(0, carla_vehicles):
            q = CarlaVehicleInterface(self.scenario, self.client)
            carla_contr_obs_classes.append(q)
            actor = q.get_spawnable()
            batch.append(actor)
        x = 0
        for response in self.client.apply_batch_sync(batch):
            if response.error:
                print(response.error)
            else:
                carla_controlled_obstacles.append(response.actor_id)
                carla_obs = carla_contr_obs_classes[x]
                carla_obs.update_after_spawn(cr_id=self.scenario.generate_object_id(),
                                             actor_id=response.actor_id)
                x += 1
        pedestrian_handler = CarlaPedestrianHandler(self.scenario, self.client, carla_pedestrians)
        pedestrian_handler.spawn()
        print(pedestrian_handler)
        return pedestrian_handler

    def _wait_for_carla_vehicle(self, time_between_ticks):
        dt = 0
        # wait for CARLA vehicles
        while dt < 80:
            self.client.get_world().tick()
            time.sleep(time_between_ticks)
            dt += 1
