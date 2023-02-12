import carla
import os
import sys
import subprocess
import time
import signal
from datetime import date, datetime
from typing import List, Tuple, Union
import logging
import pygame
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.obstacle import ObstacleRole, DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.prediction.prediction import TrajectoryPrediction
# from carlacr.interface.carla_pedestrian_handler import CarlaPedestrianHandler
# from carlacr.interface.carla_vehicle_interface import CarlaVehicleInterface
# from carlacr.interface.commonroad_ego_interface import CommonRoadEgoInterface
# from carlacr.interface.commonroad_obstacle_interface import (ApproximationType, CommonRoadObstacleInterface)
# from carlacr.helper.carla_interface_helper import GifCreator, calc_max_timestep
# from carlacr.mode.synchronous_mode import (CarlaSyncMode, draw_image, get_font, should_quit)
from carlacr.helper.config import CarlaParams

logger = logging.getLogger(__name__)


class CarlaInterface:
    """Main class of the CommonRoad-CARLA-Interface."""

    def __init__(self, map: str, cr_scenario_file_path: str = None,
                 config: CarlaParams = CarlaParams()):
        """
        Constructor of CarlaInterface.

        :param map: full path & filename to OpenDRIVE map for the simulation or name of CARLA default map
        :param cr_scenario_file_path: path to CommonRoad scenario file
        :param config: CARLA config dataclass.
        map
        """
        self._client = carla.Client(config.host, config.port)
        self._cr_path = cr_scenario_file_path
        self._config = config
        self._carla_pid = None

        if self._config.start_carla_server:
            self._start_carla_server()
        self._init_carla_world()
        self._init_carla_traffic_manager()
        self._load_map(map)

    def __del__(self):
        """Kill CARLA server in case it was started by the CARLA-Interface."""
        if self._carla_pid is not None:
            logger.info("Killing CARLA server.")
            os.killpg(os.getpgid(self._carla_pid.pid), signal.SIGTERM)
            time.sleep(self._config.sleep_time)

    def _start_carla_server(self):
        """Start CARLA server in desired operating mode (3D/offscreen)."""
        path_to_carla = self._find_carla_executable()
        logger.info("Start CARLA server.")
        if self._config.offscreen_mode:
            self._carla_pid = subprocess.Popen([path_to_carla, '-RenderOffScreen'], stdout=subprocess.PIPE,
                                               preexec_fn=os.setsid)
            logger.info("CARLA server started in non-rendering mode.")
        else:
            self._carla_pid = subprocess.Popen([path_to_carla], stdout=subprocess.PIPE, preexec_fn=os.setsid)
            logger.info("CARLA server started.")
        time.sleep(self._config.sleep_time)

    def _find_carla_executable(self) -> str:
        """Searches for CARLA executable in provided paths.

        :returns Path to CARLa executable as string.
        """
        logger.info("Search CARLA server executable.")
        for default_path in self._config.default_carla_paths:
            path = os.path.join(default_path.replace("/~", os.path.expanduser("~")), "CarlaUE4.sh")
            if os.path.exists(path):
                return path
        raise FileNotFoundError("CARLA executable not found.")

    def _init_carla_world(self):
        """Configures CARLA world."""
        self._client.set_timeout(self._config.simulation.client_init_timeout)
        world = self._client.get_world()
        # Synchrony:
        # Warning: If synchronous mode is enabled, and there is a Traffic Manager running, this must be set to sync
        # mode too. Read this to learn how to do it.
        # See: https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/#setting-synchronous-mode
        settings = world.get_settings()
        settings.synchronous_mode = self._config.simulation.synchronous
        settings.fixed_delta_seconds = self._config.simulation.time_step
        world.apply_settings(settings)

    def _init_carla_traffic_manager(self):
        """Configures CARLA traffic manager."""
        traffic_manager = self._client.get_trafficmanager(self._config.simulation.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(
                self._config.simulation.global_distance_to_leading_vehicle)
        traffic_manager.global_percentage_speed_difference(self._config.simulation.global_percentage_speed_difference)
        traffic_manager.set_hybrid_physics_mode(self._config.simulation.hybrid_physics_mode)
        traffic_manager.set_synchronous_mode(self._config.simulation.synchronous)

    def _load_map(self, map_name: str):
        """
        Loads OpenDRIVE map into CARLA.

        Based on provided CARLA code: https://github.com/carla-simulator/carla/blob/master/PythonAPI/util/config.py
        Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
        Barcelona (UAB).
        This work is licensed under the terms of the MIT license.
        For a copy, see <https://opensource.org/licenses/MIT>.
        """
        if map_name[0:4] == "Town":
            self._client.load_world(map_name)
        elif os.path.exists(map_name):
            logger.info(f"Load OpenDRIVE map: {os.path.basename(map_name)}")
            with open(map_name, encoding='utf-8') as od_file:
                try:
                    data = od_file.read()
                except OSError:
                    logger.error(f"Failed load OpenDRIVE map: {os.path.basename(map_name)}")
                    sys.exit()
            logger.info(f"Loaded OpenDRIVE map: {os.path.basename(map_name)} successfully.")

            self._client.generate_opendrive_world(data, carla.OpendriveGenerationParameters(
                    vertex_distance=self._config.map.vertex_distance,
                    max_road_length=self._config.map.max_road_length,
                    wall_height=self._config.map.wall_height,
                    additional_width=self._config.map.extra_width,
                    smooth_junctions=True,
                    enable_mesh_visibility=True,
                    enable_pedestrian_navigation=True))
        time.sleep(self._config.sleep_time)
    #
    # def calc_max_timestep(self) -> int:
    #     """Calculates maximal time step of current scenario."""
    #     self.max_timestep = calc_max_timestep(self.scenario)
    #     return self.max_timestep
    #
    # def _run_scenario_default(self, clean_up=True, time_step_delta_real=0.05, carla_vehicles=0, carla_pedestrians=0,
    #                           scenario_time_steps: int = 0):
    #     """
    #     Runs the CommonRoad Scenario in CARLA default.
    #
    #     :param clean_up: if True destroys all created actors in the CARLA simulation
    #     :param time_step_delta_real: sets the time that will be waited in real time between the time steps,
    #     if None the dt of the scenario will be used
    #     :param carla_vehicles: maximum number of vehicles that should be created & controlled by CARLA additional to
    #     the objects defined in the scenario
    #     :param carla_pedestrians: maximum number of pedestrians that should be created & controlled by CARLA
    #     additional to the objects defined in the scenario
    #     """
    #     interface_obstacles: List[CommonRoadObstacleInterface] = []
    #     pedestrian_handler: Union[None, CarlaPedestrianHandler] = None
    #     carla_interface_obstacles: List[Tuple] = []
    #     carla_controlled_obstacles: List[int] = []
    #     carla_contr_obs_classes: List[CarlaVehicleInterface] = []
    #     ego_interface_list: List[CommonRoadEgoInterface] = []
    #     batch: List[carla.command.SpawnActor] = []
    #     carla_contr_dynamic_obs: List[DynamicObstacle] = []
    #
    #     carla_controlled_imported = False
    #
    #     world = self.client.get_world()
    #     # Load all dynamic obstacles from scenario into CARLA:
    #     # dynamic_obstacles = scenario.obstacles_by_role_and_type(obstacle_role=ObstacleRole.DYNAMIC,
    #     # obstacle_type=ObstacleType.CAR)
    #     dynamic_obstacles = self.scenario.dynamic_obstacles
    #     dynamic_obstacles += self.scenario.static_obstacles
    #     # real world time step delta
    #     if time_step_delta_real:
    #         time_between_ticks = time_step_delta_real
    #     else:
    #         time_between_ticks = self.scenario.dt
    #
    #     # Create Interface
    #     for obstacle in dynamic_obstacles:
    #         obs = CommonRoadObstacleInterface(obstacle)
    #         interface_obstacles.append(obs)
    #
    #     i = 0  # time-step counter
    #     max_timesteps = max(self.calc_max_timestep(), scenario_time_steps)
    #     while i <= max_timesteps:
    #         try:  # Simulation
    #             # CommonRoad controlled:
    #             self.control_commonroad_obstacles(interface_obstacles, carla_interface_obstacles, i)
    #
    #             # CARLA controlled:
    #             if not carla_controlled_imported:
    #                 pedestrian_handler = self.create_carla_obstacles(carla_vehicles, carla_contr_obs_classes, batch,
    #                                                                  carla_controlled_obstacles, carla_pedestrians)
    #                 carla_controlled_imported = True
    #                 self.wait_for_carla_vehicle(time_between_ticks)
    #                 self._control_carla_obstacles(carla_contr_obs_classes, i, carla_contr_dynamic_obs)
    #
    #             else:
    #                 self._control_carla_obstacles(carla_contr_obs_classes, i, carla_contr_dynamic_obs)
    #
    #             # advance time:
    #             logger.debug("Timestep: %s", str(i))
    #             world.tick()
    #             time.sleep(time_between_ticks)
    #             i += 1
    #         except KeyboardInterrupt:
    #             # Create GIF
    #             settings = world.get_settings()
    #             # Stop synchronous mode to be able to investigate CARLA without a Script running
    #             settings.synchronous_mode = False
    #             world.apply_settings(settings)
    #             # clean up
    #             if clean_up:
    #                 self.clean_up_carla(ego_interface_list, interface_obstacles, carla_controlled_obstacles,
    #                                     pedestrian_handler)
    #             sys.exit(0)
    #         except Exception as e:
    #             logger.error(e, exc_info=sys.exc_info())
    #
    #     settings = world.get_settings()
    #     # Stop synchronous mode to be able to investigate CARLA without a Script running
    #     settings.synchronous_mode = False
    #     world.apply_settings(settings)
    #     # add new obs to the scenario
    #     self._update_vehicle_to_scenario(carla_contr_obs_classes)
    #     # clean up
    #     if clean_up:
    #         self.clean_up_carla(ego_interface_list, interface_obstacles, carla_controlled_obstacles, pedestrian_handler)
    #
    # def run_scenario_with_ego_vehicle(self, time_step_delta_real, ego_vehicle: DynamicObstacle, carla_vehicles=0,
    #                                   carla_pedestrians=0, clean_up=True):
    #     """
    #     Run scenario with ego vehicle setting.
    #
    #     :param clean_up: if True destroys all created actors in the CARLA simulation
    #     :param time_step_delta_real: sets the time that will be waited in real time between the timesteps,
    #     if None the dt of the scenario will be used
    #     :param ego_vehicle: vehicle that will be used as ego_vehicle
    #     :param carla_vehicles: maximum number of vehicles that should be created & controlled by CARLA additional to
    #     the objects defined in the scenario
    #     :param carla_pedestrians: maximum number of pedestrians that should be created & controlled by CARLA
    #     additional to the objects defined in the scenario
    #     """
    #     interface_obstacles: List[CommonRoadObstacleInterface] = []
    #     pedestrian_handler: Union[None, CarlaPedestrianHandler] = None
    #     carla_interface_obstacles: List[Tuple] = []
    #     carla_controlled_obstacles: List[int] = []
    #     carla_contr_obs_classes: List[CarlaVehicleInterface] = []
    #
    #     ego_interface_list: List[CommonRoadEgoInterface] = []
    #     batch: List[carla.command.SpawnActor] = []
    #
    #     carla_controlled_imported = False
    #
    #     pygame.init()
    #     display = pygame.display.set_mode((800, 600), pygame.HWSURFACE | pygame.DOUBLEBUF)
    #     font = get_font()
    #     clock = pygame.time.Clock()
    #     world = self.client.get_world()
    #     # Load all dynamic obstacles from scenario into CARLA:
    #     dynamic_obstacles = self.scenario.dynamic_obstacles
    #     dynamic_obstacles += self.scenario.static_obstacles
    #
    #     # real world time step delta
    #     if time_step_delta_real:
    #         time_between_ticks = time_step_delta_real
    #     else:
    #         time_between_ticks = self.scenario.dt
    #
    #     if ego_vehicle in dynamic_obstacles:
    #         dynamic_obstacles.remove(ego_vehicle)
    #
    #     # Create Interface
    #     for obstacle in dynamic_obstacles:
    #         obs = CommonRoadObstacleInterface(obstacle)
    #         interface_obstacles.append(obs)
    #
    #     i = 0  # time-step counter
    #     max_timesteps = self.calc_max_timestep()
    #     logger.debug(max_timesteps)
    #     # Create ego
    #     ego = self.create_ego_vehicle(ego_vehicle=ego_vehicle, ego_interface_list=ego_interface_list,
    #                                   carla_interface_obstacles=carla_interface_obstacles)
    #
    #     with CarlaSyncMode(world, ego.actor_list[0], fps=30) as sync_mode:
    #         while i <= max_timesteps:
    #             # set up gif
    #             if should_quit():
    #                 return
    #             clock.tick()
    #             snapshot, image_rgb = sync_mode.tick(timeout=2.0)
    #             if self.video_creation and self.video_path:
    #                 image_rgb.save_to_disk('%s/img/%.6d.jpg' % (self.video_path, image_rgb.frame))
    #             try:  # Simulation
    #                 # Ego Vehicle
    #                 if ego.is_spawned:
    #                     try:
    #                         state = ego.trajectory.state_at_time_step(i)
    #                         if state:
    #                             ego.update_position_by_time(world, state)
    #                     except Exception as e:
    #                         logger.debug("Error when update vehicle")
    #                         logger.debug(ego)
    #                         logger.error(e, exc_info=sys.exc_info())
    #
    #                 fps = round(1.0 / snapshot.timestamp.delta_seconds)
    #                 draw_image(display, image_rgb)
    #                 display.blit(font.render("% 5d FPS (real)" % clock.get_fps(), True, (255, 255, 255)), (8, 10))
    #                 display.blit(font.render("% 5d FPS (simulated)" % fps, True, (255, 255, 255)), (8, 28))
    #                 pygame.display.flip()
    #
    #                 # CommonRoad controlled:
    #                 self.control_commonroad_obstacles(interface_obstacles, carla_interface_obstacles, i)
    #
    #                 # CARLA controlled:
    #                 if not carla_controlled_imported:
    #                     pedestrian_handler = self.create_carla_obstacles(carla_vehicles, carla_contr_obs_classes, batch,
    #                                                                      carla_controlled_obstacles, carla_pedestrians)
    #                     carla_controlled_imported = True
    #                     self.wait_for_carla_vehicle(time_between_ticks)
    #
    #                 # advance time:
    #                 logger.debug("Timestep: %s", str(i))
    #                 world.tick()
    #                 time.sleep(time_between_ticks)
    #                 i += 1
    #             except KeyboardInterrupt:
    #                 self.end_simulation_error(clean_up=clean_up, ego_interface_list=ego_interface_list,
    #                                           interface_obstacles=interface_obstacles,
    #                                           carla_controlled_obstacles=carla_controlled_obstacles,
    #                                           pedestrian_handler=pedestrian_handler)
    #             except Exception as e:
    #                 logger.error(e, exc_info=sys.exc_info())
    #     # Create video
    #     self.video_creation()
    #     settings = world.get_settings()
    #     # Stop synchronous mode to be able to investigate CARLA without a Script running
    #     settings.synchronous_mode = False
    #     world.apply_settings(settings)
    #
    #     # clean up
    #     self.clean_up_carla(ego_interface_list, interface_obstacles, carla_controlled_obstacles, pedestrian_handler)
    #
    # def run_scenario(self, clean_up=True, time_step_delta_real=None, carla_vehicles=0, carla_pedestrians=0,
    #                  ego_vehicle=None, scenario_time_steps: int = 0):
    #     """
    #     Runs the CommonRoad Scenario in CARLA.
    #
    #     :param clean_up: if True destroys all created actors in the CARLA simulation
    #     :param time_step_delta_real: sets the time that will be waited in real time between the timesteps,
    #     if None the dt of the scenario will be used
    #     :param carla_vehicles: maximum number of vehicles that should be created & controlled by CARLA additional to
    #     the objects defined in the scenario
    #     :param carla_pedestrians: maximum number of pedestrians that should be created & controlled by CARLA
    #     additional to the objects defined in the scenario
    #     :param ego_vehicle: the vehicle to be view
    #     :param scenario_time_steps: minimum scenario time steps, use to prolong a scenario with carla traffic generation
    #     """
    #     if self.video_path:
    #         now = datetime.now()
    #         today = date.today()
    #         current_time = now.strftime("%H_%M_%S")
    #         current_date = today.strftime("%d_%m_%Y")
    #         video_folder = f"/{self.scenario.scenario_id}_{current_date}_{current_time}"
    #         self.video_path += video_folder
    #
    #     else:
    #         if self.video_creation and not ego_vehicle:
    #             logger.debug("GIFs can only be created with default mode or when a ego_vehicle is provided!")
    #         if ego_vehicle:
    #             self.run_scenario_with_ego_vehicle(time_step_delta_real=time_step_delta_real, ego_vehicle=ego_vehicle,
    #                                                carla_pedestrians=carla_pedestrians, carla_vehicles=carla_vehicles)
    #         else:
    #             self._run_scenario_default(clean_up=clean_up, time_step_delta_real=time_step_delta_real,
    #                                        carla_pedestrians=carla_pedestrians, carla_vehicles=carla_vehicles,
    #                                        scenario_time_steps=scenario_time_steps)
    #     if self.video_path:
    #         self.video_path = self.video_path[:len(self.video_path) - len(video_folder)]
    #
    # def clean_up_carla(self, ego_interface_list: List[CommonRoadEgoInterface],
    #                    interface_obstacles: List[CommonRoadObstacleInterface], carla_controlled_obstacles: List[int],
    #                    pedestrian_handler: Union[None, CarlaPedestrianHandler]):
    #     """
    #     Clean up all carla objects.
    #
    #     :param ego_interface_list: list of CommonRoadEgoInterface object
    #     :param interface_obstacles: list of CommonRoadObstacleInterface object
    #     :param carla_controlled_obstacles: list of actor_id in same position carla_contr_obs_classes as  object
    #     :param pedestrian_handler: CarlaPedestrianHandler
    #     """
    #     for ego in ego_interface_list:
    #         ego.destroy_carla_actor(self.client.get_world())
    #     for obs in interface_obstacles:
    #         obs.destroy_carla_obstacle(self.client.get_world())
    #     self.client.apply_batch([carla.command.DestroyActor(x) for x in carla_controlled_obstacles])
    #     if pedestrian_handler:
    #         pedestrian_handler.destroy()
    #
    # def control_commonroad_obstacles(self, interface_obstacles: List[CommonRoadObstacleInterface],
    #                                  carla_interface_obstacles: List[Tuple], curr_time_step: int,
    #                                  mode: str = "by-time"):
    #     """
    #     Control CommonRoad obstacles, spawn, update position and destroy regarding actor in carla if out of scenario.
    #
    #     :param interface_obstacles: list of CommonRoadObstacleInterface object
    #     :param carla_interface_obstacles: list of tuple (interface object,actor)
    #     :param curr_time_step: current time step of the scenario
    #     :param mode: update obstacle "by-time", "by-control", or "by-ackermann-control"
    #     """
    #     deleted_obs = []
    #     for obs in interface_obstacles:
    #         if not obs.is_spawned:
    #             try:
    #                 actor = obs.spawn(self.client.get_world(), ApproximationType.LENGTH)
    #                 if actor:
    #                     carla_interface_obstacles.append((obs, actor))
    #             except Exception as e:
    #                 logger.error(e, exc_info=sys.exc_info())
    #         else:
    #             try:
    #                 if obs.role == ObstacleRole.DYNAMIC and obs.trajectory.state_at_time_step(curr_time_step):
    #                     # decide control mode
    #                     if mode == "by-time":
    #                         obs.update_position_by_time(self.client.get_world(),
    #                                                     obs.trajectory.state_at_time_step(curr_time_step))
    #                     elif mode == "by-control":
    #                         obs.update_position_by_control(self.client.get_world(),
    #                                                        obs.trajectory.state_at_time_step(curr_time_step))
    #                     elif mode == "by-ackermann-control":
    #                         obs.update_with_ackermann_control(self.client.get_world(),
    #                                                           obs.trajectory.state_at_time_step(curr_time_step))
    #
    #                 elif obs.role != ObstacleRole.STATIC:
    #                     deleted_obs.append(obs)
    #                     obs.destroy_carla_obstacle(self.client.get_world())
    #             except Exception as e:
    #                 logger.error(e, exc_info=sys.exc_info())
    #     for obs in deleted_obs:
    #         interface_obstacles.remove(obs)
    #
    # def create_carla_obstacles(self, carla_vehicles: int, carla_contr_obs_classes: List[CarlaVehicleInterface],
    #                            batch: List[carla.command.SpawnActor], carla_controlled_obstacles: List[int],
    #                            carla_pedestrians: int) -> CarlaPedestrianHandler:
    #     """
    #     Create random vehicles on carla.
    #
    #     :param carla_vehicles: number of carla auto-generated vehicles
    #     :param carla_contr_obs_classes: list of CarlaVehicleInterface object
    #     :param carla_controlled_obstacles: list of actor_id in same position as carla_contr_obs_classes as  object
    #     :param batch: list of SpawnActor in carla
    #     :param carla_pedestrians: number of pedestrians
    #     :return: CarlaPedestrianHandler object
    #     """
    #     for _numb_veh in range(0, carla_vehicles):
    #         q = CarlaVehicleInterface(self.scenario, self.client)
    #         actor = q.get_spawnable()
    #         if actor:
    #             carla_contr_obs_classes.append(q)
    #             batch.append(actor)
    #     x = 0
    #     for response in self.client.apply_batch_sync(batch):
    #         if response.error:
    #             logger.debug(response.error)
    #         else:
    #             carla_controlled_obstacles.append(response.actor_id)
    #             carla_obs = carla_contr_obs_classes[x]
    #             carla_obs.update_after_spawn(cr_id=self.scenario.generate_object_id(), actor_id=response.actor_id)
    #             x += 1
    #     pedestrian_handler = CarlaPedestrianHandler(self.scenario, self.client, carla_pedestrians)
    #     pedestrian_handler.spawn()
    #     logger.debug(pedestrian_handler)
    #     return pedestrian_handler
    #
    # def _control_carla_obstacles(self, carla_contr_obs_classes: List[CarlaVehicleInterface], current_time_step: int,
    #                              carla_contr_dynamic_obs: List[DynamicObstacle]):
    #     """
    #     Control auto-generated carla vehicle. Update new state to regarding commonroad dynamic obstacle.
    #
    #     :param carla_contr_obs_classes: list of CarlaVehicleInterface object
    #     :param current_time_step:
    #     :param carla_contr_dynamic_obs: list of DynamicObstacle created by carla vehicles auto generation
    #     """
    #     for obs in carla_contr_obs_classes:
    #         if obs.dynamic_obstacle:
    #             obs.dynamic_obstacle.prediction.trajectory.state_list.append(obs.get_cr_state(current_time_step))
    #         elif obs.commonroad_id:
    #             carla_contr_dynamic_obs.append(obs.create_cr_dynamic_obstacle())
    #
    # def create_ego_vehicle(self, ego_vehicle: DynamicObstacle, ego_interface_list: List[CommonRoadEgoInterface],
    #                        carla_interface_obstacles: List) -> CommonRoadEgoInterface:
    #     """
    #     Create ego vehicle from DynamicObstacle.
    #
    #     :param ego_vehicle: DynamicObstacle  will be used as ego_vehicle
    #     :param ego_interface_list: list of DynamicObstacle that will be used as ego
    #     :param carla_interface_obstacles: list of tuple (interface object,actor)
    #     :return: CommonRoadEgoInterface for the ego_vehicle
    #     """
    #     ego = CommonRoadEgoInterface(client=self.client, trajectory=ego_vehicle.prediction.trajectory,
    #                                  initial_state=ego_vehicle.initial_state,
    #                                  size=(ego_vehicle.obstacle_shape.length, ego_vehicle.obstacle_shape.width, 0))
    #     ego_interface_list.append(ego)
    #     try:
    #         actor = ego.spawn(self.client.get_world())
    #         if actor:
    #             carla_interface_obstacles.append((ego, actor))
    #     except Exception as e:
    #         logger.debug("Can not spawn ego vehicle")
    #         logger.debug(ego)
    #         logger.error(e, exc_info=sys.exc_info())
    #     return ego
    #
    # def end_simulation_error(self, clean_up, ego_interface_list: List[CommonRoadEgoInterface],
    #                          interface_obstacles: List[CommonRoadObstacleInterface],
    #                          carla_controlled_obstacles: List[int],
    #                          pedestrian_handler: Union[None, CarlaPedestrianHandler]):
    #     """
    #     Handle error ending simulation.
    #
    #     :param clean_up: clean up flag
    #     :param ego_interface_list: list of DynamicObstacle that will be used as ego
    #     :param interface_obstacles: list of CommonRoadObstacleInterface
    #     :param carla_controlled_obstacles: list of actor_id in same position as carla_contr_obs_classes as  object
    #     :param pedestrian_handler: CarlaPedestrianHandler object
    #     """
    #     self.video_creation()
    #
    #     settings = self.client.get_world().get_settings()
    #     # Stop synchronous mode to be able to investigate CARLA without a Script running
    #     settings.synchronous_mode = False
    #     self.client.get_world().apply_settings(settings)
    #     # clean up
    #     if clean_up:
    #         self.clean_up_carla(ego_interface_list, interface_obstacles, carla_controlled_obstacles, pedestrian_handler)
    #     sys.exit(0)
    #
    # def create_video(self):
    #     """Create gif for carla visualization."""
    #     # Create GIF
    #     if self.video_creation:
    #         gc = GifCreator(self.video_path, self.video_name)
    #         if self.video_asMP4:
    #             gc.make_video()
    #         else:
    #             gc.make_gif()
    #
    # def wait_for_carla_vehicle(self, time_between_ticks):
    #     """
    #     Wait some time for update the visualization.
    #
    #     :param time_between_ticks: time to wait before trigger tick() in carla
    #     """
    #     dt = 0
    #     # wait for CARLA vehicles
    #     while dt < 80:
    #         self.client.get_world().tick()
    #         time.sleep(time_between_ticks)
    #         dt += 1
    #
    # def _update_vehicle_to_scenario(self, carla_contr_obs_classes: List[CarlaVehicleInterface]):
    #     """
    #     Update all vehicles created by carla vehicles auto_generation to our current scenario.
    #
    #     :param carla_contr_obs_classes: list of CarlaVehicleInterface object
    #     """
    #     new_dynamic_obs: List[DynamicObstacle] = []
    #     for obs in carla_contr_obs_classes:
    #         if obs.dynamic_obstacle and obs.commonroad_id:
    #             # rewrite prediction
    #             obs.dynamic_obstacle.prediction = TrajectoryPrediction(
    #                     trajectory=obs.dynamic_obstacle.prediction.trajectory,
    #                     shape=obs.dynamic_obstacle.obstacle_shape)
    #             new_dynamic_obs.append(obs.dynamic_obstacle)
    #     self.scenario.add_objects(new_dynamic_obs)
