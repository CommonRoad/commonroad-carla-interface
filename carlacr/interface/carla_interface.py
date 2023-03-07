import carla
import os
import sys
import subprocess
import time
import signal
import logging
import numpy as np
from typing import List, TypeVar, Optional
import pygame

from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.obstacle import ObstacleRole, ObstacleType, DynamicObstacle
from commonroad.geometry.shape import Rectangle

from carlacr.game.birds_eye_view import HUD2D, World2D
from carlacr.game.ego_view import HUD3D, World3D
from carlacr.interface.obstacle.ego_interface import EgoInterface
from carlacr.interface.obstacle.keyboard import KeyboardEgoInterface2D, KeyboardEgoInterface3D
from carlacr.helper.config import CarlaParams
from carlacr.interface.obstacle.vehicle_interface import VehicleInterface
from carlacr.interface.obstacle.obstacle_interface import ObstacleInterface
from carlacr.interface.obstacle.pedestrian_interface import PedestrianInterface
from carlacr.helper.traffic_generation import create_actors

logger = logging.getLogger(__name__)

EI = TypeVar('EI', bound=EgoInterface)


# This module contains helper methods for the Carla-CommonRoad Interface
def calc_max_timestep(sc: Scenario) -> int:
    """
    Calculates maximal time step of current scenario.

    :param sc: scenario to calculate max time step
    :return: length of scenario
    """
    time_steps = [obstacle.prediction.final_time_step for obstacle in sc.dynamic_obstacles]
    return np.max(time_steps) if time_steps else 0


class CarlaInterface:
    """Main class of the CommonRoad-CARLA-Interface."""

    def __init__(self, config: CarlaParams = CarlaParams()):
        """
        Constructor of CarlaInterface.

        :param config: CARLA config dataclass.
        map
        """
        self._config = config
        self._carla_pid = None

        if self._config.start_carla_server:
            self._start_carla_server()
        self._client = carla.Client(self._config.host, self._config.port)
        self._client.set_timeout(self._config.client_init_timeout)
        self._init_carla_world()
        self._init_carla_traffic_manager()
        self._load_map(self._config.map)
        sys.path.append(os.path.join(self._find_carla_distribution(), "PythonAPI"))

        # CR_PLANNING Mode
        # if self._config.operating_mode is OperatingMode.:
        self._cr_obstacles: List[ObstacleInterface] = []
        self._ego = None

        # if carla_planning_mode

        # if scenario_generation

        # if replay_mode

    def __del__(self):
        """Kill CARLA server in case it was started by the CARLA-Interface."""
        if self._carla_pid is not None:
            logger.info("Killing CARLA server.")
            os.killpg(os.getpgid(self._carla_pid.pid), signal.SIGTERM)
            time.sleep(self._config.sleep_time)

    def _start_carla_server(self):
        """Start CARLA server in desired operating mode (3D/offscreen)."""
        path_to_carla = os.path.join(self._find_carla_distribution(), "CarlaUE4.sh")
        logger.info("Start CARLA server.")
        if self._config.offscreen_mode:
            self._carla_pid = subprocess.Popen([path_to_carla, '-RenderOffScreen'], stdout=subprocess.PIPE,
                                               preexec_fn=os.setsid)
            logger.info(f"CARLA server started in off-screen mode using PID {self._carla_pid.pid}.")
        else:
            self._carla_pid = subprocess.Popen([path_to_carla], stdout=subprocess.PIPE, preexec_fn=os.setsid)
            logger.info(f"CARLA server started in normal visualization mode using PID {self._carla_pid.pid}.")
        time.sleep(self._config.sleep_time)

    def _find_carla_distribution(self) -> str:
        """Searches for CARLA executable in provided paths.

        :returns Path to CARLa executable as string.
        """
        logger.info("Search CARLA server executable.")
        for default_path in self._config.default_carla_paths:
            path = default_path.replace("/~", os.path.expanduser("~"))
            if os.path.exists(path):
                return path
        raise FileNotFoundError("CARLA executable not found.")

    def _init_carla_world(self):
        """Configures CARLA world."""
        logger.info("Init CARLA world.")
        world = self._client.get_world()
        # Synchrony:
        # Warning: If synchronous mode is enabled, and there is a Traffic Manager running, this must be set to sync
        # mode too. Read this to learn how to do it.
        # See: https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/#setting-synchronous-mode
        settings = world.get_settings()
        settings.synchronous_mode = self._config.simulation.sync
        settings.fixed_delta_seconds = self._config.simulation.time_step
        settings.max_substep_delta_time = self._config.simulation.max_substep_delta_time
        settings.max_substeps = self._config.simulation.max_substeps
        settings.no_rendering_mode = self._config.birds_eye_view
        world.apply_settings(settings)

    def _init_carla_traffic_manager(self):
        """Configures CARLA traffic manager."""
        logger.info("Init CARLA traffic manager.")
        traffic_manager = self._client.get_trafficmanager(self._config.simulation.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(
                self._config.simulation.global_distance_to_leading_vehicle)
        traffic_manager.global_percentage_speed_difference(self._config.simulation.global_percentage_speed_difference)
        traffic_manager.set_hybrid_physics_mode(self._config.simulation.hybrid_physics_mode)
        traffic_manager.set_hybrid_physics_radius(self._config.simulation.hybrid_physics_radius)
        traffic_manager.set_synchronous_mode(self._config.sync)
        traffic_manager.set_random_device_seed(self._config.simulation.seed)
        traffic_manager.set_osm_mode(self._config.simulation.osm_mode)

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
            logger.info(f"Load CARLA default map: {map_name}")
            self._client.load_world(map_name)
            self._config.simulation.osm_mode = True
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
                    vertex_distance=self._config.map_params.vertex_distance,
                    max_road_length=self._config.map_params.max_road_length,
                    wall_height=self._config.map_params.wall_height,
                    additional_width=self._config.map_params.extra_width,
                    smooth_junctions=True,
                    enable_mesh_visibility=True,
                    enable_pedestrian_navigation=True))
        time.sleep(self._config.sleep_time)

    def _set_scenario(self, sc: Scenario):
        for obs in sc.obstacles:
            if obs.obstacle_type in [ObstacleType.CAR, ObstacleType.BUS, ObstacleType.TAXI, ObstacleType.TRUCK,
                                     ObstacleType.MOTORCYCLE, ObstacleType.BICYCLE]:
                self._cr_obstacles.append(VehicleInterface(obs))
            elif obs.obstacle_type == ObstacleType.PEDESTRIAN:
                self._cr_obstacles.append(PedestrianInterface(obs))

        # TODO: set traffic light cycle
        self._spawn_cr_obstacles()

    def _spawn_cr_obstacles(self):
        for obs in self._cr_obstacles:
            obs.spawn(self._client.get_world(), 0)

    def replay(self, sc: Scenario, waypoint_control = False):
        """
        Runs the CommonRoad Scenario in CARLA.

        :param time_step_delta_real: sets the time that will be waited in real time between the time steps,
        if None the dt of the scenario will be used
        :param carla_vehicles: maximum number of vehicles that should be created & controlled by CARLA additional to
        the objects defined in the scenario
        :param carla_pedestrians: maximum number of pedestrians that should be created & controlled by CARLA
        additional to the objects defined in the scenario
        """
        self._set_scenario(sc)
        for time_step in range(calc_max_timestep(sc)):
            if not waypoint_control:
                logger.info(f"Replay time step: {time_step}.")
                self._control_commonroad_obstacles_path_dynamic(time_step)
            else:
                logger.info(f"Replay time step: {time_step}.")
                self._control_commonroad_obstacles_waypoint(time_step)
            self._client.get_world().tick(1)  # todo time step

    def _control_commonroad_obstacles_path_dynamic(self, curr_time_step: int):
        for obs in self._cr_obstacles:
            if not obs.is_spawned:
                obs.spawn(self._client.get_world(), curr_time_step)
            elif obs.get_role() == ObstacleRole.DYNAMIC:
                tm = self._client.get_trafficmanager()
                tm.set_path(self._client.get_world().get_actor(obs.carla_id), obs.get_path())

    def _control_commonroad_obstacles_waypoint(self, curr_time_step: int):
        """
        Control CommonRoad obstacles, spawn, update position and destroy regarding actor in carla if out of scenario.

        :param interface_obstacles: list of CommonRoadObstacleInterface object
        :param carla_interface_obstacles: list of tuple (interface object,actor)
        :param curr_time_step: current time step of the scenario
        :param mode: update obstacle "by-time", "by-control", or "by-ackermann-control"
        """
        for obs in self._cr_obstacles:
            if not obs.is_spawned:
                obs.spawn(self._client.get_world(), curr_time_step)
            else:
                obs.control(obs.state_at_time_step(curr_time_step))

    def keyboard_control(self, sc: Scenario = None, pp: PlanningProblem = None):
        logger.info("Start keyboard manual control.")

        if pp is not None:
            ego_obs = DynamicObstacle(0, ObstacleType.CAR, Rectangle(5, 2), pp.initial_state)
        else:
            ego_obs = None

        if self._config.birds_eye_view:
            logger.info("Init 2D Manual Control.")
            self._ego = KeyboardEgoInterface2D("2D Manual Control", ego_obs)
        else:
            logger.info("Init 3D Manual Control.")
            self._ego = KeyboardEgoInterface2D("2D Manual Control", ego_obs)

        self._run_simulation(sc, pp)

    def _run_simulation(self, sc: Optional[Scenario] = None, pp: Optional[PlanningProblem] = None):
        sim_world = self._client.get_world()

        if sc is not None:
            logger.info("Spawn CommonRoad obstacles.")
            self._set_scenario(sc)
        else:
            create_actors(self._client, self._config.simulation)

        logger.info("Spawn ego.")
        self._ego.spawn(sim_world, 0)

        COLOR_ALUMINIUM_4 = pygame.Color(85, 87, 83)
        COLOR_WHITE = pygame.Color(255, 255, 255)
        pygame.init()
        pygame.font.init()
        world = None
        try:
            display = pygame.display.set_mode(
                    (self._config.keyboard_control.width, self._config.keyboard_control.height),
                    pygame.HWSURFACE | pygame.DOUBLEBUF)

            pygame.display.set_caption(self._config.keyboard_control.description)  # Place a title to game window

            # Show loading screen
            font = pygame.font.Font(pygame.font.get_default_font(), 20)
            text_surface = font.render('Rendering map...', True, COLOR_WHITE)
            display.blit(text_surface, text_surface.get_rect(
                    center=(self._config.keyboard_control.width / 2, self._config.keyboard_control.height / 2)))
            display.fill((0, 0, 0))
            pygame.display.flip()

            if self._config.birds_eye_view:
                logger.info("Init 2D.")
                hud = HUD2D("CARLA 2D", self._config.keyboard_control.width, self._config.keyboard_control.height)
                world = World2D("CARLA 2D", self._config.keyboard_control, sim_world.get_actor(self._ego._carla_id))

                # For each module, assign other modules that are going to be used inside that module
                logger.info("Register 2D.")
                self._ego.start(hud, world)
                hud.start()
                world.start(hud, sim_world)

                # Game loop
                clock = pygame.time.Clock()
                logger.info("Loop 2D.")
                while True:
                    clock.tick_busy_loop(60)
                    print(clock.get_time())

                    # Tick all modules
                    world.tick(clock)
                    hud.tick(clock)
                    self._ego.tick(clock)

                    # Render all modules
                    display.fill(COLOR_ALUMINIUM_4)
                    world.render(display)
                    hud.render(display)

                    pygame.display.flip()
            else:
                logger.info("Init 3D.")
                hud = HUD3D(self._config.keyboard_control.width, self._config.keyboard_control.height)
                world = World3D(sim_world, hud, self._config.keyboard_control, sim_world.get_actor(self._ego._carla_id))
                controller = KeyboardEgoInterface3D(world, self._config.autopilot)

                if self._config.sync:
                    sim_world.tick()
                else:
                    sim_world.wait_for_tick()

                clock = pygame.time.Clock()
                while True:
                    if self._config.sync:
                        sim_world.tick()
                    clock.tick_busy_loop(60)
                    if controller.parse_events(self._client, world, clock, self._config.sync):
                        return
                    world.tick(clock)
                    world.render(display)
                    pygame.display.flip()

        finally:
            if world is not None:
                world.destroy()
