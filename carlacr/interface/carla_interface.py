import carla
import os
import sys
import subprocess
import signal
import logging
import numpy as np
from typing import List, TypeVar, Optional, Tuple, Dict
import pygame
import time
import psutil
import copy

from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.trajectory import Trajectory
from commonroad.common.solution import Solution, PlanningProblemSolution, VehicleType, VehicleModel, CostFunction
from commonroad_dc.feasibility.solution_checker import solution_feasible
from commonroad_dc.feasibility.vehicle_dynamics import VehicleParameterMapping
from crdesigner.map_conversion.map_conversion_interface import opendrive_to_commonroad

from carlacr.game.birds_eye_view import HUD2D, World2D
from carlacr.game.ego_view import HUD3D, World3D
from carlacr.interface.obstacle.ego_interface import EgoInterface
from carlacr.interface.obstacle.keyboard import KeyboardEgoInterface
from carlacr.helper.config import CarlaParams, CustomVis
from carlacr.interface.obstacle.obstacle_interface import ObstacleInterface
from carlacr.interface.obstacle.pedestrian_interface import PedestrianInterface
from carlacr.helper.traffic_generation import create_actors
from carlacr.helper.utils import create_cr_pm_state_from_actor, create_cr_ks_state_from_actor, \
    create_goal_region_from_state
from carlacr.interface.traffic_light import CarlaTrafficLight, create_new_light
from carlacr.interface.obstacle.cr_replay_ego import CommonRoadObstacleInterface

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

        self._cr_obstacles: List[ObstacleInterface] = []
        self._ego: Optional[EgoInterface] = None
        self.traffic_lights: List[CarlaTrafficLight] = []

        # Initialize the Lists to save the states of the traffic lights
        for actor in self._client.get_world().get_actors():
            if "light" in actor.type_id:
                self.traffic_lights.append(CarlaTrafficLight(actor.id))
                self.traffic_lights[-1].set_initial_color(actor.state)

    def __del__(self):
        """Kill CARLA server in case it was started by the CARLA-Interface."""
        if self._carla_pid is not None:
            logger.info("Killing CARLA server.")
            os.killpg(os.getpgid(self._carla_pid.pid), signal.SIGTERM)
            time.sleep(self._config.sleep_time)

    def _start_carla_server(self):
        """Start CARLA server in desired operating mode (3D/offscreen)."""
        path_to_carla = os.path.join(self._find_carla_distribution(), "CarlaUE4.sh")
        logger.info("Kill existing CARLA servers.")
        for pid in self._find_pid_by_name("CarlaUE4-Linux-"):
            os.killpg(os.getpgid(pid), signal.SIGTERM)
        logger.info("Start CARLA server.")
        if self._config.offscreen_mode:
            self._carla_pid = subprocess.Popen([path_to_carla, '-RenderOffScreen'], stdout=subprocess.PIPE,
                                               preexec_fn=os.setsid)
            logger.info(f"CARLA server started in off-screen mode using PID {self._carla_pid.pid}.")
        else:
            self._carla_pid = subprocess.Popen([path_to_carla], stdout=subprocess.PIPE, preexec_fn=os.setsid)
            logger.info(f"CARLA server started in normal visualization mode using PID {self._carla_pid.pid}.")
        time.sleep(self._config.sleep_time)

    def _find_pid_by_name(self, process_name: str) -> List[int]:
        '''
        Get a list of all the PIDs of a all the running process whose name contains
        the given string processName
        '''
        processes = []
        # Iterate over the all the running process
        for proc in psutil.process_iter():
            try:
                # Check if process name contains the given name string.
            #    print(proc.name().lower())
                if process_name.lower() in proc.name().lower():
                    processes.append(proc.pid)
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        return processes

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
        settings.no_rendering_mode = self._config.vis_type == CustomVis.BIRD
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

    def _set_scenario(self, sc: Scenario, waypoint_control: bool = False):
        for obs in sc.obstacles:
            if obs.obstacle_type in [ObstacleType.CAR, ObstacleType.BUS, ObstacleType.TAXI, ObstacleType.TRUCK,
                                     ObstacleType.MOTORCYCLE, ObstacleType.BICYCLE]:
                self._cr_obstacles.append(CommonRoadObstacleInterface(obs, waypoint_control=waypoint_control))
            elif obs.obstacle_type == ObstacleType.PEDESTRIAN:
                self._cr_obstacles.append(PedestrianInterface(obs, waypoint_control=waypoint_control))

        # TODO: set traffic light cycle


    def solution(self, planning_problem_id: int, vehicle_model: VehicleModel, vehicle_type: VehicleType,
                 cost_function: CostFunction) -> PlanningProblemSolution:
        return PlanningProblemSolution(planning_problem_id=planning_problem_id,
                 vehicle_model=vehicle_model,
                 vehicle_type=vehicle_type,
                 cost_function=cost_function,
                 trajectory=Trajectory(self._ego.trajectory[0].time_step, self._ego.trajectory))

    def create_cr_map(self) -> Scenario:
        """Converts the CARLA map to Commonroad."""
        carla_map = self._client.get_world().get_map()

        # Convert the CARLA map into OpenDRIVE in a temporary file
        f = open("temp.xodr", "w")
        f.write(carla_map.to_opendrive())
        f.close()

        # Load OpenDRIVE file, parse it, and convert it to a CommonRoad scenario
        scenario = opendrive_to_commonroad("./temp.xodr")

        # Delete temporary file
        os.remove("./temp.xodr")

        return scenario


    def replay(self, sc: Scenario, solution: Optional[Solution] = None, pps: PlanningProblemSet = None,
               ego_id: Optional[int] = None, store_video: bool = False, waypoint_control = False):
        """
        Runs CommonRoad scenario in CARLA.
        """
        assert solution is None or ego_id is None
        assert solution is None and pps is None or solution is not None and pps is not None

        obstacle_only = False
        if solution is not None:
            ego_id = list(pps.planning_problem_dict.keys())[0]
            ego_obs = self._add_solution_to_scenario(ego_id, pps, solution)
            self._config.simulation.max_time_step = len(ego_obs.prediction.trajectory.state_list)
        else:
            self._config.simulation.max_time_step = calc_max_timestep(sc)
            if ego_id is not None:
                ego_obs = sc.obstacle_by_id(ego_id)
                sc.remove_obstacle(ego_obs)
            else:
                ego_obs = None
                obstacle_only = True

        if ego_id is not None:
            self._ego = CommonRoadObstacleInterface(ego_obs, waypoint_control)

        self._set_scenario(sc, waypoint_control)

        self._run_simulation(obstacle_control=True, obstacle_only=obstacle_only)

    def _add_solution_to_scenario(self, ego_id, pps, solution) -> DynamicObstacle:
        trajectory = solution_feasible(solution, self._config.simulation.time_step, pps)[ego_id][2]
        vehicle_params = VehicleParameterMapping.from_vehicle_type(solution.planning_problem_solutions[0].vehicle_type)
        shape = Rectangle(vehicle_params.l, vehicle_params.w)

        return DynamicObstacle(ego_id, ObstacleType.CAR, shape, pps.planning_problem_dict[ego_id].initial_state,
                            TrajectoryPrediction(trajectory, shape))


    def scenario_generation(self, sc: Scenario) -> Tuple[Scenario, PlanningProblemSet]:
        assert self._config.sync is True

        logger.info("Scenario generation: Create actors.")
        self._cr_obstacles = create_actors(self._client, self._config.simulation, sc.generate_object_id())

        logger.info("Scenario generation: Start Simulation.")
        self._run_simulation(obstacle_only=True)

        for obs in self._cr_obstacles[1:]:
            obs.cr_obstacle.prediction = TrajectoryPrediction(Trajectory(1, obs.trajectory),
                                                              obs.cr_obstacle.obstacle_shape)
            sc.add_objects(obs.cr_obstacle)

        # define goal region
        if self._config.obstacle.vehicle_ks_state:
            goal_region = create_goal_region_from_state(self._cr_obstacles[0].trajectory[-1])
        else:
            goal_region = create_goal_region_from_state(self._cr_obstacles[0].trajectory[-1], False)

        old_lights = copy.deepcopy(sc.lanelet_network.traffic_lights)
        for light in old_lights:
            new_light = create_new_light(light, self.traffic_lights)
            sc.remove_traffic_light(light)
            sc.add_objects(new_light)

        return sc, PlanningProblemSet([PlanningProblem(sc.generate_object_id(),
                                                       self._cr_obstacles[0].cr_obstacle.initial_state,
                                                       goal_region)])

    def keyboard_control(self, sc: Scenario = None, pp: PlanningProblem = None,
                         vehicle_type: VehicleType = VehicleType.BMW_320i):
        logger.info("Start keyboard manual control.")

        if pp is not None:
            vehicle_params = VehicleParameterMapping.from_vehicle_type(vehicle_type)
            ego_obs = DynamicObstacle(0, ObstacleType.CAR, Rectangle(vehicle_params.l, vehicle_params.w),
                                      pp.initial_state)
        else:
            ego_obs = None

        logger.info("Init Manual Control.")
        self._ego = KeyboardEgoInterface(ego_obs)

        sim_world = self._client.get_world()

        if sc is not None:
            logger.info("Spawn CommonRoad obstacles.")
            self._set_scenario(sc)
            obstacle_control = True
      #  else:
       #     self._cr_obstacles = create_actors(self._client, self._config.simulation, 1)
        obstacle_control = False

        logger.info("Spawn ego.")
        self._ego.spawn(sim_world, 0)

        self._run_simulation(obstacle_control=obstacle_control)

    def update_cr_state(self, world: carla.World):
        # add current state to history
        # self._ego.trajectory.append(self._ego.cr_obstacle.initial_state)  # TODO replace with cr-io history

        # get world and extract new current state

        # TODO replace with cr-io initial state
        # if self._config.obstacle.vehicle_ks_state:
        #     self._ego.cr_obstacle.initial_state = \
        #         create_cr_ks_state_from_actor(world.get_actor(self._ego.carla_id),
        #                                    self._ego.cr_obstacle.initial_state.time_step + 1)
        # else:
        #     self._ego.cr_obstacle.initial_state = \
        #         create_cr_pm_state_from_actor(world.get_actor(self._ego.carla_id),
        #                                    self._ego.cr_obstacle.initial_state.time_step + 1)
        if self._ego is not None:
            time_step = self._ego.cr_obstacle.initial_state.time_step + 1 if len(self._ego.trajectory) == 0 else \
                self._ego.trajectory[-1].time_step + 1
            if self._config.obstacle.vehicle_ks_state:
                state = create_cr_ks_state_from_actor(world.get_actor(self._ego.carla_id), time_step)
            else:
                state = create_cr_pm_state_from_actor(world.get_actor(self._ego.carla_id), time_step)
            self._ego.trajectory.append(state)

        for obs in self._cr_obstacles:
            time_step = obs.cr_obstacle.initial_state.time_step + 1 if len(obs.trajectory) == 0 else \
                obs.trajectory[-1].time_step + 1

            actor = world.get_actor(obs.carla_id)

            if obs.get_type() == ObstacleType.PEDESTRIAN:
                state = create_cr_pm_state_from_actor(actor, time_step)
            elif self._config.obstacle.vehicle_ks_state:
                state = create_cr_ks_state_from_actor(actor, time_step)
            else:
                state = create_cr_pm_state_from_actor(actor, time_step)
            obs.trajectory.append(state)

        for tl in self.traffic_lights:
            tl.add_color(world.get_actor(tl.carla_id).state)

    def _run_simulation(self, obstacle_control: bool = False, obstacle_only: bool = False):
        sim_world = self._client.get_world()
        tm = self._client.get_trafficmanager()
        world = None

        hud = None
        time_step = 0
        clock = None
        display = None

        if self._config.vis_type is not CustomVis.NONE and not obstacle_only:
            self._ego.spawn(sim_world, time_step, tm)
            display = self._init_display()
            clock = pygame.time.Clock()

        if self._config.vis_type is CustomVis.BIRD and not obstacle_only:
            logger.info("Init 2D.")
            hud = HUD2D("CARLA 2D", self._config.simulation.width, self._config.simulation.height)
            world = World2D("CARLA 2D", sim_world, hud, self._config.simulation, sim_world.get_actor(self._ego.carla_id))
        elif self._config.vis_type is CustomVis.EGO and not obstacle_only:
            logger.info("Init 3D.")
            hud = HUD3D(self._config.simulation)
            world = World3D(sim_world, hud, self._config.simulation, sim_world.get_actor(self._ego.carla_id))

        logger.info("Loop.")
        while time_step <= self._config.simulation.max_time_step:
            if self._config.sync:
                sim_world.tick()
            else:
                sim_world.wait_for_tick()

            if self._ego is not None:
                self._ego.tick(clock, sim_world, tm)

            if self._config.vis_type is not CustomVis.NONE and not obstacle_only:
                clock.tick_busy_loop(60)
                world.tick(clock)
                world.render(display)
                pygame.display.flip()

            if obstacle_control:
                for obs in self._cr_obstacles:
                    if obs.get_type() is not ObstacleType.PEDESTRIAN:
                        obs.tick(clock, sim_world, tm)
                    else:
                        obs.tick(sim_world)
            time_step += 1
            self.update_cr_state(sim_world)

    def _init_display(self):
        pygame.init()
        pygame.font.init()
        display = pygame.display.set_mode((self._config.simulation.width, self._config.simulation.height),
                pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption(self._config.simulation.description)  # Place a title to game window
        # Show loading screen
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        text_surface = font.render('Rendering map...', True, pygame.Color(255, 255, 255))
        display.blit(text_surface, text_surface.get_rect(
                center=(self._config.simulation.width / 2, self._config.simulation.height / 2)))
        display.fill((0, 0, 0))
        pygame.display.flip()
        return display
