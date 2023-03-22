import carla
import os
import sys
import subprocess
import signal
import logging
from typing import List, Optional, Tuple, Union
import pygame
import time
import copy

from commonroad.scenario.scenario import Scenario, Environment, TimeOfDay, Weather
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle, StaticObstacle
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.trajectory import Trajectory
from commonroad.common.solution import Solution, PlanningProblemSolution, VehicleType, VehicleModel, CostFunction
from commonroad_dc.feasibility.solution_checker import solution_feasible
from commonroad_dc.feasibility.vehicle_dynamics import VehicleParameterMapping
from crdesigner.map_conversion.map_conversion_interface import opendrive_to_commonroad

from carlacr.visualization.birds_eye_view import HUD2D, World2D
from carlacr.visualization.ego_view import HUD3D, World3D
from carlacr.helper.config import CarlaParams, CustomVis, VehicleControlType, WeatherParams
from carlacr.helper.traffic_generation import create_actors
from carlacr.helper.utils import create_cr_pm_state_from_actor, create_cr_ks_state_from_actor, \
    create_goal_region_from_state, find_pid_by_name, calc_max_timestep, make_video, find_carla_distribution
from carlacr.objects.traffic_light import CarlaTrafficLight, create_new_light, find_closest_traffic_light
from carlacr.objects.vehicle import VehicleInterface
from carlacr.objects.pedestrian import PedestrianInterface

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class CarlaInterface:
    """Main class of the CommonRoad-CARLA-Interface."""

    def __init__(self, config: CarlaParams = CarlaParams()):
        """
        Constructor of CarlaInterface.

        :param config: CARLA config dataclass.
        """
        self._config = config
        self._carla_pid = None

        if self._config.start_carla_server:
            self._start_carla_server()

        self._client = carla.Client(self._config.host, self._config.port)
        self._client.set_timeout(self._config.client_init_timeout)

        self._load_map(self._config.map)

        self._cr_obstacles: List[Union[VehicleInterface, PedestrianInterface]] = []
        self._ego: Optional[VehicleInterface] = None
        self.traffic_lights: List[CarlaTrafficLight] = []

        self._world = self._client.get_world()
        self._tm = self._client.get_trafficmanager()
        self._init_carla_world()
        self._init_carla_traffic_manager()

        # Initialize the Lists to save the states of the traffic lights
        for actor in self._world.get_actors():
            if "light" in actor.type_id:
                self.traffic_lights.append(CarlaTrafficLight(actor))

        self._world.set_weather(carla.WeatherParameters.WetSunset)
        self.set_weather(self._config.simulation.weather)

        logger.info("CARLA-Interface initialization finished.")

    def __del__(self):
        """Kill CARLA server in case it was started by the CARLA-Interface."""
        if self._carla_pid is not None:
            logger.info("Killing CARLA server.")
            os.killpg(os.getpgid(self._carla_pid.pid), signal.SIGTERM)
            time.sleep(self._config.sleep_time)

    def _start_carla_server(self):
        """Start CARLA server in desired operating mode (3D/offscreen)."""
        path_to_carla = os.path.join(find_carla_distribution(self._config.default_carla_paths), "CarlaUE4.sh")
        for pid in find_pid_by_name("CarlaUE4-Linux-"):
            logger.info("Kill existing CARLA servers.")
            os.killpg(os.getpgid(pid), signal.SIGTERM)
        logger.info("Start CARLA server.")
        # pylint: disable=consider-using-with
        if self._config.offscreen_mode:
            self._carla_pid = subprocess.Popen([path_to_carla, '-RenderOffScreen'],
                                               stdout=subprocess.PIPE, preexec_fn=os.setsid)
            logger.info("CARLA server started in off-screen mode using PID %s.", self._carla_pid.pid)
        else:
            self._carla_pid = subprocess.Popen([path_to_carla], stdout=subprocess.PIPE, preexec_fn=os.setsid)
            logger.info("CARLA server started in normal visualization mode using PID %s.", self._carla_pid.pid)
        time.sleep(self._config.sleep_time)

    def _init_carla_world(self):
        """Configures CARLA world."""
        logger.info("Init CARLA world.")
        settings = self._world.get_settings()
        settings.synchronous_mode = self._config.simulation.sync
        settings.fixed_delta_seconds = self._config.simulation.time_step
        settings.max_substep_delta_time = self._config.simulation.max_substep_delta_time
        settings.max_substeps = self._config.simulation.max_substeps
        settings.no_rendering_mode = self._config.vis_type == CustomVis.BIRD
        self._world.apply_settings(settings)

    def _init_carla_traffic_manager(self):
        """Configures CARLA traffic manager."""
        logger.info("Init CARLA traffic manager.")
        self._tm.set_global_distance_to_leading_vehicle(self._config.simulation.tm.global_distance_to_leading_vehicle)
        self._tm.global_percentage_speed_difference(self._config.simulation.tm.global_percentage_speed_difference)
        self._tm.set_hybrid_physics_mode(self._config.simulation.tm.hybrid_physics_mode)
        self._tm.set_hybrid_physics_radius(self._config.simulation.tm.hybrid_physics_radius)
        self._tm.set_synchronous_mode(self._config.sync)
        self._tm.set_random_device_seed(self._config.simulation.tm.seed)
        self._tm.set_osm_mode(self._config.simulation.tm.osm_mode)
        if hasattr(self._tm, "global_lane_offset"):  # starting in CARLA 0.9.14
            self._tm.global_lane_offset(self._config.simulation.tm.global_lane_offset)

    def _load_map(self, map_name: str):
        """
        Loads OpenDRIVE map into CARLA.

        @param map_name: Name of map (for CARLA default maps) or path to OpenDRIVE map.
        """
        if map_name[0:4] == "Town":
            logger.info("Load CARLA default map: %s", map_name)
            self._client.load_world(map_name)
            self._config.simulation.osm_mode = True
        elif os.path.exists(map_name):
            logger.info("Load OpenDRIVE map: %s", os.path.basename(map_name))
            with open(map_name, encoding='utf-8') as od_file:
                try:
                    data = od_file.read()
                except OSError:
                    logger.error("Failed load OpenDRIVE map: %s", os.path.basename(map_name))
                    sys.exit()
            logger.info("Loaded OpenDRIVE map: {os.path.basename(map_name)} successfully.")

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
        """
        Initializes obstacles and traffic lights from CommonRoad scenario.

        @param sc: CommonRoad scenario.
        """
        for obs in sc.obstacles:
            if obs.obstacle_type in [ObstacleType.CAR, ObstacleType.BUS, ObstacleType.TAXI, ObstacleType.TRUCK,
                                     ObstacleType.MOTORCYCLE, ObstacleType.BICYCLE]:
                self._cr_obstacles.append(VehicleInterface(obs, self._world, self._tm, config=self._config.vehicle))
            elif obs.obstacle_type == ObstacleType.PEDESTRIAN:
                self._cr_obstacles.append(PedestrianInterface(obs, self._world, self._tm,
                                                              config=self._config.pedestrian))

        for tl in sc.lanelet_network.traffic_lights:
            closest_tl = find_closest_traffic_light(self.traffic_lights, tl)
            if closest_tl is not None:
                logger.error("traffic light could not be matched")
                closest_tl.set_cr_light(tl)

        self._set_cr_weather(sc.location.environment)

    def solution(self, planning_problem_id: int, vehicle_model: VehicleModel, vehicle_type: VehicleType,
                 cost_function: CostFunction) -> PlanningProblemSolution:
        """
        Creates CommonRoad planning problem solution from driven ego vehicle trajectory.

        @param planning_problem_id: ID of new planning problem.
        @param vehicle_model: Vehicle model which should be used for planning problem.
        @param vehicle_type: Type of vehicle used for planning problem.
        @param cost_function: Cost function used for planning problem.
        :return: CommonRoad planning problem solution.
        """
        return PlanningProblemSolution(planning_problem_id=planning_problem_id, vehicle_model=vehicle_model,
                                       vehicle_type=vehicle_type, cost_function=cost_function,
                                       trajectory=Trajectory(self._ego.trajectory[0].time_step, self._ego.trajectory))

    def cr_obstacles(self) -> List[Union[DynamicObstacle, StaticObstacle]]:
        """
        Extracts Commonroad obstacles.

        :return: List of CommonRoad obstacles containing driven trajectory from CARLA vehicles and walkers.
        """
        for obs in self._cr_obstacles:
            obs.cr_obstacle.prediction = TrajectoryPrediction(Trajectory(1, obs.trajectory),
                                                              obs.cr_obstacle.obstacle_shape)
        return [obs.cr_obstacle for obs in self._cr_obstacles]

    def cr_ego_obstacle(self) -> DynamicObstacle:
        """
        Extracts Commonroad obstacle object for ego vehicle.

        :return: CommonRoad obstacle containing driven trajectory from CARLA ego vehicle.
        """
        self._ego.cr_obstacle.prediction = TrajectoryPrediction(Trajectory(1, self._ego.trajectory),
                                                                self._ego.cr_obstacle.obstacle_shape)
        return self._ego.cr_obstacle

    def create_cr_map(self) -> Scenario:
        """
        Converts the CARLA map to a Commonroad map using CommonRoad Scenario Designer.

        :return: Scenario containing converted map without obstacles.
        """
        carla_map = self._world.get_map()

        # Convert the CARLA map into OpenDRIVE in a temporary file
        with open("temp.xodr", "w", encoding='UTF-8') as file:
            file.write(carla_map.to_opendrive())
            file.close()

        # Load OpenDRIVE file, parse it, and convert it to a CommonRoad scenario
        scenario = opendrive_to_commonroad("./temp.xodr")

        # Delete temporary file
        os.remove("./temp.xodr")

        return scenario

    def replay(self, sc: Scenario, solution: Optional[Solution] = None, pps: Optional[PlanningProblemSet] = None,
               ego_id: Optional[int] = None):
        """
        Runs/Replays CommonRoad scenario in CARLA.

        @param sc: CommonRoad scenario.
        @param solution: CommonRoad solution which should be driven by ego vehicle.
        @param pps: Planning problem set corresponding to solution.
        @param ego_id: ID of ego vehicle in case an obstacle from the scenario should be used as ego vehicle.
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
            self._ego = VehicleInterface(ego_obs, self._world, self._tm)

        self._set_scenario(sc)

        self._run_simulation(obstacle_control=True, obstacle_only=obstacle_only)

    def _add_solution_to_scenario(self, ego_id: int, pps: PlanningProblemSet, solution: Solution) -> DynamicObstacle:
        """
        Creates CommonRoad dynamic obstacle given planning problem solution.

        @param ego_id: ID of new obstacle.
        @param pps: Planning problem required to extract trajectory.
        @param solution: Solution used to extract trajectory of new obstacle.
        """
        trajectory = solution_feasible(solution, self._config.simulation.time_step, pps)[ego_id][2]
        vehicle_params = VehicleParameterMapping.from_vehicle_type(solution.planning_problem_solutions[0].vehicle_type)
        shape = Rectangle(vehicle_params.l, vehicle_params.w)

        return DynamicObstacle(ego_id, ObstacleType.CAR, shape, pps.planning_problem_dict[ego_id].initial_state,
                               TrajectoryPrediction(trajectory, shape))

    def scenario_generation(self, sc: Scenario) -> Tuple[Scenario, PlanningProblemSet]:
        """
        Generates CommonRoad scenario given a map and the simulation config stored in the CARLA interface object.

        @param sc: Scenario containing map.
        :return: Generated CommonRoad scenario and planning problem set.
        """
        assert self._config.sync is True

        logger.info("Scenario generation: Create actors.")
        self._cr_obstacles = create_actors(self._client, self._world, self._tm, self._config.simulation,
                                           sc.generate_object_id())

        logger.info("Scenario generation: Start Simulation.")
        self._run_simulation(obstacle_only=True)

        for obs in self._cr_obstacles[1:]:
            obs.cr_obstacle.prediction = TrajectoryPrediction(Trajectory(1, obs.trajectory),
                                                              obs.cr_obstacle.obstacle_shape)
            sc.add_objects(obs.cr_obstacle)

        # define goal region
        if self._config.vehicle.vehicle_ks_state:
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
        """
        Executes keyboard control. Either a provided CommonRoad scenario with planning problem is used or
        a random vehicle from CARLA is used.

        @param sc: CommonRoad scenario.
        @param pp: CommonRoad planning problem.
        @param vehicle_type: CommonRoad vehicle type used for simulation.
        """
        logger.info("Start keyboard manual control.")

        if self._config.ego.controller_type is not VehicleControlType.KEYBOARD:
            self._config.ego.controller_type = VehicleControlType.KEYBOARD
            logger.info("Keyboard control type not set for ego! Will be set.")

        if pp is not None:
            vehicle_params = VehicleParameterMapping.from_vehicle_type(vehicle_type)
            ego_obs = DynamicObstacle(0, ObstacleType.CAR, Rectangle(vehicle_params.l, vehicle_params.w),
                                      pp.initial_state)
        else:
            ego_obs = None

        logger.info("Init Manual Control.")
        self._ego = VehicleInterface(ego_obs, self._world, self._tm, config=self._config.ego)

        if sc is not None:
            logger.info("Spawn CommonRoad obstacles.")
            self._set_scenario(sc)
            obstacle_control = True
        else:
            self._cr_obstacles = create_actors(self._client, self._world, self._tm, self._config.simulation, 1)
            obstacle_control = False

        logger.info("Spawn ego.")
        self._ego.tick(0)

        self._run_simulation(obstacle_control=obstacle_control)

    def _update_cr_state(self):
        """
        Stores CommonRoad obstacles and traffic lights states based on current world status.

        @param world: CARLA world object.
        """
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
            if self._config.vehicle.vehicle_ks_state:
                state = create_cr_ks_state_from_actor(self._ego.actor, time_step)
            else:
                state = create_cr_pm_state_from_actor(self._ego.actor, time_step)
            self._ego.trajectory.append(state)

        for obs in self._cr_obstacles:
            time_step = obs.cr_obstacle.initial_state.time_step + 1 if len(obs.trajectory) == 0 else \
                obs.trajectory[-1].time_step + 1

            if obs.cr_obstacle.obstacle_type == ObstacleType.PEDESTRIAN:
                state = create_cr_pm_state_from_actor(obs.actor, time_step)
            elif self._config.vehicle.vehicle_ks_state:
                state = create_cr_ks_state_from_actor(obs.actor, time_step)
            else:
                state = create_cr_pm_state_from_actor(obs.actor, time_step)
            obs.trajectory.append(state)

        for tl in self.traffic_lights:
            tl.add_color(tl.carla_actor.state)

    def _set_cr_weather(self, env: Environment):
        """
        Sets weather conditions specified in CommonRoad scenario.

        @param env: CommonRoad environment storing time of day, underground and weather.
        """
        # TODO consider underground conditions
        if env.time_of_day is not TimeOfDay.NIGHT:
            if env.weather is Weather.HEAVY_RAIN:
                self._world.set_weather(carla.WeatherParameters.HardRainNoon)
            elif env.weather is Weather.LIGHT_RAIN:
                self._world.set_weather(carla.WeatherParameters.SoftRainNoon)
            elif env.weather is Weather.FOG:
                pass
                # TODO set weather since fog in general supported by CARLA
            elif env.weather is Weather.HAIL:
                logger.info("CarlaInterface::set_cr_weather: Hail not supported by CARLA.")
            elif env.weather is Weather.SNOW:
                logger.info("CarlaInterface::set_cr_weather: Snow not supported by CARLA.")
            elif env.weather is Weather.SUNNY:
                self._world.set_weather(carla.WeatherParameters.ClearNoon)
        else:
            if env.weather is Weather.HEAVY_RAIN:
                self._world.set_weather(carla.WeatherParameters.HardRainNight)
            elif env.weather is Weather.LIGHT_RAIN:
                self._world.set_weather(carla.WeatherParameters.SoftRainNight)
            elif env.weather is Weather.FOG:
                pass
                # TODO set weather since fog in general supported by CARLA
            elif env.weather is Weather.HAIL:
                logger.info("CarlaInterface::set_cr_weather: Hail not supported by CARLA.")
            elif env.weather is Weather.SNOW:
                logger.info("CarlaInterface::set_cr_weather: Snow not supported by CARLA.")
            elif env.weather is Weather.SUNNY:
                self._world.set_weather(carla.WeatherParameters.ClearNight)

    def set_weather(self, config: WeatherParams = WeatherParams()):
        """
        Sets weather based on given config.

        @param config: Weather config parameters.
        """
        self._world.\
            set_weather(carla.WeatherParameters(config.cloudiness, config.precipitation, config.precipitation_deposits,
                                                config.wind_intensity, config.sun_azimuth_angle,
                                                config.sun_altitude_angle, config.fog_density, config.fog_distance,
                                                config.wetness, config.fog_falloff, config.scattering_intensity,
                                                config.mie_scattering_scale, config.rayleigh_scattering_scale))

    def _run_simulation(self, obstacle_control: bool = False, obstacle_only: bool = False):
        """
        Performs simulation by iteratively calling tick and render functions.
        Initializes visualization worlds and head-up display.

        @param obstacle_control: Boolean indicating whether obstacles are controlled based on CommonRoad scenario.
        @param obstacle_only: Boolean indicating whether only obstacles should be simulated, i.e. no ego vehicle.
        """
        vis_world = None
        time_step = 0
        clock = None
        display = None
        hud = None

        if self._config.vis_type is not CustomVis.NONE and not obstacle_only:
            display = self._init_display()
            clock = pygame.time.Clock()

        if self._config.vis_type is CustomVis.BIRD and not obstacle_only:
            logger.info("Init 2D.")
            hud = HUD2D("CARLA 2D", self._config.simulation.width, self._config.simulation.height)
            vis_world = World2D("CARLA 2D", self._world, hud, self._config.simulation, self._ego.actor)
        elif self._config.vis_type is CustomVis.EGO and not obstacle_only:
            logger.info("Init 3D.")
            hud = HUD3D(self._config.simulation)
            vis_world = World3D(self._world, hud, self._config.simulation, self._ego.actor)
        if self._ego is not None and self._ego.control_type is VehicleControlType.KEYBOARD:
            self._ego.register_clock(clock, hud, vis_world)

        logger.info("Loop.")
        while time_step <= self._config.simulation.max_time_step:
            if self._config.sync:
                self._world.tick()
            else:
                self._world.wait_for_tick()

            if self._ego is not None:
                self._ego.tick(time_step)
            if obstacle_control:
                for obs in self._cr_obstacles:
                    obs.tick(time_step)
                for tl in self.traffic_lights:
                    tl.tick(time_step)

            if self._config.vis_type is not CustomVis.NONE and not obstacle_only:
                clock.tick_busy_loop(60)
                vis_world.tick(clock)
                vis_world.render(display)
                pygame.display.flip()

            time_step += 1
            self._update_cr_state()

        if self._config.vis_type is CustomVis.EGO:
            vis_world.destroy_sensors()

        if self._config.simulation.record_video:
            make_video(self._config.simulation.video_path, self._config.simulation.video_name)

    def _init_display(self) -> pygame.display:
        """
        Initializes Pygame display for 3D ego view.

        :return: Initialized pygame display object.
        """
        pygame.init()
        pygame.font.init()
        display = pygame.display.set_mode((self._config.simulation.width, self._config.simulation.height),
                                          pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption(self._config.simulation.description)  # Place a title to game window
        # Show loading screen
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        text_surface = font.render('Rendering map...', True, pygame.Color(255, 255, 255))
        display.blit(text_surface, text_surface.get_rect(center=(self._config.simulation.width / 2,
                                                                 self._config.simulation.height / 2)))
        display.fill((0, 0, 0))
        pygame.display.flip()
        return display
