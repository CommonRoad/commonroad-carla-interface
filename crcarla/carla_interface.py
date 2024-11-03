import copy
import logging
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple, Union

import carla
import pygame
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import (
    CostFunction,
    PlanningProblemSolution,
    Solution,
    VehicleModel,
    VehicleType,
)
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planner_interface import TrajectoryPlannerInterface
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleRole, ObstacleType, StaticObstacle
from commonroad.scenario.scenario import Environment, Scenario, Weather, TimeOfDay
from commonroad.scenario.trajectory import Trajectory
from commonroad_dc.feasibility.solution_checker import solution_feasible
from commonroad_dc.feasibility.vehicle_dynamics import VehicleParameterMapping
from crdesigner.common.config.opendrive_config import open_drive_config
from crdesigner.map_conversion.map_conversion_interface import (
    commonroad_to_opendrive,
    opendrive_to_commonroad,
)
from crpred.predictor_interface import PredictorInterface

from crcarla.helper.config import CarlaParams, CustomVis, EgoPlanner, WeatherParams
from crcarla.helper.traffic_generation import create_actors
from crcarla.helper.utils import (
    calc_max_timestep,
    create_cr_ks_state_from_actor,
    create_cr_pm_state_from_actor,
    create_goal_region_from_state,
    find_carla_distribution,
    init_camera_sensor,
    kill_existing_servers,
    make_video,
)
from crcarla.objects.pedestrian import PedestrianInterface
from crcarla.objects.traffic_light import (
    CarlaTrafficLight,
    create_new_light,
    find_closest_traffic_light,
)
from crcarla.objects.vehicle import VehicleInterface
from crcarla.visualization.visualization2D import HUD2D, World2D
from crcarla.visualization.visualization3D import Visualization3D


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
        self._ego_planner = None
        # Initialize the Lists to save the states of the traffic lights
        for actor in self._world.get_actors():
            if "light" in actor.type_id:
                self.traffic_lights.append(CarlaTrafficLight(actor, self._config.logger))
        self.set_weather(self._config.simulation.weather)
        self._pp = None  # CommonRoad planning problem

        self._config.logger.info("CARLA-Interface initialization finished.")

    def _reset_environment(self, update_map: bool = True):
        """
        Reset map, planner, etc. to values specified in config.
        Does not update CARLA world settings. We assume this stays the same, e.g., synchronous mode,
        time step size, rendering mode, etc.

        :param update_map: Boolean indicating whether the map should be updated.
        """
        if update_map:
            self._load_map(self._config.map)
            self._world = self._client.get_world()
            print(self._world.get_settings())
            # self._init_carla_world()
        self._cr_obstacles: List[Union[VehicleInterface, PedestrianInterface]] = []
        self._ego: Optional[VehicleInterface] = None
        self.traffic_lights: List[CarlaTrafficLight] = []
        self._update_traffic_manager()
        self._ego_planner = None
        for actor in self._world.get_actors():
            if "light" in actor.type_id:
                self.traffic_lights.append(CarlaTrafficLight(actor, self._config.logger))
        self.set_weather(self._config.simulation.weather)
        self._pp = None

    def __del__(self):
        """Kill CARLA server in case it was started by the CARLA-Interface."""
        if self._config.kill_carla_server:
            self.cleanup()

    def __enter__(self):
        """For use with context-manager."""
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Kill CARLA server in case it was started by the CARLA-Interface."""
        if self._config.kill_carla_server:
            self.cleanup()

    def cleanup(self):
        """Kill CARLA server in case it was started by the CARLA-Interface."""
        self._config.logger.debug("Cleanup CARLA.")

        if self._carla_pid is not None:
            try:
                os.killpg(os.getpgid(self._carla_pid.pid), 0)
            except ProcessLookupError:
                self._config.logger.warning("CARLA server with PID %s has already terminated.", self._carla_pid.pid)
            else:
                os.killpg(os.getpgid(self._carla_pid.pid), signal.SIGTERM)
                self._carla_pid.wait()
                if self._carla_pid.poll() is None:
                    self._config.logger.warning(
                        "CARLA server with PID %s did not terminate. Sending SIGKILL.",
                        self._carla_pid.pid,
                    )
                    os.killpg(os.getpgid(self._carla_pid.pid), signal.SIGKILL)
                self._config.logger.info("CARLA server with PID %s terminated.", self._carla_pid.pid)

            time.sleep(self._config.sleep_time)

            kill_existing_servers(self._config.sleep_time)

    def update_config(self, config: CarlaParams, update_map: bool = True):
        """
        Update CARLA interface config.

        :param config: CARLA config parameters.
        :param update_map: Boolean indicating whether the map should be updated.
        """
        self._config = config
        self._reset_environment(update_map)

    def get_simulation_fps(self) -> int:
        """
        Returns the frames per second (fps) of the simulation.
        :return: fps as int
        """
        settings = self._world.get_settings()
        if settings.fixed_delta_seconds is not None:
            fps = 1.0 / settings.fixed_delta_seconds
            if int(fps) != fps:
                logging.warning("Calculated fps is not a whole number: %s", fps)
            return int(fps)
        return 30

    def _start_carla_server(self):
        """Start CARLA server in desired operating mode (3D/offscreen)."""
        path_to_carla = find_carla_distribution(self._config.default_carla_paths) / "CarlaUE4.sh"

        kill_existing_servers(self._config.sleep_time)

        self._config.logger.info("Start CARLA server.")
        # pylint: disable=consider-using-with

        popen_base_params = {"stdout": subprocess.PIPE, "preexec_fn": os.setsid, "shell": False}

        if self._config.offscreen_mode:
            cmd = [str(path_to_carla), "-RenderOffScreen", f"-carla-world-port={self._config.port}"]
        else:
            cmd = [str(path_to_carla), f"-carla-world-port={self._config.port}"]

        self._carla_pid = subprocess.Popen(cmd, **popen_base_params)
        self._config.logger.info("CARLA server started in normal visualization mode using PID %s.", self._carla_pid.pid)

        time.sleep(self._config.sleep_time)

    def _init_carla_world(self):
        """Configures CARLA world."""
        self._config.logger.info("Init CARLA world.")
        settings = self._world.get_settings()
        settings.synchronous_mode = self._config.simulation.sync
        settings.fixed_delta_seconds = self._config.simulation.time_step
        settings.max_substep_delta_time = self._config.simulation.max_substep_delta_time
        settings.max_substeps = self._config.simulation.max_substeps
        settings.no_rendering_mode = self._config.vis_type == CustomVis.BIRD
        self._world.apply_settings(settings)

    def _init_carla_traffic_manager(self):
        """Configures CARLA traffic manager."""
        self._config.logger.info("Init CARLA traffic manager.")
        self._tm.set_hybrid_physics_mode(self._config.simulation.tm.hybrid_physics_mode)
        self._tm.set_hybrid_physics_radius(self._config.simulation.tm.hybrid_physics_radius)
        self._tm.set_synchronous_mode(self._config.sync)
        self._tm.set_osm_mode(True)
        self._tm.set_random_device_seed(self._config.simulation.tm.seed)
        self._update_traffic_manager()

    def _update_traffic_manager(self):
        """Updates traffic"""
        self._tm.set_global_distance_to_leading_vehicle(self._config.simulation.tm.global_distance_to_leading_vehicle)
        self._tm.global_percentage_speed_difference(self._config.simulation.tm.global_percentage_speed_difference)
        if hasattr(self._tm, "global_lane_offset"):  # starting in CARLA 0.9.14
            self._tm.global_lane_offset(self._config.simulation.tm.global_lane_offset)

    def _load_map(self, map_name: Union[Path, str]):
        """
        Loads OpenDRIVE map into CARLA.

        :param map_name: Name of map (for CARLA default maps) or path to OpenDRIVE map.
        """
        if isinstance(map_name, str):
            map_name = Path(map_name)
        if map_name.stem[0:4] == "Town":
            self._config.logger.info("Load CARLA default map: %s", map_name)
            self._client.load_world(map_name.stem)
        elif map_name.exists() and map_name.suffix == ".xodr":
            self._config.logger.info("Load OpenDRIVE map: %s", map_name.stem)
            self._load_and_generate_odr(map_name)
        elif map_name.exists() and map_name.suffix in [".xml"]:
            # convert CR map to OpenDRIVE and store map temporary
            scenario, _ = CommonRoadFileReader(map_name).open()
            commonroad_to_opendrive(map_name, Path("./temp.xodr"))
            self._load_and_generate_odr(Path("./temp.xodr"))
            Path("./temp.xodr").unlink()
        else:
            raise RuntimeError(f"CarlaInterface::_load_map: Unknown map {map_name}")
        time.sleep(self._config.sleep_time)

    def _load_and_generate_odr(self, map_name: Path):
        """Loads OpenDRIVE map and generates 3D world out of it.

        :param map_name: Path to OpenDRIVE file.
        """
        with open(map_name, encoding="utf-8") as od_file:
            try:
                data = od_file.read()
            except OSError:
                self._config.logger.error("Failed load OpenDRIVE map: %s", map_name.stem)
                sys.exit()
        self._config.logger.info("Loaded OpenDRIVE map: %s successfully.", map_name.stem)
        self._client.generate_opendrive_world(
            data,
            carla.OpendriveGenerationParameters(
                vertex_distance=self._config.map_params.vertex_distance,
                max_road_length=self._config.map_params.max_road_length,
                wall_height=self._config.map_params.wall_height,
                additional_width=self._config.map_params.extra_width,
                smooth_junctions=True,
                enable_mesh_visibility=True,
                enable_pedestrian_navigation=True,
            ),
        )

    def _set_scenario(self, sc: Scenario):
        """
        Initializes obstacles and traffic lights from CommonRoad scenario.

        :param sc: CommonRoad scenario.
        """
        for obs in sc.obstacles:
            if obs.obstacle_role != ObstacleRole.DYNAMIC and (
                not ObstacleType.PARKED_VEHICLE and obs.obstacle_role != ObstacleRole.STATIC
            ):
                self._config.logger.error(
                    "CarlaInterface::_set_scenario: Only dynamic obstacles are supported. "
                    "Obstacle with ID %s will be skipped",
                    obs.obstacle_id,
                )
                continue
            if obs.obstacle_type in [
                ObstacleType.CAR,
                ObstacleType.BUS,
                ObstacleType.TAXI,
                ObstacleType.TRUCK,
                ObstacleType.MOTORCYCLE,
                ObstacleType.BICYCLE,
                ObstacleType.PARKED_VEHICLE,
            ]:
                self._cr_obstacles.append(VehicleInterface(obs, self._world, self._tm, config=self._config.vehicle))
            elif obs.obstacle_type == ObstacleType.PEDESTRIAN:
                self._cr_obstacles.append(
                    PedestrianInterface(obs, self._world, self._tm, config=self._config.pedestrian)
                )

        for tl in sc.lanelet_network.traffic_lights:
            closest_tl = find_closest_traffic_light(self.traffic_lights, tl.position)
            if closest_tl is not None:
                closest_tl.set_cr_light(tl)
            else:
                self._config.logger.error("traffic light could not be matched")

        self._set_cr_weather(sc.location.environment)

    def solution(
        self,
        planning_problem_id: int,
        vehicle_model: VehicleModel,
        vehicle_type: VehicleType,
        cost_function: CostFunction,
    ) -> PlanningProblemSolution:
        """
        Creates CommonRoad planning problem solution from driven ego vehicle trajectory.

        :param planning_problem_id: ID of new planning problem.
        :param vehicle_model: Vehicle model which should be used for planning problem.
        :param vehicle_type: Type of vehicle used for planning problem.
        :param cost_function: Cost function used for planning problem.
        :return: CommonRoad planning problem solution.
        """
        return PlanningProblemSolution(
            planning_problem_id=planning_problem_id,
            vehicle_model=vehicle_model,
            vehicle_type=vehicle_type,
            cost_function=cost_function,
            trajectory=Trajectory(self._ego.trajectory[0].time_step, self._ego.trajectory),
        )

    def cr_obstacles(self) -> List[Union[DynamicObstacle, StaticObstacle]]:
        """
        Extracts Commonroad obstacles.

        :return: List of CommonRoad obstacles containing driven trajectory from CARLA vehicles and walkers.
        """
        for obs in self._cr_obstacles:
            obs.cr_obstacle.prediction = TrajectoryPrediction(
                Trajectory(1, obs.trajectory), obs.cr_obstacle.obstacle_shape
            )
        return [obs.cr_obstacle for obs in self._cr_obstacles]

    def cr_ego_obstacle(self) -> DynamicObstacle:
        """
        Extracts Commonroad obstacle object for ego vehicle.

        :return: CommonRoad obstacle containing driven trajectory from CARLA ego vehicle.
        """
        self._ego.cr_obstacle.prediction = TrajectoryPrediction(
            Trajectory(1, self._ego.trajectory), self._ego.cr_obstacle.obstacle_shape
        )
        return self._ego.cr_obstacle

    def create_cr_map(self, initial_id: int = 1) -> Scenario:
        """
        Converts the CARLA map to a Commonroad map using CommonRoad Scenario Designer.

        :return: Scenario containing converted map without obstacles.
        """
        carla_map = self._world.get_map()
        odr_path = Path("temp.xodr")

        # Convert the CARLA map into OpenDRIVE in a temporary file
        with open(odr_path, "w", encoding="UTF-8") as file:
            file.write(carla_map.to_opendrive())

        # Load OpenDRIVE file, parse it, and convert it to a CommonRoad scenario
        config = open_drive_config
        config.initial_cr_id = initial_id
        config.proj_string_odr = None
        scenario = opendrive_to_commonroad(odr_path, odr_conf=config)

        # Delete temporary file
        odr_path.unlink()

        return scenario

    def replay(
        self,
        sc: Scenario,
        solution: Optional[Solution] = None,
        pps: Optional[PlanningProblemSet] = None,
        ego_id: Optional[int] = None,
    ):
        """
        Runs/Replays CommonRoad scenario in CARLA.

        :param sc: CommonRoad scenario.
        :param solution: CommonRoad solution which should be driven by ego vehicle.
        :param pps: Planning problem set corresponding to solution.
        :param ego_id: ID of ego vehicle in case an obstacle from the scenario should be used as ego vehicle.
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
            self._ego = VehicleInterface(ego_obs, self._world, self._tm, config=self._config.vehicle)
            # To spawn the carla actor.
            self._ego.tick(0)

        self._config.logger.info("Spawn other vehicles.")
        for obs in self._cr_obstacles:
            obs.tick(0)

        self._set_scenario(sc)

        self._run_simulation(obstacle_control=True, obstacle_only=obstacle_only)

    def _add_solution_to_scenario(self, ego_id: int, pps: PlanningProblemSet, solution: Solution) -> DynamicObstacle:
        """
        Creates CommonRoad dynamic obstacle given planning problem solution.

        :param ego_id: ID of new obstacle.
        :param pps: Planning problem required to extract trajectory.
        :param solution: Solution used to extract trajectory of new obstacle.
        """
        trajectory = solution_feasible(solution, self._config.simulation.time_step, pps)[ego_id][2]
        vehicle_params = VehicleParameterMapping.from_vehicle_type(solution.planning_problem_solutions[0].vehicle_type)

        shape = Rectangle(vehicle_params.l, vehicle_params.w)

        return DynamicObstacle(
            ego_id,
            ObstacleType.CAR,
            shape,
            pps.planning_problem_dict[ego_id].initial_state,
            TrajectoryPrediction(trajectory, shape),
        )

    def scenario_generation(self, sc: Scenario) -> Tuple[Scenario, PlanningProblemSet]:
        """
        Generates CommonRoad scenario given a map and the simulation config stored in the CARLA interface object.

        :param sc: Scenario containing map.
        :return: Generated CommonRoad scenario and planning problem set.
        """
        assert self._config.sync is True

        self._config.logger.info("Scenario generation: Create actors.")
        self._cr_obstacles = create_actors(
            self._client,
            self._world,
            self._tm,
            self._config.simulation,
            sc.generate_object_id,
            self._config.sync,
        )
        self._config.logger.info("Scenario generation: Start Simulation.")

        self._run_simulation(obstacle_only=True)

        for obs in self._cr_obstacles[1:]:
            obs.cr_obstacle.prediction = TrajectoryPrediction(
                Trajectory(1, obs.trajectory), obs.cr_obstacle.obstacle_shape
            )
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

        self._config.logger.info("Scenario generation finished.")

        return sc, PlanningProblemSet(
            [
                PlanningProblem(
                    sc.generate_object_id(),
                    self._cr_obstacles[0].cr_obstacle.initial_state,
                    goal_region,
                )
            ]
        )

    def external_control(
        self,
        controller_type: EgoPlanner,
        sc: Optional[Scenario] = None,
        pp: Optional[PlanningProblem] = None,
        vehicle_type: VehicleType = VehicleType.BMW_320i,
    ):
        """
        Executes keyboard control. Either a provided CommonRoad scenario with planning problem is used or
        a random vehicle from CARLA is used.

        :param controller_type: Type of external controller which should be used, e.g., keyboard, steering wheel, etc.
        :param sc: CommonRoad scenario.
        :param pp: CommonRoad planning problem.
        :param vehicle_type: CommonRoad vehicle type used for simulation.
        """
        self._config.logger.info("Start keyboard manual control.")

        if self._config.ego.ego_planner is not controller_type:
            self._config.ego.ego_planner = controller_type
            self._config.logger.info("Keyboard control type not set for ego! Will be set.")
        self._init_external_control_mode(None, None, pp, sc, vehicle_type)

    def _init_external_control_mode(
        self,
        planner: Optional[TrajectoryPlannerInterface],
        predictor: Optional[PredictorInterface],
        pp: Optional[PlanningProblem],
        sc: Optional[Scenario],
        vehicle_type: Optional[VehicleType],
    ) -> bool:
        """
        Initializes and start simulation.
        Ego vehicle is controlled by external input, e.g., steering wheel, keyboard, CommonRoad planner.

        :param planner: CommonRoad planner.
        :param predictor: CommonRoad predictor.
        :param pp: CommonRoad planning problem.
        :param sc: CommonRoad scenario.
        :param vehicle_type: CommonRoad vehicle type.
        """
        if pp is not None:
            self._pp = pp
            vehicle_params = VehicleParameterMapping.from_vehicle_type(vehicle_type)
            ego_obs = DynamicObstacle(
                0, ObstacleType.CAR, Rectangle(vehicle_params.l, vehicle_params.w), pp.initial_state
            )
        else:
            ego_obs = None
        self._config.logger.info("Init ego vehicle.")

        sc_new = sc if sc is not None else self.create_cr_map(1)
        self._ego = VehicleInterface(
            ego_obs,
            self._world,
            self._tm,
            planner=planner,
            predictor=predictor,
            sc=sc_new,
            pp=pp,
            config=self._config.ego,
        )
        if sc is not None:
            self._config.logger.info("Spawn CommonRoad obstacles.")
            self._set_scenario(sc)
            obstacle_control = True
        elif sc is None and pp is not None:
            self._cr_obstacles = create_actors(
                self._client,
                self._world,
                self._tm,
                self._config.simulation,
                sc_new.generate_object_id,
                self._config.sync,
                carla.Location(
                    x=self._ego.cr_obstacle.initial_state.position[0],
                    y=-self._ego.cr_obstacle.initial_state.position[1],
                ),
            )
            obstacle_control = False
        else:
            self._cr_obstacles = create_actors(
                self._client,
                self._world,
                self._tm,
                self._config.simulation,
                sc_new.generate_object_id,
                self._config.sync,
            )
            obstacle_control = False

        self._config.logger.info("Spawn ego.")
        self._ego.tick(0)
        if obstacle_control:
            self._config.logger.info("Spawn other vehicles.")
            for obs in self._cr_obstacles:
                obs.tick(0)

        return self._run_simulation(obstacle_control=obstacle_control)

    def plan(
        self,
        planner: TrajectoryPlannerInterface,
        predictor: Optional[PredictorInterface] = None,
        sc: Optional[Scenario] = None,
        pp: Optional[PlanningProblem] = None,
        vehicle_type: VehicleType = VehicleType.BMW_320i,
    ) -> bool:
        """
        Initializes and start simulation using CommonRoad-compatible planner.
        If no scenario or solution is provided, a CARLA map with randomly-generated traffic is used.
        If not planning problem set is provided, APIs from the CARLA agent library are used to navigate.

        :param planner: Trajectory planner which should be used.
        :param predictor: CommonRoad predictor.
        :param sc: CommonRoad scenario.
        :param pp: CommonRoad Planning problem.
        :param vehicle_type: CommonRoad vehicle type used for simulation.
        """
        self._config.logger.info("Start CommonRoad Planning.")
        if self._config.ego.ego_planner is not EgoPlanner.PLANNER:
            self._config.ego.ego_planner = EgoPlanner.PLANNER
            self._config.logger.info("CommonRoad Planner control type not set for ego! Will be set.")
        return self._init_external_control_mode(planner, predictor, pp, sc, vehicle_type)

    def _update_cr_state(self):
        """
        Stores CommonRoad obstacles and traffic lights states based on current world status.

        :param world: CARLA world object.
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
            time_step = (
                self._ego.cr_obstacle.initial_state.time_step + 1
                if len(self._ego.trajectory) == 0
                else self._ego.trajectory[-1].time_step + 1
            )
            if self._config.vehicle.vehicle_ks_state and self._ego.actor.is_alive:
                state = create_cr_ks_state_from_actor(self._ego.actor, time_step)
            elif self._ego.actor.is_alive:
                state = create_cr_pm_state_from_actor(self._ego.actor, time_step)
            self._ego.trajectory.append(state)

        for obs in self._cr_obstacles:
            # TODO Investigate the reason why certain actors are being destroyed prior to the completion of the loop.
            if obs.actor is None or not obs.actor.is_alive:
                self._config.logger.warning(
                    "Actor was destroyed before loop finished!: cr-id %s",
                    str(obs.cr_obstacle.obstacle_id),
                )
                continue

            time_step = (
                obs.cr_obstacle.initial_state.time_step + 1
                if len(obs.trajectory) == 0
                else obs.trajectory[-1].time_step + 1
            )

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

        :param env: CommonRoad environment storing time of day, underground and weather.
        """
        # TODO consider underground conditions
        if env is None:
            return
        if env.time_of_day is not TimeOfDay.NIGHT:
            if env.weather is Weather.HEAVY_RAIN:
                self._world.set_weather(carla.WeatherParameters.HardRainNoon)
            elif env.weather is Weather.LIGHT_RAIN:
                self._world.set_weather(carla.WeatherParameters.SoftRainNoon)
            elif env.weather is Weather.FOG:
                pass  # TODO set weather since fog in general supported by CARLA
            elif env.weather is Weather.HAIL:
                self._config.logger.info("CarlaInterface::set_cr_weather: Hail not supported by CARLA.")
            elif env.weather is Weather.SNOW:
                self._config.logger.info("CarlaInterface::set_cr_weather: Snow not supported by CARLA.")
            elif env.weather is Weather.CLEAR:
                self._world.set_weather(carla.WeatherParameters.ClearNoon)
            else:
                self._config.logger.info("CarlaInterface::set_cr_weather: Unsupported CommonRoad weather.")
                self._world.set_weather(carla.WeatherParameters.ClearNoon)
        else:
            if env.weather is Weather.HEAVY_RAIN:
                self._world.set_weather(carla.WeatherParameters.HardRainNight)
            elif env.weather is Weather.LIGHT_RAIN:
                self._world.set_weather(carla.WeatherParameters.SoftRainNight)
            elif env.weather is Weather.FOG:
                pass  # TODO set weather since fog in general supported by CARLA
            elif env.weather is Weather.HAIL:
                self._config.logger.info("CarlaInterface::set_cr_weather: Hail not supported by CARLA.")
            elif env.weather is Weather.SNOW:
                self._config.logger.info("CarlaInterface::set_cr_weather: Snow not supported by CARLA.")
            elif env.weather is Weather.CLEAR:
                self._world.set_weather(carla.WeatherParameters.ClearNight)
            else:
                self._config.logger.info("CarlaInterface::set_cr_weather: Unsupported CommonRoad weather.")
                self._world.set_weather(carla.WeatherParameters.ClearNight)

    def set_weather(self, config: WeatherParams = WeatherParams()):
        """
        Sets weather based on given config.

        :param config: Weather config parameters.
        """
        self._world.set_weather(
            carla.WeatherParameters(
                config.cloudiness,
                config.precipitation,
                config.precipitation_deposits,
                config.wind_intensity,
                config.sun_azimuth_angle,
                config.sun_altitude_angle,
                config.fog_density,
                config.fog_distance,
                config.wetness,
                config.fog_falloff,
                config.scattering_intensity,
                config.mie_scattering_scale,
                config.rayleigh_scattering_scale,
            )
        )

    def _init_visualization(self, obstacle_only: bool):
        """
        Initializes the visualization world, clock,
        and display based on the visualization type specified in the configuration.

        :param obstacle_only: Boolean indicating whether only obstacles should be simulated, i.e. no ego vehicle.
        :return: A tuple containing the visualization world, clock, and display.
        """
        vis_world = None
        clock = None
        display = None

        if self._config.vis_type is not CustomVis.NONE and not obstacle_only:
            display = self._init_display()
            clock = pygame.time.Clock()

        if self._config.vis_type is CustomVis.BIRD and not obstacle_only:
            self._config.logger.info("Init 2D.")
            hud = HUD2D("CARLA 2D", self._config.visualization.width, self._config.visualization.height)
            vis_world = World2D("CARLA 2D", self._world, hud, self._ego.actor, self._config.birds_eye_view)
        elif (
            self._config.vis_type is CustomVis.THIRD_PERSON or self._config.vis_type is CustomVis.DRIVER
        ) and not obstacle_only:
            self._config.logger.info("Init 3D.")
            vis_world = Visualization3D(self._world, self._config, self._ego.actor)

        return vis_world, clock, display

    def create_cr_scenario(self, base_scenario: Optional[Scenario] = None, reuse_created_map: bool = True) -> Scenario:
        """
        Creates a CommonRoad scenario based on the recorded traffic given a base scenario, e.g., empty map.
        If no map is provided, current OpenDRIVE map will be used.

        :param base_scenario: CommonRoad scenario containing at least a map.
        :param reuse_created_map: Boolean indicating whether stored map should be used for scenario.
        :return CommonRoad scenario.
        """
        obstacles = self.cr_obstacles() + [self.cr_ego_obstacle()]
        if reuse_created_map and self._ego.get_scenario() is not None:
            base_scenario = self._ego.get_scenario()
        if base_scenario is None:
            max_id = max([obs.obstacle_id for obs in obstacles])
            base_scenario = self.create_cr_map(max_id + 1)
        base_scenario.add_objects(obstacles)
        return base_scenario

    def _run_simulation(self, obstacle_control: bool = False, obstacle_only: bool = False) -> bool:
        """
        Performs simulation by iteratively calling tick and render functions.
        Initializes visualization worlds and head-up display.

        :param obstacle_control: Boolean indicating whether obstacles are controlled based on CommonRoad scenario.
        :param obstacle_only: Boolean indicating whether only obstacles should be simulated, i.e. no ego vehicle.
        """
        # add cameras to create custom pictures, e.g., for papers
        # (needs to be done here since map might change after init)
        camera_actors = []
        if self._config.visualization.camera_storage_path != "" and Path.exists(
            Path(self._config.visualization.camera_storage_path)
        ):
            camera_actors = init_camera_sensor(
                self._world,
                self._config.visualization.camera_transform_horizontal,
                self._config.visualization.camera_transform_bird,
                self._config.visualization.camera_storage_path,
            )

        # set SDL to use the dummy NULL video driver,
        # so it doesn't need a windowing system.
        # (to run pygame without a display)
        if self._config.simulation.ignore_video_driver:
            os.environ["SDL_VIDEODRIVER"] = "dummy"

        vis_world, clock, display = self._init_visualization(obstacle_only)

        time_step = 0
        goal_reached = False

        self._config.logger.info("Start simulation.")
        while time_step <= self._config.simulation.max_time_step:
            position = self._ego.actor.get_location()
            vel = self._ego.actor.get_velocity()
            if self._config.sync:
                self._world.tick()
            else:
                self._world.wait_for_tick()

            if self._ego is not None:
                position2 = self._ego.actor.get_location()
                self._ego.tick(time_step)
                position3 = self._ego.actor.get_location()
            if obstacle_control:
                obstacle_errors = []
                for obs in self._cr_obstacles:
                    try:
                        obs.tick(time_step)
                    except Exception:
                        obstacle_errors.append(obs)
                for obs in obstacle_errors:  # remove obstacle when an error occurs
                    self._cr_obstacles.remove(obs)
                for tl in self.traffic_lights:
                    tl.tick(time_step)

            if self._config.vis_type is not CustomVis.NONE and not obstacle_only:
                clock.tick_busy_loop(1 / self._config.simulation.time_step)
                vis_world.tick(clock)
                vis_world.render(display)
                pygame.display.flip()

            time_step += 1
            self._update_cr_state()

            if self._pp is not None and self._pp.goal.is_reached(self._ego.trajectory[-1]):
                print("CommonRoad goal reached!")
                goal_reached = True
                break

            if time_step > self._config.simulation.max_time_step:
                print("Simulation time limit reached!")
                break

        print("Simulation finished.")

        if vis_world is not None:
            vis_world.destroy()

        if self._config.ego_view.record_video:
            make_video(self._config.ego_view.video_path, self._config.ego_view.video_name)

        for camera_actor in camera_actors:
            if camera_actor.is_alive:
                camera_actor.destroy()

        for actor in self._world.get_actors():
            if "walker" in actor.type_id or "vehicle" in actor.type_id:
                actor.destroy()

        return goal_reached

    def _init_display(self) -> pygame.display:
        """
        Initializes Pygame display for 3D ego view.

        :return: Initialized pygame display object.
        """
        pygame.init()
        pygame.font.init()
        display = pygame.display.set_mode(
            (self._config.visualization.width, self._config.visualization.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF,
        )
        pygame.display.set_caption(self._config.visualization.description)  # Place a title to game window
        # Show loading screen
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        text_surface = font.render("Rendering map...", True, pygame.Color(255, 255, 255))
        display.blit(
            text_surface,
            text_surface.get_rect(center=(self._config.visualization.width / 2, self._config.visualization.height / 2)),
        )
        display.fill((0, 0, 0))
        pygame.display.flip()
        return display
