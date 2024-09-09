import copy
import logging
from typing import Optional

import numpy as np
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.writer.file_writer_interface import OverwriteExistingFile
from commonroad.planning.planner_interface import TrajectoryPlannerInterface
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import (
    create_collision_object,
)
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.state import ReactivePlannerState
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_rp.utility.visualization import visualize_planner_at_timestep

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class ReactivePlannerInterface(TrajectoryPlannerInterface):
    """CARLA-Interface for reactive planner."""

    def __init__(
        self,
        sc: Scenario,
        pp: PlanningProblem,
        config: ReactivePlannerConfiguration = ReactivePlannerConfiguration(),
        store_failing_scenarios: bool = False,
    ):
        """
        Initialization for reactive planner interface.

        :param config: Reactive planner configuration parameters.
        """
        self._config = config
        self._config.scenario = sc
        self._config.planning_problem = pp
        route_planner = RoutePlanner(config.scenario, config.planning_problem)
        route = route_planner.plan_routes().retrieve_first_route()
        self._config.planning.route = route
        self._config.planning.reference_path = route.reference_path
        self._planner = ReactivePlanner(config)
        self._optimal = None
        self._error_counter = 0
        self._store_failing_scenarios = store_failing_scenarios
        self._reference_velocity = 15  # 15  # TODO
        tmp_sc = copy.deepcopy(sc)
        for obs in tmp_sc.obstacles:
            tmp_sc.remove_obstacle(obs)
        self._planner.set_collision_checker(sc)
        self._cc = self._planner.collision_checker

    def plan(
        self,
        sc: Scenario,
        pp: PlanningProblem,
        ref_path: Optional[np.ndarray] = None,
        steering_angle: float = 0.0,
    ) -> Trajectory:
        """
        Performs trajectory planning of reactive planner.

        :param sc: CommonRoad scenario.
        :param pp: CommonRoad planning problem.
        :param ref_path: Reference path which the trajectory planner should follow.
        :return: CommonRoad trajectory.
        """
        self._config.scenario = sc
        self._config.planning_problem = pp

        # set reference velocity for planner
        self._planner.set_desired_velocity(
            desired_velocity=self._reference_velocity, current_speed=pp.initial_state.velocity
        )

        # self._planner.set_collision_checker(sc)
        cc_scenario = copy.deepcopy(self._cc)
        for co in sc.static_obstacles:
            obs = create_collision_object(co)
            cc_scenario.add_collision_object(obs)
        for co in sc.dynamic_obstacles:
            tvo = create_collision_object(co)
            cc_scenario.add_collision_object(tvo)
        self._planner.set_collision_checker(None, cc_scenario)

        x0_planner_cart = ReactivePlannerState()
        x0_planner_cart = pp.initial_state.convert_state_to_state(x0_planner_cart)
        x0_planner_cart.steering_angle = steering_angle
        self._planner.reset(
            initial_state_cart=x0_planner_cart,
            initial_state_curv=None,
            collision_checker=self._planner.collision_checker,
            coordinate_system=self._planner.coordinate_system,
        )

        try:
            # call plan function and generate trajectory
            optimal_traj = self._planner.plan()

            # check if valid trajectory is found
            if optimal_traj:
                # add to planned trajectory
                self._cr_state_list = optimal_traj[0].state_list

                # record planned state and input TODO check this
                self._planner.record_state_and_input(optimal_traj[0].state_list[1])
            else:
                # TODO: sample emergency brake trajectory if no trajectory is found
                self._cr_state_list = None

            self._optimal = self._planner.plan()[0]
            self._error_counter = 0
            # visualize the current time step of the simulation
            if self._config.debug.save_plots:
                ego_vehicle = self._planner.convert_state_list_to_commonroad_object(self._optimal.state_list)
                sampled_trajectory_bundle = None
                if self._config.debug.draw_traj_set:
                    sampled_trajectory_bundle = copy.deepcopy(self._planner.stored_trajectories)

                visualize_planner_at_timestep(
                    scenario=self._config.scenario,
                    planning_problem=self._config.planning_problem,
                    ego=ego_vehicle,
                    traj_set=sampled_trajectory_bundle,
                    ref_path=self._planner.reference_path,
                    timestep=pp.initial_state.time_step,
                    config=self._config,
                )  # , plot_limits=[250.0, 460, -160, -130])

            return self._optimal
        except (AssertionError, TypeError):
            if self._store_failing_scenarios:
                logger.error(
                    "ReactivePlannerInterface::plan AssertionError: Scenario and " "Planning Problem will be stored."
                )
                fw = CommonRoadFileWriter(sc, PlanningProblemSet([pp]))
                sc_map_name_tmp = sc.scenario_id.map_name
                sc.scenario_id.map_name += "ReactivePlannerError"
                configuration_id_tmp = sc.scenario_id.configuration_id
                sc.scenario_id.configuration_id = self._error_counter + 1
                fw.write_to_file(f"{sc.scenario_id}.xml", OverwriteExistingFile.ALWAYS)
                sc.scenario_id.map_name = sc_map_name_tmp
                sc.scenario_id.configuration_id = configuration_id_tmp
            # if no optimal trajectory can be computed use last computed trajectory
            self._error_counter += 1
            traj = Trajectory(
                self._optimal.initial_time_step + self._error_counter,
                self._optimal.state_list[self._error_counter : :],
            )
            return traj
