import copy
import logging
from typing import Optional

import matplotlib
import numpy as np
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.writer.file_writer_interface import OverwriteExistingFile
from commonroad.planning.planner_interface import TrajectoryPlannerInterface
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import KSState
from commonroad.scenario.trajectory import Trajectory
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import (
    create_collision_object,
)
import commonroad_route_planner.fast_api.fast_api as rfapi
from commonroad_route_planner.reference_path import ReferencePath
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.state import ReactivePlannerState
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_rp.utility.visualization import visualize_planner_at_timestep
from matplotlib import pyplot as plt

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class SpeedTrackingController(TrajectoryPlannerInterface):
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
        route: ReferencePath = rfapi.generate_reference_path_from_lanelet_network_and_planning_problem(
            lanelet_network=config.scenario.lanelet_network, planning_problem=config.planning_problem
        )
        self._config.planning.route = route
        self._config.planning.reference_path = route.reference_path
        self._planner = ReactivePlanner(config)
        self._planner.set_reference_path(route.reference_path)
        self._optimal = None
        self._error_counter = 0
        self._store_failing_scenarios = store_failing_scenarios
        self._reference_velocity = 15  # TODO use velocity-planner
        tmp_sc = copy.deepcopy(sc)
        for obs in tmp_sc.obstacles:
            tmp_sc.remove_obstacle(obs)
        self._planner.set_collision_checker(sc)
        self._cc = self._planner.collision_checker
        self._wb_rear_axle = self._planner.config.vehicle.wb_rear_axle
        self._des_vel = [pp.initial_state.velocity]
        self._act_vel = []

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
        :param steering_angle: Steering angle in rad.
        :return: CommonRoad trajectory.
        """
        self._config.scenario = sc
        self._config.planning_problem = pp

        if (len(self._act_vel) == 50):
            print(sum(abs(a - b) for a, b in zip(self._act_vel, self._des_vel)))
            return None


        next_state = KSState(pp.initial_state.time_step, np.array([0.0, 0.0]), 0.0, 2.0, 0.0)
        state_list = [next_state, next_state]
        traj = Trajectory(pp.initial_state.time_step, state_list)


        return traj

    def convert_from_rear_to_middle(self, traj: Trajectory) -> Trajectory:
        """
        Converts a trajectories rear positions of a car to the middle positions of the car, based on its orientation.

        :param Trajectory traj: Trajectory to be converted.
        """
        shifted_traj = []
        for state in traj.state_list:
            shifted_traj.append(state.shift_positions_to_center(self._wb_rear_axle))
        return Trajectory(traj.initial_time_step, shifted_traj)
