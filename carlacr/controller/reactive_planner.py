import copy
import logging
import sys

import numpy as np
from typing import Optional

from commonroad.common.writer.file_writer_interface import OverwriteExistingFile
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_rp.state import ReactivePlannerState
from commonroad_rp.utility.visualization import visualize_planner_at_timestep

from carlacr.helper.planner import TrajectoryPlannerInterface

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class ReactivePlannerInterface(TrajectoryPlannerInterface):
    """CARLA-Interface for reactive planner."""

    def __init__(self, sc: Scenario, pp: PlanningProblem,
                 config: ReactivePlannerConfiguration = ReactivePlannerConfiguration()):
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

    def plan(self, sc: Scenario, pp: PlanningProblem, ref_path: Optional[np.ndarray] = None) -> Trajectory:
        """
        Performs trajectory planning of reactive planner.

        :param sc: CommonRoad scenario.
        :param pp: CommonRoad planning problem.
        :param ref_path: Reference path which the trajectory planner should follow.
        :return: CommonRoad trajectory.
        """
        self._config.scenario = sc
        self._config.planning_problem = pp
        self._planner.reset(self._config,
                            ReactivePlannerState(position=pp.initial_state.position, velocity=pp.initial_state.velocity,
                                                 orientation=pp.initial_state.orientation,
                                                 acceleration=pp.initial_state.acceleration,
                                                 yaw_rate=pp.initial_state.yaw_rate,
                                                 time_step=pp.initial_state.time_step, steering_angle=0))
        try:
            optimal = self._planner.plan()[0]
            # visualize the current time step of the simulation
            if self._config.debug.save_plots:
                ego_vehicle = self._planner.convert_state_list_to_commonroad_object(optimal.state_list)
                sampled_trajectory_bundle = None
                if self._config.debug.draw_traj_set:
                    sampled_trajectory_bundle = copy.deepcopy(self._planner.stored_trajectories)

                visualize_planner_at_timestep(scenario=self._config.scenario,
                                              planning_problem=self._config.planning_problem,
                                              ego=ego_vehicle,
                                              traj_set=sampled_trajectory_bundle,
                                              ref_path=self._planner.reference_path,
                                              timestep=pp.initial_state.time_step,
                                              config=self._config)

            return optimal
        except AssertionError:
            logger.error("ReactivePlannerInterface::plan AssertionError: Scenario and "
                         "Planning Problem will be stored.")
            fw = CommonRoadFileWriter(sc, PlanningProblemSet([pp]))
            sc.scenario_id.map_name += "ReactivePlannerError"
            fw.write_to_file(f"{sc.scenario_id}.xml", OverwriteExistingFile.ALWAYS)
            sys.exit()
