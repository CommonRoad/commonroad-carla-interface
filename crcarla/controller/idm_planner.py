import logging
from typing import Optional

import numpy as np
from commonroad.planning.planner_interface import TrajectoryPlannerInterface
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad_idm_planner.configuration.planner_config import IDMConfig
from commonroad_idm_planner.idm_path import IDMPath, IDMPathFactory
from commonroad_idm_planner.idm_planner import IDMPlanner
from commonroad_idm_planner.idm_trajectory import IDMTrajectory
from commonroad_idm_planner.util.collision.collision_status import CollisionStatus
from commonroad_route_planner.fast_api.fast_api import generate_reference_path_from_lanelet_network_and_planning_problem
from commonroad_route_planner.reference_path import ReferencePath

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class IDMPlannerInterface(TrajectoryPlannerInterface):
    """CARLA-Interface for idm planner."""

    def __init__(self, sc: Scenario, pp: PlanningProblem, config: IDMConfig):
        """
        Initialization for reactive planner interface.

        :param config: Reactive planner configuration parameters.
        """
        reference_path: ReferencePath = generate_reference_path_from_lanelet_network_and_planning_problem(
            lanelet_network=sc.lanelet_network, planning_problem=pp
        )

        idm_path: IDMPath = IDMPathFactory().generate_idm_path_from_cr_route_planner_reference_path(
            cr_reference_path=reference_path
        )

        self._planner = IDMPlanner(scenario=sc, planning_problem=pp, config=config, idm_path=idm_path)

    def plan(
        self,
        sc: Scenario,
        pp: PlanningProblem,
        ref_path: Optional[np.ndarray] = None,
        steering_angle: float = 0.0,
    ) -> IDMTrajectory | None:
        """
        Performs trajectory planning of reactive planner.

        :param sc: CommonRoad scenario.
        :param pp: CommonRoad planning problem.
        :param ref_path: Reference path which the trajectory planner should follow.
        :param steering_angle: Steering angle in rad.
        :return: CommonRoad trajectory.
        """
        new_rp: ReferencePath = generate_reference_path_from_lanelet_network_and_planning_problem(
            lanelet_network=sc.lanelet_network, planning_problem=pp
        )

        new_idm_path: IDMPath = IDMPathFactory().generate_idm_path_from_cr_route_planner_reference_path(
            cr_reference_path=new_rp
        )

        trajectory_one_cycle: IDMTrajectory = self._planner.re_plan(planning_problem=pp, idm_path=new_idm_path)

        collision_status: CollisionStatus = self._planner.check_collision()

        if collision_status.collision_detected:
            self._planner.logger.info(f"{collision_status}")

        return trajectory_one_cycle
