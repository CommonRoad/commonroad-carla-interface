import numpy as np
from typing import Optional

from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad.planning.planning_problem import PlanningProblem
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.config import ReactivePlannerConfiguration

from carlacr.helper.planner import TrajectoryPlannerInterface


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
        self._planner.reset(self._config)
        return self._planner.plan()[0]
