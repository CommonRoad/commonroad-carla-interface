from typing import Optional
import numpy as np
from abc import ABC

from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblemSet


class TrajectoryPlannerInterface(ABC):
    """Base class for trajectory planner."""

    def plan(self, sc: Scenario, pps: PlanningProblemSet, ref_path: Optional[np.ndarray] = None) -> Trajectory:
        """
        Interface method for planning.

        :param sc: CommonRoad scenario.
        :param pps: CommonRoad planning problem set.
        :param ref_path: Reference path which the trajectory planner should follow.
        :return: CommonRoad trajectory.
        """
