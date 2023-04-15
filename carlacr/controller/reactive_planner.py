from copy import deepcopy
import numpy as np
from typing import Optional

from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad.planning.planning_problem import PlanningProblem
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.config import ReactiveParams

from carlacr.helper.planner import TrajectoryPlannerInterface


class ReactivePlannerInterface(TrajectoryPlannerInterface):
    """CARLA-Interface for reactive planner."""

    def __init__(self, config: ReactiveParams = ReactiveParams()):
        """
        Initialization for reactive planner interface.

        :param config: Reactive planner configuration parameters.
        """
        self._config = config
        self._planner = ReactivePlanner(config)
        self._planner.set_d_sampling_parameters(config.sampling.d_min, config.sampling.d_max)
        self._planner.set_t_sampling_parameters(config.sampling.t_min, config.planning.dt,
                                                config.planning.planning_horizon)

    def plan(self, sc: Scenario, pp: PlanningProblem, ref_path: Optional[np.ndarray] = None) -> Trajectory:
        """
        Performs trajectory planning of reactive planner.

        :param sc: CommonRoad scenario.
        :param pp: CommonRoad planning problem.
        :param ref_path: Reference path which the trajectory planner should follow.
        :return: CommonRoad trajectory.
        """
        # initial state configuration
        problem_init_state = pp.initial_state
        current_velocity = problem_init_state.velocity
        if not hasattr(problem_init_state, 'acceleration'):
            problem_init_state.acceleration = 0.
        x_0 = deepcopy(problem_init_state)
        delattr(x_0, "slip_angle")

        if hasattr(pp.goal.state_list[0], 'velocity'):
            if pp.goal.state_list[0].velocity.start > 0:
                desired_velocity = (pp.goal.state_list[0].velocity.start +
                                    pp.goal.state_list[0].velocity.end) / 2
            else:
                desired_velocity = pp.goal.state_list[0].velocity.end / 2
        else:
            desired_velocity = x_0.velocity

        # set collision checker
        self._planner.set_collision_checker(sc)
        # initialize route planner and set reference path
        if ref_path is None:
            route_planner = RoutePlanner(sc, pp)
            route = route_planner.plan_routes().retrieve_first_route()
            ref_path = route.reference_path

        self._planner.set_desired_velocity(desired_velocity, current_velocity)
        self._planner.set_reference_path(ref_path)
        x_0 = self._planner.process_initial_state_from_pp(x0_pp=x_0)

        return self._planner.plan(x_0)[0]
