from copy import deepcopy
import numpy as np
from dataclasses import dataclass, field
from typing import Optional


from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_rp.reactive_planner import ReactivePlanner

from carlacr.helper.planner import TrajectoryPlannerInterface


@dataclass
class PlanningParams:
    """Planning parameters for reactive planner."""

    # planner time step (in s)
    dt: float = 0.1
    # time_steps_computation * dt = horizon. e.g. 20 * 0.1 = 2s
    time_steps_computation: int = 60
    planning_horizon: float = dt * time_steps_computation
    # replanning frequency (in time steps)
    replanning_frequency: int = 3
    # continuous collision checking
    continuous_cc: bool = False
    # collision check in curvilinear coordinates
    collision_check_in_cl: bool = False
    # time scaling factor for collision checking if planner time step and scenario time step deviate
    factor: int = 1
    # velocity threshold (in m/s) for switching to low velocity mode
    low_vel_mode_threshold: int = 4


@dataclass
class SamplingParams:
    """Sampling parameters for reactive planner."""

    # minimum time sampling in s (t_max is given by planning horizon)
    t_min: float = 0.4
    # velocity sampling interval
    v_min: float = 0
    v_max: float = 0
    # lateral sampling interval
    d_min: float = -3
    d_max: float = 3


@dataclass
class ReactiveParams:
    """Configuration parameters for reactive planner."""

    planning: PlanningParams = field(default_factory=PlanningParams)
    sampling: SamplingParams = field(default_factory=SamplingParams)


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

    def plan(self, sc: Scenario, pps: PlanningProblemSet, ref_path: Optional[np.ndarray] = None) -> Trajectory:
        """
        Performs trajectory planning of reactive planner.

        :param sc: CommonRoad scenario.
        :param pps: CommonRoad planning problem set.
        :param ref_path: Reference path which the trajectory planner should follow.
        :return: CommonRoad trajectory.
        """
        planning_problem = list(pps.planning_problem_dict.values())[0]

        # initial state configuration
        problem_init_state = planning_problem.initial_state
        current_velocity = problem_init_state.velocity
        if not hasattr(problem_init_state, 'acceleration'):
            problem_init_state.acceleration = 0.
        x_0 = deepcopy(problem_init_state)
        delattr(x_0, "slip_angle")

        if hasattr(planning_problem.goal.state_list[0], 'velocity'):
            if planning_problem.goal.state_list[0].velocity.start > 0:
                desired_velocity = (planning_problem.goal.state_list[0].velocity.start +
                                    planning_problem.goal.state_list[0].velocity.end) / 2
            else:
                desired_velocity = planning_problem.goal.state_list[0].velocity.end / 2
        else:
            desired_velocity = x_0.velocity

        # set collision checker
        self._planner.set_collision_checker(sc)
        # initialize route planner and set reference path
        if ref_path is None:
            route_planner = RoutePlanner(sc, planning_problem)
            route = route_planner.plan_routes().retrieve_first_route()
            ref_path = route.reference_path

        self._planner.set_desired_velocity(desired_velocity, current_velocity)
        self._planner.set_reference_path(ref_path)
        x_0 = self._planner.process_initial_state_from_pp(x0_pp=x_0)

        return self._planner.plan(x_0)[0]
