from copy import deepcopy
from enum import Enum
from typing import Optional, List

import numpy as np
from commonroad.planning.planner_interface import TrajectoryPlannerInterface
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import KSState
from commonroad.scenario.trajectory import Trajectory

from crcarla.controller.automaton_planner.auxiliary.overlappingLanelets import overlappingLanelets
from crcarla.controller.automaton_planner.auxiliary.prediction import prediction
from crcarla.controller.automaton_planner.maneuverAutomaton.createManeuverAutomaton import create_ma
from crcarla.controller.automaton_planner.src.highLevelPlanner import highLevelPlanner
from crcarla.controller.automaton_planner.src.lowLevelPlannerManeuverAutomaton import lowLevelPlannerManeuverAutomaton
from crcarla.controller.automaton_planner.src.lowLevelPlannerOptimization import lowLevelPlannerOptimization


class PlannerType(Enum):
    AUTOMATON = 1
    OPTIMIZATION = 2

class AutomatonPlannerInterface(TrajectoryPlannerInterface):
    def __init__(self, scenario: Scenario, params, type: PlannerType = PlannerType.AUTOMATON, real_dynamics: bool = False):
        self._params = params
        self._type = type
        self._real_dynamics = real_dynamics
        self._horizon = int(np.round(params['horizon']/scenario.dt))


    def plan(self,
             sc: Scenario,
             pp: PlanningProblem,
             ref_path: Optional[np.ndarray] = None,
             steering_angle: float = 0.0
             ):
        scenario_ = deepcopy(sc)
        x0 = np.concatenate((pp.initial_state.position, np.array([pp.initial_state.velocity, pp.initial_state.orientation])))
        overlaps = overlappingLanelets(sc)
        scenario_: Scenario = prediction(scenario_, self._horizon, x0, overlapping_lanelets=overlaps)

        if self._type == PlannerType.AUTOMATON:
            plan, space, ref_traj = highLevelPlanner(scenario_, pp, self._params, compute_free_space=True,
                                                     minimum_safe_distance=0.5, improve_velocity_profile=True)
            x, u, controller = lowLevelPlannerManeuverAutomaton(scenario_, pp, self._params, space, ref_traj,
                                                                create_ma(self._params))
        else:
            plan, space, ref_traj = highLevelPlanner(scenario_, pp, self._params, compute_free_space=False,
                                                     minimum_safe_distance=0.5, improve_velocity_profile=True)
            x, u, controller = lowLevelPlannerOptimization(scenario_, pp, self._params, space, ref_traj, feedback_control=True,
                                                           R_diff=np.diag([0, 0.1]))

        state_list: List[KSState] = self.create_state_list(x, u, pp.initial_state.time_step)

        traj = Trajectory(pp.initial_state.time_step, state_list)

        return traj

    def create_state_list(self, x, u, init_ts) -> List[KSState]:
        state_list: List[KSState] = []
        for i in range(len(x)):
            pos = np.array([x[0][i], x[1][i]])
            steering_angle = u[1][i]
            velocity = x[2][i]
            orientation = x[3][i]
            state = KSState(i + init_ts, pos, steering_angle, velocity, orientation)
            state_list.append(state)
        return state_list