from typing import Optional, Union
import logging
import numpy as np
import carla
from commonroad.scenario.state import TraceState
from commonroad.scenario.scenario import Scenario, ScenarioID
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.state import InitialState, State
from commonroad.planning.goal import GoalRegion

from carlacr.controller.controller import CarlaController
from carlacr.helper.config import VehicleControlType, ControlParams
from carlacr.controller.controller import TransformControl
from carlacr.controller.vehicle_controller import PIDController, AckermannController, \
    VehicleTMPathFollowingControl, VehicleBehaviorAgentPathFollowingControl
from carlacr.helper.planner import TrajectoryPlannerInterface

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def create_scenario_from_world(world: carla.World) -> Scenario:
    """
    Creates scenario without prediction from CARLA world.

    :param world: CARLA world.
    :return: CommonRoad scenario.
    """
    print(world.id)
    return Scenario(0.1, ScenarioID(), "")


def get_planning_problem_from_world(world: carla.World) -> PlanningProblem:
    """
    Creates planning problem from CARLA world.

    :param world: CARLA world.
    :return: CommonRoad planning problem.
    """
    print(world.id)
    return PlanningProblem(0, InitialState(0, np.array([0, 0]), 0, 0, 0), GoalRegion([State(60)]))


class CommonRoadPlannerController(CarlaController):
    """Controller which uses trajectory generated by CommonRoad planner as input."""

    def __init__(self, actor: carla.Actor, planner: TrajectoryPlannerInterface, pp: PlanningProblem,
                 control_type: VehicleControlType, dt: float, control_config: ControlParams):
        """
        Initialization of CommonRoad planner controller.

        :param actor: CARLA actor.
        :param planner: CommonRoad planner.
        :param pp: CommonRoad planning problem.
        :param control_type: CARLA control type used for CommonRoad planner.
        :param dt: Time step size.
        :param control_config: CARLA controller params.
        """
        super().__init__(actor)
        self._planner = planner
        self._pp = pp
        self._current_trajectory = None
        self._controller = self._create_controller(control_type, dt, control_config)

    def _create_controller(self, control_type: VehicleControlType, dt: float, control_config: ControlParams) \
            -> Union[TransformControl, PIDController, AckermannController, VehicleBehaviorAgentPathFollowingControl,
                     VehicleTMPathFollowingControl]:
        """
        Creates CARLA controller object.

        :param control_type: CARLA control type used for CommonRoad planner.
        :param dt: Time step size.
        :param control_config: CARLA controller params.
        :return: CARLA controller.
        """
        if control_type is VehicleControlType.TRANSFORM:
            return TransformControl(self._actor)
        if control_type is VehicleControlType.PID:
            return PIDController(actor=self._actor, config=control_config, dt=dt)
        if control_type is VehicleControlType.ACKERMANN:
            return AckermannController(self._actor, config=control_config)
        if control_type is VehicleControlType.PATH_TM:
            return VehicleTMPathFollowingControl(self._actor)
        if control_type is VehicleControlType.PATH_AGENT:
            return VehicleBehaviorAgentPathFollowingControl(self._actor)
        logger.error("CommonRoadPlannerController::_create_controller: Unknown controller type.")
        return TransformControl(self._actor)

    def control(self, state: Optional[TraceState] = None):
        """
        Computes and applies CARLA steering wheel control.

        :param state: State which should be reached at next time step.
        """
        world = self._actor.get_world()
        sc = create_scenario_from_world(world)
        if self._pp is None:
            pp = get_planning_problem_from_world(world)
        else:
            pp = self._pp
        self._current_trajectory = self._planner.plan(sc, pp)
        self._controller.control(self._current_trajectory.state_list[0])
