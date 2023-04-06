import copy
from dataclasses import dataclass
from typing import Optional, Union
import logging
import carla
import numpy as np

from commonroad.scenario.state import TraceState
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.state import CustomState
from commonroad.planning.goal import GoalRegion, Interval
from crpred.predictor_interface import PredictorInterface
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_rp.utility.config import VehicleParams
from commonroad_dc.geometry.geometry import compute_pathlength_from_polyline, compute_orientation_from_polyline
from commonroad.geometry.shape import Rectangle

from carlacr.controller.controller import CarlaController
from carlacr.helper.config import VehicleControlType, ControlParams
from carlacr.controller.controller import TransformControl
from carlacr.controller.vehicle_controller import PIDController, AckermannController, \
    VehicleTMPathFollowingControl, VehicleBehaviorAgentPathFollowingControl
from carlacr.helper.planner import TrajectoryPlannerInterface
from carlacr.helper.utils import create_cr_vehicle_from_actor, create_cr_initial_state_from_actor

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


@dataclass
class RouteData:
    """Representation for route data."""

    route: Optional[np.ndarray] = None
    path_length: Optional[np.ndarray] = None
    orientation: Optional[np.ndarray] = None

    def __post_init__(self):
        """Initialization of path length and orientation."""
        if self.route is not None and self.path_length is None:
            self.path_length = compute_pathlength_from_polyline(self.route)
        if self.route is not None and self.orientation is None:
            self.orientation = compute_orientation_from_polyline(self.route)


def create_scenario_from_world(world: carla.World, sc: Scenario) -> Scenario:
    """
    Creates scenario without prediction from CARLA world.

    :param world: CARLA world.
    :param sc: Base scenario containing road network and static obstacles.
    :return: CommonRoad scenario.
    """
    for actor in world.get_actors():
        sc.add_objects(create_cr_vehicle_from_actor(actor, sc.generate_object_id()))
    return sc


def get_planning_problem_from_world(actor: carla.Actor, vehicle_params: VehicleParams,
                                    t_h: float, dt: float, global_route: RouteData) -> PlanningProblem:
    """
    Creates planning problem from global route.

    :param actor: Ego vehicle actor.
    :param vehicle_params: Ego vehicle parameters.
    :param t_h: Trajectory time horizon [s].
    :param dt: Time step size [s].
    :param global_route: Global route.
    :return: CommonRoad planning problem.
    """
    initial_state = create_cr_initial_state_from_actor(actor, 0)
    min_dist = max(0, initial_state.velocity * t_h + 0.5 * vehicle_params.a_max * 6 ** 2)
    max_dist = initial_state.velocity * t_h + 0.5 * vehicle_params.a_max * 6 ** 2

    init_idx = max(0, (np.abs(global_route.route - initial_state.position)).argmin() - 1)
    distance_min = global_route.path_length[init_idx] + min_dist
    distance_max = global_route.path_length[init_idx] + max_dist

    idx_min = max(np.searchsorted(global_route.path_length, distance_min) - 1, 0)
    idx_max = min(np.searchsorted(global_route.path_length, distance_max) + 1, len(global_route.route) - 1)
    position = 0.5 * (global_route.route[idx_min] + global_route.route[idx_max])
    length = global_route.path_length[idx_max] - global_route.path_length[idx_min]
    orientation = global_route.orientation[idx_min]
    time = int(initial_state.time_step + t_h/dt)

    return PlanningProblem(0, initial_state,
                           GoalRegion([CustomState(time_step=Interval(time, time),
                                                   position=Rectangle(length, 5, position, orientation))]))


def compute_global_route(sc: Scenario, pp: PlanningProblem) -> np.ndarray:
    """
    Computes global route from a given initial state to goal region.
    This route should not be used for planning. It is mainly used for extracting the sub-planning problems.

    :param sc: CommonRoad scenario.
    :param pp: Planning problem.
    :return: Route.
    """
    return RoutePlanner(sc, pp).plan_routes().retrieve_first_route().reference_path


class CommonRoadPlannerController(CarlaController):
    """Controller which uses trajectory generated by CommonRoad planner as input."""

    def __init__(self, actor: carla.Actor, planner: TrajectoryPlannerInterface, predictor: PredictorInterface,
                 pp: PlanningProblem, sc: Scenario, control_type: VehicleControlType, dt: float,
                 control_config: ControlParams, vehicle_params: VehicleParams = VehicleParams()):
        """
        Initialization of CommonRoad planner controller.

        :param actor: CARLA actor.
        :param planner: CommonRoad planner.
        :param predictor: CommonRoad predictor.
        :param pp: CommonRoad planning problem.
        :param sc: Base scenario containing road network and static obstacles.
        :param control_type: CARLA control type used for CommonRoad planner.
        :param dt: Time step size.
        :param control_config: CARLA controller params.
        :param vehicle_params: Vehicle parameters.
        """
        super().__init__(actor)
        self._planner = planner
        self._predictor = predictor
        self._base_sc = copy.deepcopy(sc)
        self._actor_id = int
        self._global_route = RouteData(compute_global_route(self._base_sc, pp))
        self._current_trajectory = None
        self._controller = self._create_controller(control_type, dt, control_config)
        self._vehicle_params = vehicle_params

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

    def _reset_base_scenario(self):
        """Removes all dynamic obstacles from base scenario."""
        for obs in self._base_sc.dynamic_obstacles:
            self._base_sc.remove_obstacle(obs)

    def control(self, state: Optional[TraceState] = None):
        """
        Computes and applies CARLA steering wheel control.

        :param state: State which should be reached at next time step.
        """
        self._reset_base_scenario()
        world = self._actor.get_world()
        sc = create_scenario_from_world(world, self._base_sc)
        if self._predictor is not None:
            sc = self._predictor.predict(sc, 0)
        pp = get_planning_problem_from_world(self._actor, self._vehicle_params, 6, 0.1, self._global_route)
        self._current_trajectory = self._planner.plan(sc, pp)
        self._controller.control(self._current_trajectory.state_list[0])
