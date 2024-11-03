import copy
import math
from dataclasses import dataclass
from typing import Optional, Union

import carla
import numpy as np
from commonroad.common.util import make_valid_orientation
from commonroad.geometry.shape import Rectangle
from commonroad.planning.goal import GoalRegion, Interval
from commonroad.planning.planner_interface import TrajectoryPlannerInterface
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import CustomState, InitialState, TraceState
from commonroad.scenario.trajectory import Trajectory
from commonroad_dc.geometry.geometry import (
    compute_orientation_from_polyline,
    compute_pathlength_from_polyline,
)
import commonroad_route_planner.fast_api.fast_api as rfapi
from commonroad_rp.utility.config import VehicleConfiguration
from crpred.predictor_interface import PredictorInterface
from scipy import spatial

from crcarla.controller.controller import CarlaController, TransformControl
from crcarla.controller.vehicle_controller import (
    AckermannController,
    PIDController,
    VehicleBehaviorAgentPathFollowingControl,
    VehicleTMPathFollowingControl,
)
from crcarla.helper.config import ControlParams, VehicleControlType
from crcarla.helper.utils import create_cr_initial_state_from_actor, create_cr_vehicle_from_actor


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


def create_scenario_from_world(world: carla.World, sc: Scenario, ego_id: int, time_step: int) -> Scenario:
    """
    Creates scenario without prediction from CARLA world.

    :param world: CARLA world.
    :param sc: Base scenario containing road network and static obstacles.
    :param ego_id: ID of ego vehicle.
    :param time_step: Initial time step which should be used.
    :return: CommonRoad scenario.
    """
    for actor in world.get_actors():
        if actor.id == ego_id or "number_of_wheels" not in actor.attributes.keys():
            continue
        sc.add_objects(create_cr_vehicle_from_actor(actor, sc.generate_object_id(), time_step))
    return sc


def get_planning_problem_from_world(
    actor: carla.Actor,
    vehicle_params: VehicleConfiguration,
    t_h: float,
    dt: float,
    global_route: RouteData,
    current_time_step: int,
    traj: Trajectory,
    for_transform_controller: bool,
) -> PlanningProblem:
    """
    Creates planning problem from global route.

    :param actor: Ego vehicle actor.
    :param vehicle_params: Ego vehicle parameters.
    :param t_h: Trajectory time horizon [s].
    :param dt: Time step size [s].
    :param global_route: Global route.
    :param current_time_step: Current time step.
    :param traj: Trajectory
    :param for_transform_controller: bool whether pp is for transform controller
    :return: CommonRoad planning problem.
    """

    # if transform controller used, set initial state for next planning iteration to 1st traj state of planning result
    if for_transform_controller:
        cur = traj.state_list[1]
        initial_state = InitialState()
        initial_state.time_step = cur.time_step
        initial_state.position = cur.position
        initial_state.orientation = cur.orientation
        initial_state.velocity = cur.velocity
        initial_state.acceleration = cur.acceleration if hasattr(cur, "acceleration") else 0.0
        initial_state.yaw_rate = cur.yaw_rate if hasattr(cur, "yaw_rate") else 0.0
        initial_state.slip_angle = 0
    else:
        initial_state = create_cr_initial_state_from_actor(actor, current_time_step)

    min_dist = (initial_state.velocity**2) / (2 * vehicle_params.a_max)
    max_dist = initial_state.velocity * t_h + 0.5 * vehicle_params.a_max * 6**2

    _, init_idx = spatial.KDTree(global_route.route).query(initial_state.position)
    distance_min = global_route.path_length[init_idx] + min_dist
    distance_max = global_route.path_length[init_idx] + max_dist

    idx_min = max(np.searchsorted(global_route.path_length, distance_min) - 1, 0)
    idx_max = min(np.searchsorted(global_route.path_length, distance_max) + 1, len(global_route.route) - 1)
    position = 0.5 * (global_route.route[idx_min] + global_route.route[idx_max])
    length = global_route.path_length[idx_max] - global_route.path_length[idx_min]
    orientation = global_route.orientation[idx_min]
    time = int(initial_state.time_step + t_h / dt)

    return PlanningProblem(
        0,
        initial_state,
        GoalRegion(
            [
                CustomState(
                    time_step=Interval(time - 1, time),
                    position=Rectangle(float(length), 20, position, float(orientation)),
                )
            ]
        ),
    )


def compute_global_route(sc: Scenario, pp: PlanningProblem) -> np.ndarray:
    """
    Computes global route from a given initial state to goal region.
    This route should not be used for planning. It is mainly used for extracting the sub-planning problems.

    :param sc: CommonRoad scenario.
    :param pp: Planning problem.
    :return: Route.
    """
    return rfapi.generate_reference_path_from_lanelet_network_and_planning_problem(
        lanelet_network=sc.lanelet_network, planning_problem=pp
    ).reference_path


class CommonRoadPlannerController(CarlaController):
    """Controller which uses trajectory generated by CommonRoad planner as input."""

    def __init__(
        self,
        actor: carla.Actor,
        planner: TrajectoryPlannerInterface,
        predictor: PredictorInterface,
        pp: PlanningProblem,
        sc: Scenario,
        control_type: VehicleControlType,
        dt: float,
        t_h: float,
        control_config: ControlParams,
        vehicle_params: VehicleConfiguration = VehicleConfiguration(),
    ):
        """
        Initialization of CommonRoad planner controller.

        :param actor: CARLA actor.
        :param planner: CommonRoad planner.
        :param predictor: CommonRoad predictor.
        :param pp: CommonRoad planning problem.
        :param sc: Base scenario containing road network and static obstacles.
        :param control_type: CARLA control type used for CommonRoad planner.
        :param dt: Time step size.
        :param t_h: Intermediate goal time horizon [s].
        :param control_config: CARLA controller params.
        :param vehicle_params: Vehicle parameters.
        """
        super().__init__(actor)
        self._planner = planner
        self._predictor = predictor
        self._base_sc = copy.deepcopy(sc)
        self._base_pp = copy.deepcopy(pp)
        self._actor_id = int
        self._global_route = RouteData(compute_global_route(self._base_sc, pp))
        self._current_trajectory = None
        self._controller = self._create_controller(control_type, dt, control_config)
        self._dt = dt
        self._time_horizon_sec = t_h
        self._vehicle_params = vehicle_params
        self._current_time_step = 0
        self._logger = control_config.logger

    def _create_controller(
        self, control_type: VehicleControlType, dt: float, control_config: ControlParams
    ) -> Union[
        TransformControl,
        PIDController,
        AckermannController,
        VehicleBehaviorAgentPathFollowingControl,
        VehicleTMPathFollowingControl,
    ]:
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
            return AckermannController(self._actor, config=control_config, dt=dt)
        if control_type is VehicleControlType.PATH_TM:
            return VehicleTMPathFollowingControl(self._actor)
        if control_type is VehicleControlType.PATH_AGENT:
            return VehicleBehaviorAgentPathFollowingControl(self._actor)
        self._logger.error("CommonRoadPlannerController::_create_controller: Unknown controller type.")
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
        sc = create_scenario_from_world(world, self._base_sc, self._actor.id, self._current_time_step)
        if self._predictor is not None:
            sc = self._predictor.predict(sc, self._current_time_step)
        # as the last planning result is needed if transform control is used, use initial state for first iteration
        if self._current_trajectory is None:
            pp = self._base_pp
        else:
            pp = get_planning_problem_from_world(
                self._actor,
                self._vehicle_params,
                self._time_horizon_sec,
                self._dt,
                self._global_route,
                self._current_time_step,
                self._current_trajectory,
                isinstance(self._controller, TransformControl),
            )
        pp.goal = self._base_pp.goal

        # from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
        # from commonroad.planning.planning_problem import PlanningProblemSet
        #
        # CommonRoadFileWriter(sc, PlanningProblemSet([pp])).write_to_file(f"test{self._current_time_step}.xml",
        #                                                                  OverwriteExistingFile.ALWAYS)

        # if transform control, steering needs to be set manual because actual steering angle is always zero
        if isinstance(self._controller, TransformControl):
            if self._current_trajectory is None:
                steering_angle = 0
            else:
                steering_angle = self._current_trajectory.state_list[1].steering_angle
        else:
            try:
                steer = self._actor.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
            except RuntimeError:
                steer = 0
            steering_angle = make_valid_orientation(steer * (math.pi / 180))
        self._current_trajectory = self._planner.plan(sc, pp, steering_angle=steering_angle)
        self._controller.control(self._current_trajectory.state_list[1])
        self._current_time_step += 1
