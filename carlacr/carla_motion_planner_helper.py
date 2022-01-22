from typing import List

import numpy as np
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.trajectory import State, Trajectory

"""
This module contains helper methods for carla_commondroad interface 
"""


def divide_scenario(scenario: Scenario, length_child_scenario: int = 5) -> List[Scenario]:
    """
    This method divide a big scenario into smaller one

    :param scenario: scenario to divide
    :param length_child_scenario: length of chile scenario

    :return: list of child scenario
    """
    assert length_child_scenario > 0
    scenario_child_list = []
    dynamic_obstacle_time_step_state_dict = {}
    max_timestep = calc_max_timestep(scenario)
    for obj in scenario.dynamic_obstacles:
        dynamic_obstacle_time_step_state_dict[(obj.obstacle_id, obj.initial_state.time_step)] = obj.initial_state
        for state in obj.prediction.trajectory.state_list:
            dynamic_obstacle_time_step_state_dict[(obj.obstacle_id, state.time_step)] = state
    if max_timestep % length_child_scenario == 0:
        num_child = max_timestep // length_child_scenario
    else:
        num_child = max_timestep // length_child_scenario + 1
    for i in range(num_child):
        # Create child scenario with map obstacle
        child_scenario = Scenario(dt=scenario.dt)
        for obj in scenario.environment_obstacle:
            child_scenario.add_objects(scenario.environment_obstacle)
        child_scenario.add_objects(scenario.lanelet_network)
        for obj in scenario.phantom_obstacle:
            child_scenario.add_objects(scenario.phantom_obstacle)
        # Add dynamic obstacle from parent scenario
        for obj in scenario.dynamic_obstacles:
            state_list = []
            start_time = int(i * length_child_scenario)
            if i != num_child - 1:
                length = length_child_scenario
            else:
                length = max_timestep - start_time
            for time in range(start_time, start_time + length):
                if (obj.obstacle_id, time) in dynamic_obstacle_time_step_state_dict:
                    current_state = dynamic_obstacle_time_step_state_dict[(obj.obstacle_id, time)]
                    new_state = State(position=current_state.position,
                                      orientation=current_state.orientation,
                                      time_step=max(current_state.time_step - start_time, 0)
                                      )
                    state_list.append(new_state)
            if state_list:
                new_trajectory = Trajectory(initial_time_step=max(obj.initial_state.time_step - start_time, 0),
                                            state_list=state_list)
                new_prediction = TrajectoryPrediction(trajectory=new_trajectory,
                                                      shape=obj.obstacle_shape,
                                                      center_lanelet_assignment=obj.prediction.center_lanelet_assignment,
                                                      shape_lanelet_assignment=obj.prediction.shape_lanelet_assignment)
                new_obj = DynamicObstacle(obstacle_id=obj.obstacle_id,
                                          obstacle_type=obj.obstacle_type,
                                          obstacle_shape=obj.obstacle_shape,
                                          initial_state=state_list[0],
                                          prediction=new_prediction,
                                          initial_center_lanelet_ids=obj.initial_center_lanelet_ids,
                                          initial_shape_lanelet_ids=obj.initial_shape_lanelet_ids,
                                          initial_signal_state=obj.initial_signal_state)
                if len(state_list) != 1:
                    new_prediction.trajectory.state_list.pop(0)
                child_scenario.add_objects(new_obj)
        # Add static obstacle from parent scenario
        for obj in scenario.static_obstacles:
            child_scenario.add_objects(obj)
        scenario_child_list.append(child_scenario)
    return scenario_child_list


def calc_max_timestep(scenario: Scenario) -> int:
    """
    Calculates maximal time step of current scenario

    :param scenario: scenario to calculate max time step
    :return: length of scenario
    """
    if scenario is None:
        return 0
    timesteps = [
        obstacle.prediction.occupancy_set[-1].time_step
        for obstacle in scenario.dynamic_obstacles
    ]
    max_timestep = np.max(timesteps) if timesteps else 0
    return max_timestep
