import dataclasses
import logging
from multiprocessing import Pool
from pathlib import Path
from typing import Union

import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.polyline_util import compute_total_polyline_length
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.scenario import ScenarioID

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


@dataclasses.dataclass
class ScenarioEval:
    scenario_id: Union[ScenarioID, str]
    num_obstacles: int
    num_intersections: int
    num_traffic_signs: int
    num_traffic_lights: int
    scenario_time_duration: float
    scenario_distance: float
    road_network_length: float
    num_bus: int
    num_cars: int
    num_pedestrians: int
    num_trucks: int


def evaluate_single_scenario(sc_path: Path) -> ScenarioEval:
    logger.info(f"Evaluate {sc_path.stem}.")
    sc, _ = CommonRoadFileReader(sc_path).open()
    return ScenarioEval(
        scenario_id=sc.scenario_id,
        num_obstacles=len(sc.obstacles),
        num_intersections=len(sc.lanelet_network.intersections),
        num_traffic_signs=len(sc.lanelet_network.traffic_signs),
        num_traffic_lights=len(sc.lanelet_network.traffic_lights),
        scenario_time_duration=sum(
            [obs.prediction.trajectory.final_state.time_step - obs.initial_state.time_step for obs in sc.obstacles]
        )
        * sc.dt,
        scenario_distance=sum(
            [
                (
                    compute_total_polyline_length(
                        np.array([state.position for state in obs.prediction.trajectory.state_list])
                    )
                    if len(obs.prediction.trajectory.state_list) > 1
                    else 0.0
                )
                for obs in sc.obstacles
            ]
        )
        / 1000,
        road_network_length=sum([la.distance[-1] for la in sc.lanelet_network.lanelets]) / 1000,
        num_bus=len(list(filter(lambda obs: obs.obstacle_type == ObstacleType.BUS, sc.obstacles))),
        num_cars=len(list(filter(lambda obs: obs.obstacle_type == ObstacleType.CAR, sc.obstacles))),
        num_pedestrians=len(list(filter(lambda obs: obs.obstacle_type == ObstacleType.PEDESTRIAN, sc.obstacles))),
        num_trucks=len(list(filter(lambda obs: obs.obstacle_type == ObstacleType.TRUCK, sc.obstacles))),
    )
    # eva = scenario_evals[-1]
    # print(eva.scenario_id, eva.num_obstacles - eva.num_pedestrians, eva.scenario_distance,
    #       eva.scenario_time_duration,
    #       eva.road_network_length, eva.num_intersections)


num_threads = 8
scenario_path = Path(__file__).parent / "generated_scenarios"

with Pool(processes=num_threads) as pool:
    scenario_evals = pool.map(evaluate_single_scenario, list(scenario_path.glob("*.pb")))


evals_combined = {}
for eva in scenario_evals:
    if evals_combined.get(eva.scenario_id.map_name) is None:
        evals_combined[eva.scenario_id.map_name] = ScenarioEval(
            scenario_id=eva.scenario_id.map_name,
            num_obstacles=eva.num_obstacles,
            num_intersections=eva.num_intersections,
            num_traffic_signs=eva.num_traffic_signs,
            num_traffic_lights=eva.num_traffic_lights,
            scenario_time_duration=eva.scenario_time_duration,
            scenario_distance=eva.scenario_distance,
            road_network_length=eva.road_network_length,
            num_bus=eva.num_bus,
            num_cars=eva.num_cars,
            num_pedestrians=eva.num_pedestrians,
            num_trucks=eva.num_trucks,
        )

    else:
        evals_combined[eva.scenario_id.map_name].num_obstacles += eva.num_obstacles
        evals_combined[eva.scenario_id.map_name].scenario_time_duration += eva.scenario_time_duration
        evals_combined[eva.scenario_id.map_name].scenario_distance += eva.scenario_distance
        evals_combined[eva.scenario_id.map_name].num_bus += eva.num_bus
        evals_combined[eva.scenario_id.map_name].num_cars += eva.num_cars
        evals_combined[eva.scenario_id.map_name].num_pedestrians += eva.num_pedestrians
        evals_combined[eva.scenario_id.map_name].num_trucks += eva.num_trucks

evals_complete = None
for eva in evals_combined.values():
    print(
        eva.scenario_id,
        eva.num_obstacles - eva.num_pedestrians,
        eva.num_pedestrians,
        eva.scenario_distance,
        eva.scenario_time_duration,
        eva.road_network_length,
        eva.num_intersections,
    )
    if evals_complete is None:
        evals_complete = ScenarioEval(
            scenario_id="Accumulated",
            num_obstacles=eva.num_obstacles,
            num_intersections=eva.num_intersections,
            num_traffic_signs=eva.num_traffic_signs,
            num_traffic_lights=eva.num_traffic_lights,
            scenario_time_duration=eva.scenario_time_duration,
            scenario_distance=eva.scenario_distance,
            road_network_length=eva.road_network_length,
            num_bus=eva.num_bus,
            num_cars=eva.num_cars,
            num_pedestrians=eva.num_pedestrians,
            num_trucks=eva.num_trucks,
        )
    else:
        evals_complete.num_intersections += eva.num_intersections
        evals_complete.num_obstacles += eva.num_obstacles
        evals_complete.num_bus += eva.num_bus
        evals_complete.road_network_length += eva.road_network_length
        evals_complete.num_trucks += eva.num_trucks
        evals_complete.num_pedestrians += eva.num_pedestrians
        evals_complete.num_cars += eva.num_cars
        evals_complete.num_traffic_signs += eva.num_traffic_signs
        evals_complete.num_traffic_lights += eva.num_traffic_lights
        evals_complete.scenario_time_duration += eva.scenario_time_duration
        evals_complete.scenario_distance += eva.scenario_distance


print(
    evals_complete.scenario_id,
    evals_complete.num_obstacles - evals_complete.num_pedestrians,
    evals_complete.num_pedestrians,
    evals_complete.scenario_distance,
    evals_complete.scenario_time_duration,
    evals_complete.road_network_length,
    evals_complete.num_intersections,
)
