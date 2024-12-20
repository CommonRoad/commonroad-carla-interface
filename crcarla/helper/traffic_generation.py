import logging
import time
from typing import Callable, Dict, List, Optional, Tuple, Union

import carla
from numpy import random

from crcarla.helper.config import SimulationParams
from crcarla.helper.utils import create_cr_pedestrian_from_walker, create_cr_vehicle_from_actor
from crcarla.objects.pedestrian import PedestrianInterface
from crcarla.objects.vehicle import VehicleInterface

SetAutopilot = carla.command.SetAutopilot
FutureActor = carla.command.FutureActor
SpawnActor = carla.command.SpawnActor

# based on CARLA's traffic generation example
# script: https://github.com/carla-simulator/carla/blob/dev/PythonAPI/examples/generate_traffic.py


def create_actors(
    client: carla.Client,
    world: carla.World,
    tm: carla.TrafficManager,
    config: SimulationParams,
    generate_object_id: Callable[[], int],
    sync: bool,
    ego_location: Optional[carla.Location] = None,
) -> List[Union[PedestrianInterface, VehicleInterface]]:
    """
    Spawns actors in CARLA as defined in configuration.

    :param client: CARLA client.
    :param world: CARLA world.
    :param tm: CARLA traffic manager.
    :param config: Simulation configuration.
    :param generate_object_id: Function to create unique  ID for CommonRoad obstacles.
    :param sync: Boolean indicating whether synchronous mode is used.
    :param ego_location: CARLA location of ego vehicle.
    """
    random.seed(config.tm.seed if config.tm.seed is not None else int(time.time()))

    blueprints_vehicles, blueprints_walkers = extract_blueprints(config, world)

    # Spawn vehicles
    spawn_vehicles(config, blueprints_vehicles, world, tm, ego_location)

    # Spawn Walkers
    all_walker_actors = spawn_walker_with_control(config, blueprints_walkers, client, world, generate_object_id)

    # tick required, otherwise call to get_transform returns [0, 0] position since world object contains the state
    # from last time step where obstacles were not present (https://github.com/carla-simulator/carla/issues/1424)
    if sync:
        world.tick()
    else:
        world.wait_for_tick()

    return init_cr_vehicles(generate_object_id, world, tm) + all_walker_actors


def init_cr_vehicles(
    generate_object_id: Callable[[], int], world: carla.World, traffic_manager: carla.TrafficManager
) -> List[VehicleInterface]:
    """
    Initializes CommonRoad vehicle obstacles.

    :param generate_object_id: Function to create unique IDs.
    :param world: CARLA world.
    :param traffic_manager: CARLA traffic manager.
    """
    obstacle_list = []
    for actor in world.get_actors():
        if "vehicle" in actor.type_id:
            cr_obstacle = create_cr_vehicle_from_actor(actor, generate_object_id(), 0)
            obstacle_list.append(VehicleInterface(cr_obstacle, world, traffic_manager, actor=actor))

    return obstacle_list


def extract_blueprints(
    config: SimulationParams, world: carla.World
) -> Tuple[List[carla.ActorBlueprint], List[carla.ActorBlueprint]]:
    """
    Extracts available blueprints for vehicles and walkers.

    :param config: Simulation config.
    :param world: CARLA world.
    :return: List of vehicle blueprints and list of walker blueprints
    """
    blueprints_vehicles = world.get_blueprint_library().filter(config.filter_vehicle)
    blueprints_walkers = world.get_blueprint_library().filter(config.filter_pedestrian)
    if config.safe_vehicles:
        blueprints_vehicles = [x for x in blueprints_vehicles if int(x.get_attribute("number_of_wheels")) == 4]
        blueprints_vehicles = [x for x in blueprints_vehicles if not x.id.endswith("microlino")]
        blueprints_vehicles = [x for x in blueprints_vehicles if not x.id.endswith("carlacola")]
        blueprints_vehicles = [x for x in blueprints_vehicles if not x.id.endswith("cybertruck")]
        blueprints_vehicles = [x for x in blueprints_vehicles if not x.id.endswith("t2")]
        blueprints_vehicles = [x for x in blueprints_vehicles if not x.id.endswith("sprinter")]
        blueprints_vehicles = [x for x in blueprints_vehicles if not x.id.endswith("firetruck")]
        blueprints_vehicles = [x for x in blueprints_vehicles if not x.id.endswith("ambulance")]
    blueprints_vehicles = sorted(blueprints_vehicles, key=lambda bp: bp.id)
    return blueprints_vehicles, list(blueprints_walkers)


def spawn_walker_with_control(
    config: SimulationParams,
    blueprints_walkers: List[carla.ActorBlueprint],
    client: carla.Client,
    world: carla.World,
    generate_object_id: Callable[[], int],
) -> List[PedestrianInterface]:
    """
    Spawns walkers as defined in provided config.

    :param config: Simulation configuration.
    :param blueprints_walkers: Available CARLA blueprints for walkers.
    :param client: CARLA client.
    :param world: CARLA world.
    :param generate_object_id: Function to create unique ID for CommonRoad obstacles.
    """
    config.logger.info("Traffic Generation spawn walkers.")
    if config.seed_walker:
        world.set_pedestrians_seed(config.seed_walker)
        random.seed(config.seed_walker)
    # 1. take all the random locations to spawn
    spawn_points = extract_spawn_points(config, world)

    # 2. we spawn the walker object
    walker_speed, walkers_list = spawn_walkers(blueprints_walkers, config, spawn_points, client)

    # 3. we spawn the walker controller
    all_actors, all_id = spawn_walker_controller(walkers_list, client, world, config.logger)

    # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
    # set how many pedestrians can cross the road
    cr_walkers_list = init_walker_controller_traffic_generation(
        all_actors, all_id, config, generate_object_id, walker_speed, client, world
    )

    return cr_walkers_list


def extract_spawn_points(config: SimulationParams, world: carla.World) -> List[carla.Transform]:
    """
    Extracts available spawn points.

    :param config: Simulation config.
    :param world: CARLA world.
    :return: List of CARLA transform objects.
    """
    spawn_points = []
    while len(spawn_points) < config.number_walkers:
        spawn_point = carla.Transform()
        loc = world.get_random_location_from_navigation()
        if loc is not None:
            spawn_point.location = loc
            spawn_points.append(spawn_point)
    return spawn_points


def init_walker_controller_traffic_generation(
    all_actors: List[carla.Actor],
    all_id: List[int],
    config: SimulationParams,
    generate_object_id: Callable[[], int],
    walker_speed: List[float],
    client: carla.Client,
    world: carla.World,
) -> List[PedestrianInterface]:
    """
    Initializes AI Walker controllers and creates pedestrian interface object.

    :param all_actors: List of all CARLA actors.
    :param all_id:  List of all CARLA actor IDs.
    :param config: Simulation config.
    :param generate_object_id: Function to create unique CommonRoad obstacle IDs.
    :param walker_speed: Maximum speed for each walker.
    :param client: CARLA client.
    :param world: CARLA world.
    :return: List of pedestrian interfaces.
    """
    cr_walkers_list = []
    world.set_pedestrians_cross_factor(config.tm.global_percentage_pedestrians_crossing)
    for idx in range(0, len(all_id), 2):
        # start walker
        all_actors[idx].start()
        # set walk to random point
        all_actors[idx].go_to_location(world.get_random_location_from_navigation())
        # max speed
        all_actors[idx].set_max_speed(float(walker_speed[int(idx / 2)]))
    for idx in range(1, len(all_id), 2):
        cr_walkers_list.append(
            PedestrianInterface(
                create_cr_pedestrian_from_walker(
                    all_actors[idx], generate_object_id(), config.pedestrian_default_shape
                ),
                world,
                client.get_trafficmanager(),
                actor=all_actors[idx],
            )
        )
    return cr_walkers_list


def spawn_walker_controller(
    walkers_list: List[Dict[str, int]],
    client: carla.Client,
    world: carla.World,
    logger: logging.Logger,
) -> Tuple[List[carla.Actor], List[int]]:
    """
    Spawns walker AI controller.

    :param walkers_list: List of walker IDs.
    :param client: CARLA client.
    :param world: CARLA world.
    :param logger: Logging instance.
    :return: List of all actors and list of all actor IDs.
    """
    batch = []
    all_id = []
    walker_controller_bp = world.get_blueprint_library().find("controller.ai.walker")
    for walker in walkers_list:
        if world.get_actor(walker["id"]) is not None:
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walker["id"]))
    try:
        results = client.apply_batch_sync(batch, True)
    except RuntimeError:
        return [], []

    for i, res in enumerate(results):
        if res.error:
            logger.error(res.error)
        else:
            walkers_list[i]["con"] = res.actor_id
    # 4. we put together the walkers and controllers id to get the objects from their id
    for walker in walkers_list:
        all_id.append(walker["con"])
        all_id.append(walker["id"])
    all_actors = world.get_actors(all_id)
    return all_actors, all_id


def spawn_walkers(
    blueprints_walkers: List[carla.ActorBlueprint],
    config: SimulationParams,
    spawn_points: List[carla.Transform],
    client: carla.Client,
) -> Tuple[List[float], List[Dict[str, int]]]:
    """
    Spawns walkers.

    :param blueprints_walkers: Available walker blueprints.
    :param config: Simulation config.
    :param spawn_points: Available spawn points.
    :param client: CARLA client.
    :param logger: Logger instance.
    :return: Maximum walker speed and dictionary containing actor IDs.
    """
    batch = []
    walker_speed = []
    walkers_list = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(blueprints_walkers)
        # set as not invincible
        if walker_bp.has_attribute("is_invincible"):
            walker_bp.set_attribute("is_invincible", "false")
        # set the max speed
        if walker_bp.has_attribute("speed"):
            if random.random() > config.tm.global_percentage_pedestrians_running:
                # walking
                walker_speed.append(walker_bp.get_attribute("speed").recommended_values[1])
            else:
                # running
                walker_speed.append(walker_bp.get_attribute("speed").recommended_values[2])
        else:
            config.logger.info("spawn_walker: Walker has no speed")
            walker_speed.append(0.0)
        batch.append(SpawnActor(walker_bp, spawn_point))
    try:
        results = client.apply_batch_sync(batch, True)
    except RuntimeError:
        results = []

    walker_speed2 = []
    for i, res in enumerate(results):
        if res.error:
            config.logger.error(res.error)
        else:
            walkers_list.append({"id": res.actor_id})
            walker_speed2.append(walker_speed[i])
    walker_speed = walker_speed2
    return walker_speed, walkers_list


def spawn_vehicles(
    config: SimulationParams,
    blueprints: List[carla.ActorBlueprint],
    world: carla.World,
    traffic_manager: carla.TrafficManager,
    ego_location: carla.Location,
):
    """
    Spawns vehicles as defined in provided config.

    :param config: Simulation configuration.
    :param blueprints: Available CARLA blueprints for vehicles.
    :param world: CARLA world.
    :param traffic_manager: CARLA traffic manager.
    :param ego_location: Location of ego vehicle. Only spawn points in a certain distance are considered.
    """
    config.logger.info("Traffic Generation spawn vehicles.")

    spawn_points = world.get_map().get_spawn_points()
    number_of_spawn_points = len(spawn_points)

    if config.number_vehicles < number_of_spawn_points:
        random.shuffle(spawn_points)

    num_vehicles = 0
    for transform in spawn_points:
        if ego_location is not None and transform.location.distance(ego_location) < config.spawn_point_distance_ego:
            continue
        if num_vehicles >= config.number_vehicles:
            break
        blueprint = select_blueprint(blueprints)

        # spawn the cars and set their autopilot and light state all together
        spawned_actor = world.try_spawn_actor(blueprint, transform)

        if spawned_actor is not None:
            spawned_actor.set_autopilot(True)
            traffic_manager.update_vehicle_lights(spawned_actor, True)
            traffic_manager.ignore_walkers_percentage(spawned_actor, config.tm.ignore_walkers_percentage)
            traffic_manager.keep_right_rule_percentage(spawned_actor, config.tm.keep_right_rule_percentage)
            traffic_manager.ignore_vehicles_percentage(spawned_actor, config.tm.ignore_vehicles_percentage)
            traffic_manager.ignore_signs_percentage(spawned_actor, config.tm.ignore_signs_percentage)
            traffic_manager.ignore_lights_percentage(spawned_actor, config.tm.ignore_lights_percentage)
            traffic_manager.random_left_lanechange_percentage(
                spawned_actor, config.tm.random_left_lane_change_percentage
            )
            traffic_manager.random_right_lanechange_percentage(
                spawned_actor, config.tm.random_right_lane_change_percentage
            )
            num_vehicles += 1
        else:
            continue


def select_blueprint(blueprints: List[carla.ActorBlueprint]) -> carla.ActorBlueprint:
    """
    Selects random blueprint from available ones anc configures it randomly. e.g., color, id, etc.

    :param blueprints: List of available CARLA actor blueprints.
    """
    blueprint = random.choice(blueprints)
    if blueprint.has_attribute("color"):
        color = random.choice(blueprint.get_attribute("color").recommended_values)
        blueprint.set_attribute("color", color)
    if blueprint.has_attribute("driver_id"):
        driver_id = random.choice(blueprint.get_attribute("driver_id").recommended_values)
        blueprint.set_attribute("driver_id", driver_id)
    else:
        blueprint.set_attribute("role_name", "autopilot")
    if blueprint.has_attribute("is_invincible"):
        blueprint.set_attribute("is_invincible", "true")
    return blueprint
