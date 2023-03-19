#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import time
import logging
from numpy import random
from typing import List, Union
import carla
from carlacr.helper.utils import create_cr_vehicle_from_actor, create_cr_pedestrian_from_walker
from carlacr.objects.vehicle import VehicleInterface
from carlacr.objects.pedestrian import PedestrianInterface
from carlacr.helper.config import SimulationParams

SetAutopilot = carla.command.SetAutopilot
FutureActor = carla.command.FutureActor
SpawnActor = carla.command.SpawnActor


def create_actors(client: carla.Client, config: SimulationParams, cr_id: int) -> List[Union[PedestrianInterface, VehicleInterface]]:
    traffic_manager = client.get_trafficmanager()
    world = client.get_world()
    random.seed(config.tm.seed if config.tm.seed is not None else int(time.time()))

    blueprints_vehicles, blueprints_walkers = extract_blueprints(config, world)

    # Spawn vehicles
    all_vehicle_actors = spawn_vehicle(config, blueprints_vehicles, client, traffic_manager, cr_id)
    cr_id += len(all_vehicle_actors)

    # Spawn Walkers
    all_walker_actors = spawn_walker(config, blueprints_walkers, client, cr_id)

    return all_vehicle_actors + all_walker_actors

def extract_blueprints(config: SimulationParams, world: carla.World):
    blueprints_vehicles = world.get_blueprint_library().filter(config.filter_vehicle)
    blueprints_walkers = world.get_blueprint_library().filter(config.filter_pedestrian)
    if config.safe_vehicles:
        blueprints_vehicles = [x for x in blueprints_vehicles if int(x.get_attribute('number_of_wheels')) == 4]
        blueprints_vehicles = [x for x in blueprints_vehicles if not x.id.endswith('microlino')]
        blueprints_vehicles = [x for x in blueprints_vehicles if not x.id.endswith('carlacola')]
        blueprints_vehicles = [x for x in blueprints_vehicles if not x.id.endswith('cybertruck')]
        blueprints_vehicles = [x for x in blueprints_vehicles if not x.id.endswith('t2')]
        blueprints_vehicles = [x for x in blueprints_vehicles if not x.id.endswith('sprinter')]
        blueprints_vehicles = [x for x in blueprints_vehicles if not x.id.endswith('firetruck')]
        blueprints_vehicles = [x for x in blueprints_vehicles if not x.id.endswith('ambulance')]
    blueprints_vehicles = sorted(blueprints_vehicles, key=lambda bp: bp.id)
    return blueprints_vehicles, blueprints_walkers


def spawn_walker(config: SimulationParams, blueprints_walkers, client: carla.Client, cr_id: int) -> List[PedestrianInterface]:
    logging.info("Traffic Generation spawn walkers.")
    walkers_list = []
    cr_walkers_list = []
    all_id = []
    world = client.get_world()
    if config.seed_walker:
        world.set_pedestrians_seed(config.seed_walker)
        random.seed(config.seed_walker)
    # 1. take all the random locations to spawn
    spawn_points = []
    for i in range(config.number_walkers):
        spawn_point = carla.Transform()
        loc = world.get_random_location_from_navigation()
        if (loc != None):
            spawn_point.location = loc
            spawn_points.append(spawn_point)

    # 2. we spawn the walker object
    batch = []
    walker_speed = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(blueprints_walkers)
        # set as not invincible
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        # set the max speed
        if walker_bp.has_attribute('speed'):
            if random.random() > config.tm.global_percentage_pedestrians_running:
                # walking
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
            else:
                # running
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
        else:
            logging.info("spawn_walker: Walker has no speed")
            walker_speed.append(0.0)
        batch.append(SpawnActor(walker_bp, spawn_point))
    results = client.apply_batch_sync(batch, True)
    walker_speed2 = []
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list.append({"id": results[i].actor_id})
            walker_speed2.append(walker_speed[i])
    walker_speed = walker_speed2

    # 3. we spawn the walker controller
    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list[i]["con"] = results[i].actor_id

    # 4. we put together the walkers and controllers id to get the objects from their id
    for i in range(len(walkers_list)):
        all_id.append(walkers_list[i]["con"])
        all_id.append(walkers_list[i]["id"])
    all_actors = world.get_actors(all_id)

    # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
    # set how many pedestrians can cross the road
    world.set_pedestrians_cross_factor(config.tm.global_percentage_pedestrians_crossing)
    for idx in range(0, len(all_id), 2):
        # start walker
        all_actors[idx].start()
        # set walk to random point
        all_actors[idx].go_to_location(world.get_random_location_from_navigation())
        # max speed
        all_actors[idx].set_max_speed(float(walker_speed[int(idx / 2)]))
    for actor in all_actors:
        cr_walkers_list.append(PedestrianInterface(
                create_cr_pedestrian_from_walker(actor, cr_id, config.pedestrian_default_shape), actor))
        cr_id += 1

    return cr_walkers_list


def spawn_vehicle(config: SimulationParams, blueprints, client: carla.Client,
                  traffic_manager: carla.TrafficManager, cr_id: int) -> List[VehicleInterface]:
    logging.info("Traffic Generation spawn vehicles.")
    batch = []
    vehicles_list = []
    world = client.get_world()

    spawn_points = world.get_map().get_spawn_points()
    number_of_spawn_points = len(spawn_points)

    if config.number_vehicles < number_of_spawn_points:
        random.shuffle(spawn_points)

    num_vehicles = 0
    for transform in spawn_points:
        if num_vehicles >= config.number_vehicles:
            break
        blueprint = select_blueprint(blueprints)

        # spawn the cars and set their autopilot and light state all together
        spawned_actor = world.try_spawn_actor(blueprint, transform)

        if spawned_actor is not None:
            vehicles_list.append(
                    VehicleInterface(create_cr_vehicle_from_actor(world.get_actor(spawned_actor.id), cr_id), spawned_actor))
            spawned_actor.set_autopilot(True)
            traffic_manager.update_vehicle_lights(spawned_actor, True)
            traffic_manager.ignore_walkers_percentage(spawned_actor, config.tm.ignore_walkers_percentage)
            traffic_manager.keep_right_rule_percentage(spawned_actor, config.tm.keep_right_rule_percentage)
            traffic_manager.ignore_vehicles_percentage(spawned_actor, config.tm.ignore_vehicles_percentage)
            traffic_manager.ignore_signs_percentage(spawned_actor, config.tm.ignore_signs_percentage)
            traffic_manager.ignore_lights_percentage(spawned_actor, config.tm.ignore_lights_percentage)
            traffic_manager.random_left_lanechange_percentage(spawned_actor,
                                                              config.tm.random_left_lane_change_percentage)
            traffic_manager.random_right_lanechange_percentage(spawned_actor,
                                                               config.tm.random_right_lane_change_percentage)
            cr_id += 1
            num_vehicles += 1
        else:
            continue

    return vehicles_list


def select_blueprint(blueprints):
    blueprint = random.choice(blueprints)
    if blueprint.has_attribute('color'):
        color = random.choice(blueprint.get_attribute('color').recommended_values)
        blueprint.set_attribute('color', color)
    if blueprint.has_attribute('driver_id'):
        driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
        blueprint.set_attribute('driver_id', driver_id)
    else:
        blueprint.set_attribute('role_name', 'autopilot')
    if blueprint.has_attribute('is_invincible'):
        blueprint.set_attribute('is_invincible', 'true')
    return blueprint
