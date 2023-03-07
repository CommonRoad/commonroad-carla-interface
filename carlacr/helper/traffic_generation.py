#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example script to generate traffic in the simulation"""

import time


import carla

SetAutopilot = carla.command.SetAutopilot
FutureActor = carla.command.FutureActor
SpawnActor = carla.command.SpawnActor

import logging
from numpy import random

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

def main(client, args, traffic_manager):
    world = client.get_world()
    random.seed(args.seed if args.seed is not None else int(time.time()))

    blueprints, blueprintsWalkers = extract_blueprints(args, world)

    # Spawn vehicles
    all_vehicle_actors = spawn_vehicle(args, blueprints, client, traffic_manager)

    # Spawn Walkers
    all_walker_actors = spawn_walker(args, blueprintsWalkers, client)


def extract_blueprints(args, world):
    blueprints = get_actor_blueprints(world, args.filterv, args.generationv)
    blueprintsWalkers = get_actor_blueprints(world, args.filterw, args.generationw)
    if args.safe:
        blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
        blueprints = [x for x in blueprints if not x.id.endswith('microlino')]
        blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
        blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
        blueprints = [x for x in blueprints if not x.id.endswith('t2')]
        blueprints = [x for x in blueprints if not x.id.endswith('sprinter')]
        blueprints = [x for x in blueprints if not x.id.endswith('firetruck')]
        blueprints = [x for x in blueprints if not x.id.endswith('ambulance')]
    blueprints = sorted(blueprints, key=lambda bp: bp.id)
    return blueprints, blueprintsWalkers


def spawn_walker(args, blueprintsWalkers, client):
    walkers_list = []
    all_id = []
    world = client.get_world()
    if args.seedw:
        world.set_pedestrians_seed(args.seedw)
        random.seed(args.seedw)
    # 1. take all the random locations to spawn
    spawn_points = []
    for i in range(args.number_of_walkers):
        spawn_point = carla.Transform()
        loc = world.get_random_location_from_navigation()
        if (loc != None):
            spawn_point.location = loc
            spawn_points.append(spawn_point)
    # 2. we spawn the walker object
    batch = []
    walker_speed = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(blueprintsWalkers)
        # set as not invincible
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        # set the max speed
        if walker_bp.has_attribute('speed'):
            if (random.random() > args.percentagePedestriansRunning):
                # walking
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
            else:
                # running
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
        else:
            print("Walker has no speed")
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
    # wait for a tick to ensure client receives the last transform of the walkers we have just created
    if args.asynch or not args.sync:
        world.wait_for_tick()
    else:
        world.tick()
    # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
    # set how many pedestrians can cross the road
    world.set_pedestrians_cross_factor(args.percentagePedestriansCrossing)
    for i in range(0, len(all_id), 2):
        # start walker
        all_actors[i].start()
        # set walk to random point
        all_actors[i].go_to_location(world.get_random_location_from_navigation())
        # max speed
        all_actors[i].set_max_speed(float(walker_speed[int(i / 2)]))

    return all_actors


def spawn_vehicle(args, blueprints, client, traffic_manager):
    batch = []
    vehicles_list = []
    world = client.get_world()

    spawn_points = world.get_map().get_spawn_points()
    number_of_spawn_points = len(spawn_points)

    if args.number_of_vehicles < number_of_spawn_points:
        random.shuffle(spawn_points)

    for n, transform in enumerate(spawn_points):
        if n >= args.number_of_vehicles:
            break
        blueprint = random.choice(blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        else:
            blueprint.set_attribute('role_name', 'autopilot')

        # spawn the cars and set their autopilot and light state all together
        batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))
    for response in client.apply_batch_sync(batch, args.sync):
        if response.error:
            logging.error(response.error)
        else:
            vehicles_list.append(response.actor_id)
    # Set automatic vehicle lights update if specified
    for actor in world.get_actors(vehicles_list):
        traffic_manager.update_vehicle_lights(actor, True)
