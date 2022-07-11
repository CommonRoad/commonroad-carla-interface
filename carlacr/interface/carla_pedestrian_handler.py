import carla
import numpy as np
import sys
import time
from math import sqrt
from typing import List, Dict
from commonroad.geometry.shape import Circle
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from numpy import array, random
import logging
from configurations.set_configs import set_configs
logger = logging.getLogger(__name__)


class CarlaPedestrianHandler:
    """
    Creates and controlls walker in CARLA
    """

    def __init__(self, cr_scenario: Scenario, carla_client: carla.Client, carla_pedestrians: int):
        """
        :param cr_scenario: CommonRoad scenario object
        :param carla_client: carla.Client() object connected to the simulation
        :param carla_pedestrians: maximum number of walkers that should be created
        """
        self.client = carla_client
        self.scenario = cr_scenario
        self.numb_ped = carla_pedestrians
        self.spawned = False
        self.walkers_list: List[Dict] = []
        self.actor_ids: List[str] = []


    def __str__(self):
        resp = f"numb of pedestrians: {self.numb_ped}\n"
        resp += f"walkers_list: {self.walkers_list}\n"
        resp += f"actor_ids: {self.actor_ids}\n"
        resp += f"numb pedestrians spawned: {len(self.walkers_list)}\n"
        return resp

    def get_cr_dyn_obs_list(self) -> List[DynamicObstacle]:
        """
        Returns list containing all spawned walkers as CommonRoad dynamic obstacles
        :return: list of dynamic obstacles generated from spawned walkers
        """
        cr_dyn_obs_list = []
        for pedestrian in self.walkers_list:
            try:
                actor = self.client.get_world().get_actor(pedestrian["id"])
                transform = actor.get_transform()
                location = transform.location
                rotation = transform.rotation
                vel_vec = actor.get_velocity()
                vel = sqrt(vel_vec.x ** 2 + vel_vec.y ** 2)  # velocity
                dynamic_obstacle_init_state = State(position=array([location.x, -location.y]),
                                                    orientation=-((rotation.yaw * np.pi) / 180), velocity=vel)
                # length = actor.bounding_box.extent.x * 2
                # width = actor.bounding_box.extent.y * 2
                dynamic_obstacle_type = ObstacleType.PEDESTRIAN
                dynamic_obstacle_shape = Circle(radius=0.4)
                obs = DynamicObstacle(pedestrian["cr_id"], dynamic_obstacle_type, dynamic_obstacle_shape,
                                      dynamic_obstacle_init_state)
                cr_dyn_obs_list.append(obs)
            except Exception as e:
                logger.debug("Following error occurred while retrieving current position for:")
                logger.debug(self)
                logger.error(e, exc_info=sys.exc_info())
        return cr_dyn_obs_list

    def spawn(self):
        """
        Spawns self.numb_ped of pedestrians in CARLA (if not already spawned)

        Based on CARLAs PythonAPI example: PythonAPI/examples/spawn_npc.py
        Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
        Barcelona (UAB).

        This work is licensed under the terms of the MIT license.
        For a copy, see <https://opensource.org/licenses/MIT>.
        """
        world = self.client.get_world()
        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)
        if self.numb_ped < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif self.numb_ped > number_of_spawn_points:
            msg = 'requested %d pedestrians, but could only find %d spawn points'
            logging.warning(msg, self.numb_ped, number_of_spawn_points)
            self.numb_ped = number_of_spawn_points
        if self.spawned:
            return

        """
        Load configs for percentage_pedestrians_running and percentage_pedestrians_crossing
        from Configurations
        """
        config = set_configs()
        # how many pedestrians will run, default = 0
        percentage_pedestrians_running = config.config_carla_pedestrian.percentage_pedestrians_running
        # how many pedestrians will walk through the road, default = 0
        percentage_pedestrians_crossing = config.config_carla_pedestrian.percentage_pedestrians_crossing

        world = self.client.get_world()
        blueprints_walkers = world.get_blueprint_library().filter("walker.pedestrian.*")

        # 1. take all the random locations to spawn
        spawn_points = []
        for i in range(self.numb_ped):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if loc is not None:
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
                if random.random() > percentage_pedestrians_running:
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                logger.debug("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(carla.command.SpawnActor(walker_bp, spawn_point))
        results = self.client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in enumerate(results):
            if results[i].error:
                logger.debug(results[i].error)
            else:
                self.walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in enumerate(self.walkers_list):
            batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), self.walkers_list[i]["id"]))
        results = self.client.apply_batch_sync(batch, True)
        for i in enumerate(results):
            if results[i].error:
                logging.error(results[i].error)
            else:
                self.walkers_list[i]["con"] = results[i].actor_id
                self.walkers_list[i]["cr_id"] = self.scenario.generate_object_id()
        # 4. we put altogether the walkers and controllers id to get the objects from their id
        for i in enumerate(self.walkers_list):
            self.actor_ids.append(self.walkers_list[i]["con"])
            self.actor_ids.append(self.walkers_list[i]["id"])
        all_actors = world.get_actors(self.actor_ids)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        world.tick()

        # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        world.set_pedestrians_cross_factor(percentage_pedestrians_crossing)
        for i in range(0, len(self.actor_ids), 2):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            # max speed
            all_actors[i].set_max_speed(float(walker_speed[int(i / 2)]))
            self.spawned = True

    def destroy(self):
        """
        Destroys all spawned walker in CARLA
        """
        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(self.actor_ids), 2):
            try:
                self.client.get_world().get_actor(self.actor_ids[i]).stop()
            except Exception as e:
                logger.error(e)
        print('\n destroying %d walkers' % len(self.walkers_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_ids])

        time.sleep(0.5)
