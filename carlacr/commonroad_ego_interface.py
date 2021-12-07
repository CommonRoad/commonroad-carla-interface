#!/usr/bin/env python

import glob
import os
import sys
from enum import Enum

import carla
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.obstacle import ObstacleRole, ObstacleType, Obstacle
from commonroad.scenario.trajectory import Trajectory
from commonroad.visualization.mp_renderer import MPRenderer

from carlacr.vehicle_dict import (similar_by_area, similar_by_length,
                                  similar_by_width)


class CommonRoadEgoInterface:
    """
    Creates and controles the ego-vehicle in CARLA
    """

    def __init__(self, planning_problem: PlanningProblem = None, trajectory: Trajectory = None,
                 initial_state: Obstacle.initial_state = None):
        """
        :param planning_problem: CommonRoad planning problem object
        :param trajectory: CommonRoad trajectory for the ego-vehicle
        :param initial_state: initial_state of commonroad obstacle when commonroad obstacle used as ego vehicle
        """
        if planning_problem:
            self.init_state = planning_problem.initial_state
        elif initial_state:
            self.init_state = initial_state
        else:
            raise AttributeError("planning_problem and initial_state can not be None at the same time")
        self.trajectory = trajectory
        self.is_spawned = False
        self.carla_id = None
        if planning_problem:
            self.spawn_timestep = planning_problem.initial_state.time_step
        elif initial_state:
            self.spawn_timestep = initial_state.time_step
        self.actor_list = []

    def spawn(self, world: carla.World, physics=True, create_gif=False, path=None) -> carla.Actor:
        """
        Tries to spawn the ego-vehicle and a camera for it in the given CARLA world and returns the spawned vehicle.

        :param world: the CARLA world object
        :param physics: if physics should be enabled for the ego-vehicle
        :param create_gif: True if a GIF should be created
        :param path: base path of the directory where the GIF should be stored
        :return: if spawn successful the according CARLA actor else None
        """
        ego_transform = carla.Transform(
            carla.Location(x=self.init_state.position[0], y=-self.init_state.position[1], z=0.5),
            carla.Rotation(yaw=(-(180 * self.init_state.orientation) / np.pi)))
        ego_blueprint = world.get_blueprint_library().filter('model3')[0]  # Just for reference

        try:
            ego = world.try_spawn_actor(ego_blueprint, ego_transform)
            ego.set_simulate_physics(physics)

            self.carla_id = ego.id
            self.is_spawned = True

            sensor = world.spawn_actor(
                world.get_blueprint_library().find('sensor.camera.rgb'),
                carla.Transform(carla.Location(x=-5.5, z=3.8), carla.Rotation(pitch=-15)),
                attach_to=ego)

            self.actor_list.append(sensor)
            # sensor.listen(lambda image: self.process_image(image))
            # if create_gif and path:
            #     image_rgb.save_to_disk('%s/img/%.6d.jpg' % (path, image.frame))
            # sensor.listen(lambda image: image.save_to_disk('./images/ego/%.6d.jpg' % image.frame))
            print("Ego spawn successful")
            return ego

        except Exception as e:
            print("Error while spawning:")
            raise e

    def set_trajectory(self, trajectory: Trajectory):
        """
        Sets trajectory for the ego-vehicle

        :param trajectory: CommonRoad trajectory for the ego-vehicle
        """
        if trajectory is not None:
            self.trajectory = trajectory
            print("Ego-Vehicle Trajectory is set")
        else:
            print("Invalid Trajectory")

    def update_position_by_time(self, world: carla.World, timestep: int):
        """
        Tries to update the position of the ego-vehicle

        :param world: the CARLA world object
        :param timestep: timestep that should be used to update
        """
        try:
            if self.is_spawned and (self.trajectory is not None):
                actor = world.get_actor(self.carla_id)

            if actor:
                state = self.trajectory.state_at_time_step(timestep)
                if state:
                    new_orientation = state.orientation
                    new_position = state.position
                    transform = carla.Transform(carla.Location(
                        x=new_position[0], y=-new_position[1], z=0),
                        carla.Rotation(yaw=(-(180 * new_orientation) / np.pi)))
                    actor.set_transform(transform)
            else:
                print("Could not fing actor")
        except Exception as e:
            print("Error while updating position")
            raise (e)

    def process_image(self, path: str, image: carla.Image):
        """
        Process image from ego RGB camera

        :param path: path to base folder, where in folder /img the images will be saved
        :param image: CARLA image to be saved
        """
        image.save_to_disk('%s/img/%.6d.jpg' % (path, image.frame))

    def destroy_carla_actor(self, world):
        """
        Destroys ego-vehicle in CARLA

        :param world: the CARLA world object
        """
        if self.is_spawned:
            for sensor in self.actor_list:
                sensor.destroy()
            actor = world.get_actor(self.carla_id)
            if actor:
                actor.destroy()
