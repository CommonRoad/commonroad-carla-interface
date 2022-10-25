from typing import Tuple, List
import sys
import logging
from math import sqrt
import carla
import numpy as np
from commonroad.scenario.obstacle import Obstacle
from commonroad.scenario.trajectory import Trajectory
from carlacr.interface.commonroad_obstacle_interface import ApproximationType
from carlacr.helper.vehicle_dict import (similar_by_area, similar_by_length,
                                         similar_by_width)
from commonroad.scenario.state import CustomState as State
logger = logging.getLogger(__name__)


class CommonRoadEgoInterface:
    """
    Creates and controls the ego-vehicle in CARLA
    """

    def __init__(self, client: carla.Client, trajectory: Trajectory,
                 initial_state: Obstacle.initial_state, size: Tuple[float, float, float] = None):
        """
        :param trajectory: CommonRoad trajectory for the ego-vehicle
        :param initial_state: initial_state of commonroad obstacle when commonroad obstacle used as ego vehicle
        """
        self.init_state = initial_state
        self.trajectory = trajectory
        self.is_spawned = False
        self.carla_id = None
        self.client = client
        self.spawn_timestep = initial_state.time_step
        self.spawn_timestep = initial_state.time_step
        self.actor_list: List[int] = []
        self.size = size

    def spawn(self, world: carla.World, physics=True,
              approx_type=ApproximationType.LENGTH) -> carla.Actor:
        """
        Tries to spawn the ego-vehicle and a camera for it in the given CARLA world and returns the spawned vehicle.

        :param world: the CARLA world object
        :param physics: if physics should be enabled for the ego-vehicle
        :param create_gif: True if a GIF should be created
        :param path: base path of the directory where the GIF should be stored
        :param approx_type:based on what approximation of the vehicle size the blue print should be selected
        :return: if spawn successful the according CARLA actor else None
        """
        ego_transform = carla.Transform(
            carla.Location(x=self.init_state.position[0], y=-self.init_state.position[1], z=0.5),
            carla.Rotation(yaw=(-(180 * self.init_state.orientation) / np.pi)))
        if self.size:
            if approx_type == ApproximationType.LENGTH:
                nearest_vehicle_type = similar_by_length(self.size[0], self.size[1], 0)
            if approx_type == ApproximationType.WIDTH:
                nearest_vehicle_type = similar_by_width(self.size[0], self.size[1], 0)
            if approx_type == ApproximationType.AREA:
                nearest_vehicle_type = similar_by_area(self.size[0], self.size[1], 0)
            ego_blueprint = world.get_blueprint_library().filter(nearest_vehicle_type[0])[0]  # Just for reference
        else:
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
            logger.debug("Ego spawn successful")
            return ego

        except Exception as e:
            logger.error("Error while spawning:")
            raise e

    def set_trajectory(self, trajectory: Trajectory):
        """
        Sets trajectory for the ego-vehicle

        :param trajectory: CommonRoad trajectory for the ego-vehicle
        """
        if trajectory is not None:
            self.trajectory = trajectory
            logger.debug("Ego-Vehicle Trajectory is set")
        else:
            logger.error("Invalid Trajectory")

    def update_position_by_time(self, world: carla.World, state: State):
        """
        Tries to update the position of the ego-vehicle

        :param world: the CARLA world object
        :param state: state at the time step
        """
        try:
            if self.is_spawned and (self.trajectory is not None):
                actor = world.get_actor(self.carla_id)

            if actor:
                new_orientation = state.orientation
                new_position = state.position
                transform = carla.Transform(carla.Location(
                    x=new_position[0], y=-new_position[1], z=actor.get_location().z),
                    carla.Rotation(yaw=(-(180 * new_orientation) / np.pi)))
                actor.set_transform(transform)
                # set_target_velocity, set_target_angular_velocity

            else:
                logger.debug("Could not find actor")
        except Exception as e:
            logger.error("Error while updating position")
            raise e

    def process_image(self, path: str, image: carla.Image):
        """
        Process image from ego RGB camera

        :param path: path to base folder, where in folder /img the images will be saved
        :param image: CARLA image to be saved
        """
        image.save_to_disk('%s/img/%.6d.jpg', (path, image.frame))

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

    def get_cr_state(self, time_step=0) -> State:
        """
        Get current CommonRoad state if spawned, else None

        :return: CommonRoad state of the CARLA vehicle represented by this class
        """
        if self.carla_id:
            try:
                actor = self.client.get_world().get_actor(self.carla_id)
                vel_vec = actor.get_velocity()
                vel = sqrt(vel_vec.x ** 2 + vel_vec.y ** 2)  # velocity
                transform = actor.get_transform()
                location = transform.location
                rotation = transform.rotation
                state = State(position=np.array([location.x, -location.y]), orientation=-((rotation.yaw * np.pi) / 180),
                              velocity=vel, time_step=time_step)
                return state
            except Exception as e:
                logger.debug("Following error occured while retrieving current position for:")
                logger.debug(self)
                logger.error(e, exc_info=sys.exc_info())
                return None
        else:
            return None
