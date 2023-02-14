import logging
import carla

from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import State

from carlacr.helper.vehicle_dict import (similar_by_area, similar_by_length, similar_by_width)
from carlacr.helper.config import ObstacleParams, ApproximationType
from carlacr.interface.obstacle_interface import ObstacleInterface

logger = logging.getLogger(__name__)


class VehicleInterface(ObstacleInterface):
    """One to one representation of a CommonRoad obstacle to be worked with in CARLA."""

    def __init__(self, cr_obstacle: DynamicObstacle):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        super().__init__(cr_obstacle)

    def spawn(self, world: carla.World, config: ObstacleParams) -> carla.Actor:
        """
        Tries to spawn the vehicle (incl. lights if supported) in the given CARLA world and returns the spawned vehicle.

        :param world: the CARLA world object
        :param config: obstacle-related configuration
        :param physics: if physics should be enabled for the vehicle
        :return: if spawn successful the according CARLA actor else None
        """
        transform = self.transform(self.cr_base.initial_state)

        if self.cr_base.obstacle_type in [ObstacleType.CAR, ObstacleType.TRUCK, ObstacleType.BUS, ObstacleType.PRIORITY_VEHICLE,
                         ObstacleType.PARKED_VEHICLE, ObstacleType.MOTORCYCLE, ObstacleType.TAXI]:
            nearest_vehicle_type = None
            if config.approximation_type == ApproximationType.LENGTH:
                nearest_vehicle_type = similar_by_length(self.cr_base.obstacle_shape.length, self.cr_base.obstacle_shape.width, 0)
            if config.approximation_type == ApproximationType.WIDTH:
                nearest_vehicle_type = similar_by_width(self.cr_base.obstacle_shape.length, self.cr_base.obstacle_shape.width, 0)
            if config.approximation_type == ApproximationType.AREA:
                nearest_vehicle_type = similar_by_area(self.cr_base.obstacle_shape.length, self.cr_base.obstacle_shape.width, 0)
            obstacle_blueprint = world.get_blueprint_library().filter(nearest_vehicle_type[0])[0]

            try:
                obstacle = world.try_spawn_actor(obstacle_blueprint, transform)
                if obstacle:
                    obstacle.set_simulate_physics(config.physics)
                    logger.debug("Spawn successful: CR-ID %s CARLA-ID %s", self.commonroad_id, obstacle.id)
                    # Set up the lights to initial states:
                    vehicle = world.get_actor(obstacle.id)
                    if self.cr_base.initial_signal_state:
                        if vehicle:
                            sig = self.cr_base.initial_signal_state
                            self._set_up_lights(vehicle=vehicle, sig=sig)
                    self.carla_id = obstacle.id
                    self.is_spawned = True
            except Exception as e:
                logger.error(f"Error while spawning VEHICLE: {e}")
                raise e

    def _set_up_lights(self, vehicle, state: State = None, sig=None):
        """
        Sets up the lights of the Obstacle.

        :param state: state at the time step
        :param sig: signals of the Obstacle
        """
        # vehicle = world.get_actor(self.carla_id)
        z = carla.VehicleLightState.NONE
        vehicle.set_light_state(z)
        if sig is None and state is not None:
            sig = self.cr_base.signal_state_at_time_step(state.time_step)
        if sig:
            if sig.braking_lights:
                z = z | carla.VehicleLightState.Brake
            if sig.indicator_left and not sig.hazard_warning_lights:
                z = z | carla.VehicleLightState.LeftBlinker
            if sig.indicator_right and not sig.hazard_warning_lights:
                z = z | carla.VehicleLightState.RightBlinker
            if sig.hazard_warning_lights:
                z = z | carla.VehicleLightState.RightBlinker
                z = z | carla.VehicleLightState.LeftBlinker
            vehicle.set_light_state(carla.VehicleLightState(z))
