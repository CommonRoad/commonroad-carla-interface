import math
import sys
from typing import Tuple, List

import carla
import pygame
import pygame.locals as keys

COLOR_WHITE = pygame.Color(255, 255, 255)
COLOR_BLACK = pygame.Color(0, 0, 0)


def is_quit_shortcut(key: pygame.key):
    """
    Returns True if one of the specified keys are pressed

    :param key: Returns true if key equals keys for quitting.
    """
    return (key == keys.K_ESCAPE) or (key == keys.K_q and pygame.key.get_mods() & keys.KMOD_CTRL)


def exit_game():
    """Shuts down program and PyGame"""
    pygame.quit()
    sys.exit()


def get_actor_display_name(actor: carla.Actor, truncate: int = 250) -> str:
    """
    Extracts name of actor which should be displayed in window.

    :param actor: CARLA actor.
    :param truncate: Maximum length name can have.
    :return: Name of actor.
    """
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + '\u2026') if len(name) > truncate else name


def sort_vehicles_by_dist(vehicles: carla.ActorList,
                          actor: carla.Actor,
                          max_dist: float = 200.0) -> List[Tuple[carla.Vehicle, float]]:
    """
    Returns a list of vehicles sorted by distance from a given actor.

    :param actor: Actor to measure distance from.
    :param max_dist: Maximum distance to consider. 0 for all vehicles.
    :return: List of tuples with distance and actor.
    """
    actor_t = actor.get_transform()

    def distance(location: carla.Location):
        return math.sqrt((location.x - actor_t.location.x) ** 2 + (location.y - actor_t.location.y) ** 2 + (
            location.z - actor_t.location.z) ** 2)

    vehicles: carla.ActorList = [(x, distance(x.get_location())) for x in vehicles if x.id != actor.id]
    if len(vehicles) == 0:
        return None
    vehicles = sorted(vehicles, key=lambda vehicles: vehicles[1])
    if max_dist > 0.0:
        for i, (_, d) in enumerate(vehicles):
            if d > max_dist:
                vehicles = vehicles[:i]
                break
    return vehicles
