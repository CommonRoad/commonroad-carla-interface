import logging
from carla import Client
import pygame

# from examples.manual_control import KeyboardControl, HUD, World
from carlacr.carla_org.manual_control import KeyboardControl, HUD, World

from carlacr.helper.config import ManualControlParams

logger = logging.getLogger(__name__)

def manual_keyboard_control(client: Client, config: ManualControlParams):
    pygame.init()
    pygame.font.init()
    world = None
    try:
        sim_world = client.get_world()

        display = pygame.display.set_mode((config.width, config.height), pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0, 0, 0))
        pygame.display.flip()

        hud = HUD(config.width, config.height)
        world = World(sim_world, hud, config)
        controller = KeyboardControl(world, config.autopilot)

        if config.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        clock = pygame.time.Clock()
        while True:
            if config.sync:
                sim_world.tick()
            clock.tick_busy_loop(60)
            if controller.parse_events(client, world, clock, config.sync):
                return
            world.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:
        if world and world.recording_enabled:
            client.stop_recorder()

        if world is not None:
            world.destroy()

        pygame.quit()