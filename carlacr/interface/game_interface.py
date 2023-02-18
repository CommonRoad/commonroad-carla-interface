import logging
from carla import World, Client
import pygame

from examples.manual_control import KeyboardControl, HUD

from carlacr.helper.config import ManualControlParams

logger = logging.getLogger(__name__)

def manual_keyboard_control(client: Client, config: ManualControlParams):
    pygame.init()
    pygame.font.init()
    world = None
    sim_world = None
    original_settings = None

    try:
        sim_world = client.get_world()
        # if args.sync:
        #     original_settings = sim_world.get_settings()
        #     settings = sim_world.get_settings()
        #     if not settings.synchronous_mode:
        #         settings.synchronous_mode = True
        #         settings.fixed_delta_seconds = 0.05
        #     sim_world.apply_settings(settings)
        #
        #     traffic_manager = client.get_trafficmanager()
        #     traffic_manager.set_synchronous_mode(True)
        #
        # if args.autopilot and not sim_world.get_settings().synchronous_mode:
        #     print("WARNING: You are currently in asynchronous mode and could "
        #           "experience some issues with the traffic simulation")

        display = pygame.display.set_mode((config.width, config.height), pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0, 0, 0))
        pygame.display.flip()

        hud = HUD(config.width, config.height)
        world = World(sim_world, hud)
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

        if original_settings:
            sim_world.apply_settings(original_settings)

        if world and world.recording_enabled:
            client.stop_recorder()

        if world is not None:
            world.destroy()

        pygame.quit()