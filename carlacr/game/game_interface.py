import logging
from carla import Client
import pygame

from carlacr.game.birds_eye_view import HUD2D
from carlacr.game.birds_eye_view import World2D
from carlacr.interface.control_interface import KeyBoardControl2D


from carlacr.helper.config import ManualControlParams

logger = logging.getLogger(__name__)

COLOR_ALUMINIUM_4 = pygame.Color(85, 87, 83)
COLOR_WHITE = pygame.Color(255, 255, 255)

def manual_keyboard_control(client: Client, config: ManualControlParams):
    logger.info("Start keyboard manual control.")
    pygame.init()
    pygame.font.init()
    world = None
    try:
        sim_world = client.get_world()

        display = pygame.display.set_mode((config.width, config.height), pygame.HWSURFACE | pygame.DOUBLEBUF)

        pygame.display.set_caption(config.description)  # Place a title to game window

        # Show loading screen
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        text_surface = font.render('Rendering map...', True, COLOR_WHITE)
        display.blit(text_surface, text_surface.get_rect(center=(config.width / 2, config.height / 2)))
        display.fill((0, 0, 0))
        pygame.display.flip()

        if config.birds_eye_view:
            logger.info("Init 2D.")
            hud = HUD2D("CARLA 2D", config.width, config.height)
            world = World2D("CARLA 2D", config)
            controller = KeyBoardControl2D("2D Manual Control")

            # For each module, assign other modules that are going to be used inside that module
            logger.info("Register 2D.")
            controller.start(hud, world)
            hud.start()
            world.start(hud, sim_world)

            # Game loop
            clock = pygame.time.Clock()
            logger.info("Loop 2D.")
            while True:
                clock.tick_busy_loop(60)

                # Tick all modules
                world.tick(clock)
                hud.tick(clock)
                controller.tick(clock)

                # Render all modules
                display.fill(COLOR_ALUMINIUM_4)
                world.render(display)
                hud.render(display)

                pygame.display.flip()


    finally:
        if world is not None:
            world.destroy()