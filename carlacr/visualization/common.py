import pygame
from typing import Tuple
import carla
import sys
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


class FadingText:
    """Renders texts that fades out after some seconds that the user specifies"""

    def __init__(self, font: pygame.font.Font, dim: Tuple[int, int], pos: Tuple[int, int]):
        """
        Initializes variables such as text font, dimensions and position

        :param font: Font type used for fading text.
        :param dim: Dimensions of fading text.
        :param pos: Position of fading text.
        """
        self.font = font
        self.dim = dim
        self.pos = pos
        self._seconds_left = 0
        self._surface = pygame.Surface(self.dim)

    def set_text(self, text: str, color: pygame.color = COLOR_WHITE, seconds=2.0):
        """
        Sets the text, color and seconds until fade out

        :param text: Text which should be displayed.
        :param color: Color in which the text should be displayed.
        :param seconds: Time how long text should be displayed.
        """
        text_texture = self.font.render(text, True, color)
        self._surface = pygame.Surface(self.dim)
        self._seconds_left = seconds
        self._surface.fill(COLOR_BLACK)
        self._surface.blit(text_texture, (10, 11))

    def tick(self, clock: pygame.time.Clock):
        """
        Each frame, it shows the displayed text for some specified seconds, if any.

        :param clock: Pygame clock.
        """
        delta_seconds = 1e-3 * clock.get_time()
        self._seconds_left = max(0.0, self._seconds_left - delta_seconds)
        self._surface.set_alpha(int(500.0 * self._seconds_left))

    def render(self, display: pygame.display):
        """
        Renders the text in its surface and its position

        :param display: Pygame display for visualization.
        """
        display.blit(self._surface, self.pos)


class HelpText:
    """Renders the help text that shows the controls for using no rendering mode."""

    def __init__(self, font: pygame.font.Font, width: int, height: int):
        """
        Initialization of help text.

        :param: font: Font type of help text.
        :param width: Width of pygame window [px] (used to position text)
        :param height: Height of pygame window [px] (used to position text)
        """
        lines = self.__doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self._seconds_left = 0
        self._surface = pygame.Surface(self.dim)
        self._surface.fill(COLOR_BLACK)
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, COLOR_WHITE)
            self._surface.blit(text_texture, (22, n * 22))
            self._render = False
        self._surface.set_alpha(220)

    def toggle(self):
        """Toggles display of help text"""
        self._render = not self._render

    def render(self, display: pygame.display):
        """
        Renders the help text, if enabled

        :param display: Pygame display for visualization.
        """
        if self._render:
            display.blit(self._surface, self.pos)
