import os
from typing import Tuple

import pygame

from carlacr.visualization.visualization_base import VisualizationBase

COLOR_WHITE = pygame.Color(255, 255, 255)
COLOR_BLACK = pygame.Color(0, 0, 0)
COLOR_RED = pygame.Color(255, 0, 0)


class Text(VisualizationBase):
    """This text ui-element can be used to create a text inside the canvas."""

    Font_ubuntu = None
    Font_mono = None
    Font_mono_16 = None

    def __init__(self, font: pygame.font.Font, dim: Tuple[int, int], pos: Tuple[int, int], z_axis: float = 1) -> None:
        """Initialize the canvas ui-element text."""
        super().__init__(z_axis)  # must have for VisualizationBase inheriting __init__()
        self.font = font
        self.dim = dim
        self.pos = pos

        self.active = True
        self._surface = pygame.Surface(self.dim)
        self._text = ""

    @classmethod
    def font_ubuntu(cls):
        """Create pygame font classic ubuntu."""
        if cls.Font_ubuntu is None:
            cls.Font_ubuntu = pygame.font.Font(pygame.font.get_default_font(), 20)
        return cls.Font_ubuntu

    @classmethod
    def font_mono(cls):
        """Create pygame font classic mono."""
        if cls.Font_mono is None:
            font_name = "courier" if os.name == "nt" else "mono"
            fonts = [x for x in pygame.font.get_fonts() if font_name in x]
            default_font = "ubuntumono"
            mono = default_font if default_font in fonts else fonts[0]
            mono = pygame.font.match_font(mono)
            cls.Font_mono = pygame.font.Font(mono, 12 if os.name == "nt" else 14)
            cls.Font_mono_16 = pygame.font.Font(mono, 16)
        return cls.Font_mono

    @classmethod
    def font_mono_16(cls):
        """Create pygame font mono 16."""
        if cls.Font_mono is None:
            font_name = "courier" if os.name == "nt" else "mono"
            fonts = [x for x in pygame.font.get_fonts() if font_name in x]
            default_font = "ubuntumono"
            mono = default_font if default_font in fonts else fonts[0]
            mono = pygame.font.match_font(mono)
            cls.Font_mono = pygame.font.Font(mono, 12 if os.name == "nt" else 14)
            cls.Font_mono_16 = pygame.font.Font(mono, 16)
        return cls.Font_mono_16

    def set_active(self, active: bool):
        """Turn ui-elment on or off."""
        self.active = active

    def set_text(self, text: str, color: pygame.color.Color = COLOR_WHITE):
        """
        Sets the text, color and seconds until fade out

        :param text: Text which should be displayed.
        :param color: Color in which the text should be displayed.
        :param seconds: Time how long text should be displayed.
        """
        text_texture = self.font.render(text, True, color)
        self._surface = pygame.Surface(self.dim)
        self._surface.fill(COLOR_BLACK)

        self._surface.blit(text_texture, (10, 11))

    def render(self, display: pygame.surface.Surface):
        """
        Renders the text in its surface and its position

        :param display: Pygame display for visualization.
        """
        super().render(display)
        if self.active:
            display.blit(self._surface, self.pos)
