from typing import Tuple

import pygame

from crcarla.visualization.canvas.ui_elements.text import COLOR_WHITE, Text


class FadingText(Text):
    """Renders texts that fades out after some ms that the user specifies"""

    def __init__(self, font: pygame.font.Font, dim: Tuple[int, int], pos: Tuple[int, int], z_axis: float = 1) -> None:
        """
        Initializes variables such as text font, dimensions and position

        :param font: Font type used for fading text.
        :param dim: Dimensions of fading text.
        :param pos: Position of fading text.
        """
        super().__init__(font, dim, pos, z_axis)  # must have for VisualizationBase inheriting __init__()
        self._ms_duration = 0
        self._ms_left = 0

    def set_text(self, text: str, color: pygame.color.Color = COLOR_WHITE, t_ms=2000):
        """
        Sets the text, color and seconds until fade out

        :param text: Text which should be displayed.
        :param color: Color in which the text should be displayed.
        :param t_ms: Time how long text should be displayed.
        """
        super().set_text(text, color)
        self._ms_left = t_ms
        self._ms_duration = t_ms

    def tick(self, clock: pygame.time.Clock):
        """
        Each frame, it shows the displayed text for some specified ms, if any.

        :param clock: Pygame clock.
        """
        super().tick(clock)

        delta_ms = clock.get_time()
        self._ms_left = max(0, self._ms_left - delta_ms)
        alpha = float(self._ms_left) / float(self._ms_duration) * 255.0
        self._surface.set_alpha(int(alpha))
