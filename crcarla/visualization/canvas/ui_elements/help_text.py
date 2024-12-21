import pygame

from crcarla.visualization.canvas.ui_elements.text import COLOR_BLACK, COLOR_WHITE, Text


class HelpText(Text):
    """Renders the help text that shows the controls for using no rendering mode."""

    def __init__(self, font: pygame.font.Font, width: int, height: int, z_axis: float = 1) -> None:
        """
        Initialize help text.

        :param: font: Font type of help text.
        :param width: Width of pygame window [px] (used to position text)
        :param height: Height of pygame window [px] (used to position text)
        """
        self._lines = self.__doc__.split("\n")
        dim = (680, len(self._lines) * 22 + 12)
        pos = (0.5 * width - 0.5 * dim[0], 0.5 * height - 0.5 * dim[1])

        super().__init__(font, dim, pos, z_axis)  # must have for VisualizationBase inheriting __init__()

        self.set_text()

    def set_text(self, color: pygame.color.Color = COLOR_WHITE):  # pylint: disable=arguments-differ
        self._surface = pygame.Surface(self.dim)
        self._surface.fill(COLOR_BLACK)

        for n, line in enumerate(self._lines):
            text_texture = self.font.render(line, True, color)
            self._surface.blit(text_texture, (22, n * 22))

        self._surface.set_alpha(220)
