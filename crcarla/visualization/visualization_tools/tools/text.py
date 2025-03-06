from typing import TYPE_CHECKING, List, Tuple

import pygame

from crcarla.visualization.visualization_base import VisualizationBase

if TYPE_CHECKING:
    from crcarla.visualization.visualization3D import Visualization3D


class Text(VisualizationBase):
    """
    This class is generating different types of text to visualize information from the environment.
    """

    def __init__(self, vis3d: "Visualization3D", z_axis: float = 1) -> None:
        """
        Set important parameters to print text with pygame.

        :param vis3d: Base Visualization3D instance.
        :param z_axis: The z-coordinate of the visualization reference point. Defaults to 1.
        """
        super().__init__(z_axis)

        self._vis3d = vis3d  # The 3D visualization instance
        self._data: List[dict] = []

    def add_static_text_2d(
        self, text: str, x: float, y: float, size: float, lifetime: int = -1, color: Tuple[int, int, int] = (255, 0, 0)
    ):
        """
        Add text on a specific projected camera position.

        :param text: Printed text.
        :param x: x-position.
        :param y: y-position.
        :param size: Size of the static text.
        :param lifetime: Lifetime for text object. Defaults to -1.
        :param color: RGB font color. Defaults to (255, 0, 0).
        """
        self._data.append({"text": text, "x": x, "y": y, "size": size, "color": color, "lifetime": lifetime})

    def tick(self, clock: pygame.time.Clock):
        """
        Called to update the current position of the bounding boxes.

        :param clock: The game clock.
        """
        super().tick(clock)
        if not VisualizationBase.is_visible:
            return

    def render(self, display: pygame.display):
        """
        Draws the text on the display.

        :param display: The display to draw onto.
        :return: True if the visualization is successful.
        """
        super().render(display)
        if not VisualizationBase.is_visible:
            return

        dead_items = []
        for item in self._data:
            item: dict
            text = item["text"]
            pos_x = item["x"]
            pos_y = item["y"]
            size = item["size"]
            color = item["color"]
            lifetime = item["lifetime"]

            # Schriftgröße festlegen
            font = pygame.font.Font(None, size)

            # Text erstellen und rendern
            text = font.render(text, True, color)

            # Position des Textes festlegen
            text_rect = text.get_rect()

            text_rect.midbottom = (pos_x, pos_y)  # Top-left corner

            # Text auf den Bildschirm zeichnen
            display.blit(text, text_rect)

            if lifetime > 0:
                item["lifetime"] -= 1

            if lifetime == 0:
                dead_items.append(item)

        for item in dead_items:  # sorted(dead_items, reverse=True):
            self._data.remove(item)
