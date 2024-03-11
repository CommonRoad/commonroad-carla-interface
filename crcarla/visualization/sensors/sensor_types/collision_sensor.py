import collections
import math
import weakref
from typing import TYPE_CHECKING, Dict

import carla

from crcarla.visualization.common import get_actor_display_name
from crcarla.visualization.visualization_base import VisualizationBase

if TYPE_CHECKING:
    from crcarla.visualization.canvas.canvas_controller import CanvasController


class CollisionSensor(VisualizationBase):
    """Manages GNSS sensor attached to ego vehicle."""

    def __init__(self, parent_actor: carla.Vehicle, canvas_controller: "CanvasController"):
        """
        Initialization of collision sensor.

        :param parent_actor: Parent CARLA actor.
        :param hud: Head-up display object.
        """
        super().__init__()  # must have for VisualizationBase inheriting __init__()

        self.sensor: carla.Sensor = None

        self.history = []
        self._parent = parent_actor
        self._canvas_controller = canvas_controller

        world = self._parent.get_world()
        bp = world.get_blueprint_library().find("sensor.other.collision")
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self) -> Dict[int, float]:
        """
        Extracts collision intensity for each frame.

        :return: Mapping of frame ID to collision intensity.
        """
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self: weakref, event: carla.CollisionEvent):
        """
        Call back function to extract collision data.

        :param weak_self: Weak self-reference.
        :param event: CARLA CollisionEvent measurement.
        """
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self._canvas_controller.notify(f"Collision with {actor_type}")  # pylint: disable=protected-access
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)

    def destroy(self):
        super().destroy()
        self.sensor.stop()
        self.sensor.destroy()
