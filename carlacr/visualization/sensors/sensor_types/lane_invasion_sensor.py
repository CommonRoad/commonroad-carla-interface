import carla
import weakref

from carlacr.visualization.visualization_base import VisualizationBase

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from carlacr.visualization.canvas.canvas_controller import CanvasController


class LaneInvasionSensor(VisualizationBase):
    """Manages lane invasion sensor attached to ego vehicle."""

    def __init__(self, parent_actor: carla.Vehicle, canvas_controller: 'CanvasController'):
        """
        Initialization of lane invasion sensor.

        :param parent_actor: Parent CARLA actor.
        :param hud: Head-up display object.
        """
        super().__init__()  # must have for VisualizationBase inheriting __init__()

        self.sensor: carla.Sensor = None
        self._canvas_controller = canvas_controller
        self._parent = parent_actor

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid circular
            # reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self: weakref, event: carla.LaneInvasionSensor):
        """
        Call back function to extract GNSS data.

        :param weak_self: Weak self-reference.
        :param event: CARLA LaneInvasionSensor measurement.
        """
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = [f"{str(x).split()[-1]}" for x in lane_types]
        self._canvas_controller.notify(f"Crossed line and {text}")  # pylint: disable=protected-access

    def destroy(self):
        super().destroy()
        self.sensor.stop()
        self.sensor.destroy()
