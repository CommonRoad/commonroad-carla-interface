import carla
import weakref

from carlacr.visualization.visualization_base import VisualizationBase


class GnssSensor(VisualizationBase):
    """Manages GNSS sensor attached to ego vehicle."""

    def __init__(self, parent_actor: carla.Vehicle):
        """
        Initialization of gnss sensor.

        :param parent_actor: Parent CARLA actor.
        """
        super().__init__()  # must have for VisualizationBase inheriting __init__()

        self.sensor: carla.Sensor = None
        self._parent = parent_actor

        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))
        print(type(self.sensor))

    @staticmethod
    def _on_gnss_event(weak_self: weakref, event: carla.GnssMeasurement):
        """
        Call back function to extract GNSS data.

        :param weak_self: Weak self-reference.
        :param event: CARLA GNSS measurement.
        """
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude

    def destroy(self):
        super().destroy()
        self.sensor.stop()
        self.sensor.destroy()
