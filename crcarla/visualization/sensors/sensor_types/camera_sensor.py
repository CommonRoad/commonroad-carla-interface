import shutil
import weakref
from typing import TYPE_CHECKING

import carla
import numpy as np
import pygame

from crcarla.helper.config import CustomVis, CarlaParams
from crcarla.visualization.visualization_base import VisualizationBase

if TYPE_CHECKING:
    from crcarla.visualization.canvas.canvas_controller import CanvasController


class CameraSensor(VisualizationBase):
    """Manages camera sensor attached to ego vehicle."""

    def __init__(
        self,
        parent_actor: carla.Vehicle,
        config: CarlaParams,
        canvas_controller: "CanvasController",
    ):
        """
        Initialization of camera manager.

        :param parent_actor: Parent CARLA actor.
        :param hud: Head-up display object.
        :param gamma_correction: Gamma correction value for camera lens.
        :param path: Path where video should be stored.
        """
        super().__init__(z_axis=-1.0)  # must have for VisualizationBase inheriting __init__()

        # Placeholder for camera sensor and surface
        self.sensor: carla.Sensor = None
        self.surface: pygame.surface.Surface = None

        # Reference to the parent vehicle actor
        self._parent = parent_actor
        self._config = config
        self._canvas_controller = canvas_controller

        # Recording status and path for video storage
        self.recording = config.ego_view.record_video
        self.path = config.ego_view.video_path
        if (tmp_path := self.path / "_tmp").exists():
            shutil.rmtree(tmp_path)

        self._camera_transforms = self._create_camera_transforms()
        self.sensors = self._create_sensors()

        self.index = None

        self.transform_index = 0
        self.set_sensor(0, notify=False)

        self._images = []  # storage for images

    def _create_camera_transforms(self):
        """
        Creates camera transforms based on the type of the parent actor.

        :return: List of camera transforms.
        :rtype: List
        """
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z

        if not self._parent.type_id.startswith("walker.pedestrian"):
            # Camera transforms for non-pedestrian actors
            return [
                # 3rd person view
                (
                    carla.Transform(
                        carla.Location(x=-2.0 * bound_x, y=+0.0 * bound_y, z=2.0 * bound_z),
                        carla.Rotation(pitch=8.0),
                    ),
                    carla.AttachmentType.SpringArm,
                )
                if self._config.vis_type == CustomVis.THIRD_PERSON
                # driver view
                else (
                    carla.Transform(
                        carla.Location(x=-0.01 * bound_x, y=-0.3 * bound_y, z=1.0 * bound_z),
                        carla.Rotation(pitch=0.0),
                    ),
                    carla.AttachmentType.Rigid,
                ),
                (
                    carla.Transform(carla.Location(x=+0.8 * bound_x, y=+0.0 * bound_y, z=1.3 * bound_z)),
                    carla.AttachmentType.Rigid,
                ),
                (
                    carla.Transform(carla.Location(x=+1.9 * bound_x, y=+1.0 * bound_y, z=1.2 * bound_z)),
                    carla.AttachmentType.SpringArm,
                ),
                (
                    carla.Transform(
                        carla.Location(x=-2.8 * bound_x, y=+0.0 * bound_y, z=4.6 * bound_z),
                        carla.Rotation(pitch=6.0),
                    ),
                    carla.AttachmentType.SpringArm,
                ),
                (
                    carla.Transform(carla.Location(x=-1.0, y=-1.0 * bound_y, z=0.4 * bound_z)),
                    carla.AttachmentType.Rigid,
                ),
            ]

        # Camera transforms for pedestrian actors
        return [
            (
                carla.Transform(carla.Location(x=-2.5, z=0.0), carla.Rotation(pitch=-8.0)),
                carla.AttachmentType.SpringArm,
            ),
            (carla.Transform(carla.Location(x=1.6, z=1.7)), carla.AttachmentType.Rigid),
            (
                carla.Transform(carla.Location(x=2.5, y=0.5, z=0.0), carla.Rotation(pitch=-8.0)),
                carla.AttachmentType.SpringArm,
            ),
            (
                carla.Transform(carla.Location(x=-4.0, z=2.0), carla.Rotation(pitch=6.0)),
                carla.AttachmentType.SpringArm,
            ),
            (
                carla.Transform(carla.Location(x=0, y=-2.5, z=-0.0), carla.Rotation(yaw=90.0)),
                carla.AttachmentType.Rigid,
            ),
        ]

    def _create_sensors(self):
        """
        Creates sensor definitions based on the HUD and gamma correction value.

        :return: List of sensor definitions.
        :rtype: List
        """
        sensors = [
            ["sensor.camera.rgb", carla.ColorConverter.Raw, "Camera RGB", {}],
            ["sensor.camera.dvs", carla.ColorConverter.Raw, "Dynamic Vision Sensor", {}],
            [
                "sensor.camera.rgb",
                carla.ColorConverter.Raw,
                "Camera RGB Distorted",
                {
                    "lens_circle_multiplier": "3.0",
                    "lens_circle_falloff": "3.0",
                    "chromatic_aberration_intensity": "0.5",
                    "chromatic_aberration_offset": "0",
                },
            ],
        ]

        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith("sensor.camera"):
                bp.set_attribute("image_size_x", str(self._config.ego_view.width))
                bp.set_attribute("image_size_y", str(self._config.ego_view.height))
                if bp.has_attribute("gamma"):
                    bp.set_attribute("gamma", str(self._config.ego_view.gamma))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
                image_w = bp.get_attribute("image_size_x").as_int()
                image_h = bp.get_attribute("image_size_y").as_int()
                fov = bp.get_attribute("fov").as_float()
                item.append(bp)
                item.append([image_w, image_h, fov])

        return sensors

    def get_camera_intrinsics(self) -> np.ndarray:
        """Create camera intrinsics matrix."""
        w, h, fov = self.sensors[self.index][-1]

        focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
        k = np.identity(3)
        k[0, 0] = k[1, 1] = focal
        k[0, 2] = w / 2.0
        k[1, 2] = h / 2.0
        return k

    @property
    def camera_bp(self):
        """Return the blueprint of the active camera sensor."""
        return self.sensors[self.index][-2]

    def set_sensor(self, index: int, notify: bool = True, force_respawn: bool = False):
        """
        Sets camera sensor.

        :param index: Index of sensor.
        :param notify: Boolean whether notification on HUD should be shown.
        :param force_respawn: Boolean indicating whether the sensor has to be respawned.
        """
        index = index % len(self.sensors)
        needs_respawn = (
            True if self.index is None else (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        )
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            bp = self.sensors[index][-2]
            transform = self._camera_transforms[self.transform_index][0]
            attach_to = self._parent
            attachment_type = self._camera_transforms[self.transform_index][1]
            self.sensor = self._parent.get_world().spawn_actor(bp, transform, attach_to, attachment_type)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraSensor._parse_image(weak_self, image))
        if notify:
            self._canvas_controller.notify(self.sensors[index][2])
        self.index = index

    def toggle_recording(self):
        """Activates/deactivates camera recording."""
        self.recording = not self.recording

    def render(self, display: pygame.display):
        """
        Renders camera.

        :param display: Pygame display used for rendering.
        """
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self: weakref, image: carla.Image):
        """
        Parses camera image and stores it if recording is activated.

        :param weak_self: Weak self-reference.
        :param image: CARLA image.
        """
        self: "CameraSensor" = weak_self()

        if not self:
            return
        if self.sensors[self.index][0].startswith("sensor.camera.dvs"):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(
                image.raw_data,
                dtype=np.dtype([("x", np.uint16), ("y", np.uint16), ("t", np.int64), ("pol", np.bool)]),
            )
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]["y"], dvs_events[:]["x"], dvs_events[:]["pol"] * 2] = 255
            self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            if self.recording:
                # pylint: disable=protected-access
                self._images.append(image)

    # if self.recording:
    # image.save_to_disk(f'{self.path}/_tmp/%08d' % image.frame)

    def destroy(self):
        if self.recording:
            for image in self._images:
                image.save_to_disk(f"{self.path}/_tmp/%08d" % image.frame)

        super().destroy()
        self.sensor.stop()
        self.sensor.destroy()

        self.sensor = None
        self.index = None
