from typing import TYPE_CHECKING, List

import carla
import pygame
import shapely

from crcarla.visualization.visualization_base import VisualizationBase

if TYPE_CHECKING:
    from crcarla.visualization.visualization3D import Visualization3D


class TrajectoryTool(VisualizationBase):
    """
    A class to visualize trajectories in a 3D world.
    """

    def __init__(self, vis3d: "Visualization3D", z_axis: float = 1) -> None:
        """
        Initializes an instance of TrajectoryTool.

        :param vis3D: A reference to the 3D visualization instance.
        :type vis3D: Visualization3D
        :param z_axis: The z-coordinate of the visualization reference point.
        :type z_axis: float
        """
        super().__init__(z_axis)

        self._vis3d = vis3d  # The 3D visualization instance
        self._config = vis3d.config.visualization

        self._lines: List[dict] = []

    def tick(self, clock: pygame.time.Clock):
        """
        Called to update the current trajectories.

        :param clock: The game clock.
        :type clock: pygame.time.Clock
        """
        super().tick(clock)
        if not VisualizationBase.is_visible or len(self._vis3d.trajectories) == 0:
            return
        ego_vehicle = self._vis3d.ego_vehicle
        ego_location = ego_vehicle.get_location()
        ego_image_point = self._vis3d.vis_tool_controller.get_image_point(ego_location)

        trajectories = [self._vis3d.trajectories[0]]
        num_infeasible_trajectories = 0
        if not self._config.trajectory_vis.ONLY_OPTIMAL:
            if self._config.trajectory_vis.ONLY_FEASABLE:
                trajectories += self._vis3d.trajectories[1][
                    : len(self._vis3d.trajectories[1]) - self._vis3d.trajectories[2]
                ]
                num_infeasible_trajectories = 0
            else:
                trajectories = [self._vis3d.trajectories[0]] + self._vis3d.trajectories[1]
                num_infeasible_trajectories = self._vis3d.trajectories[2]

        trajectories.reverse()

        for i in range(len(trajectories)):
            last = ego_image_point
            for state in trajectories[i].state_list:
                traj_point = self._vis3d.vis_tool_controller.get_image_point(
                    carla.Location(state.position[0], -state.position[1], ego_location.z)
                )
                color = "red" if i < num_infeasible_trajectories else "green" if i < len(trajectories) - 1 else "black"

                self._lines.append(
                    {
                        "start": last,
                        "end": traj_point,
                        "color": color,
                    }
                )
                last = traj_point

    def render(self, display: pygame.display):
        """
        Draws the trajectories on the display.

        :param display: The display to draw onto.
        :type display: pygame.display
        """
        super().render(display)
        if not VisualizationBase.is_visible:
            return
        for target in self._lines:
            color = target["color"]
            start = target["start"]
            end = target["end"]

            line = shapely.LineString([start, end])
            pygame_linie = list(line.coords)
            pygame.draw.lines(display, color, False, pygame_linie, 3)

        self._lines.clear()
