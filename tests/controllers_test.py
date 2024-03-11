import os

from commonroad.common.file_reader import CommonRoadFileReader
from utils import make_output_dir

from crcarla.helper.config import PedestrianControlType, VehicleControlType
from crcarla.helper.utils import (
    render_obs_trajectory_and_speed_comparisons,
    render_trajectory_video,
)
from tests.base_test_case.carla_server_test_case import CarlaServerTestCase

CONTROLLERS = [VehicleControlType.PID]

PEDESTRIAN_CONTROLLER = PedestrianControlType.AI

SCENARIOS = ["DEU_Test-1_1_T-2.xml"]
MAPS = ["DEU_Test-1_1_T-1.xodr"]


class ControllerTestCase(CarlaServerTestCase):
    def __init__(self, method_name: str = "runTest") -> None:
        super().__init__(method_name)

        current_dir = os.getcwd()
        parent_dir = os.path.dirname(current_dir)

        scenarios_dir = os.path.join(parent_dir, "scenarios")
        maps_dir = os.path.join(parent_dir, "maps")

        self.scenario_files = [os.path.join(scenarios_dir, s) for s in SCENARIOS]
        self.map_files = [os.path.join(maps_dir, s) for s in MAPS]

    @classmethod
    def setUpClass(self):
        super().setUpClass()

        self.basic_cases = []
        self.edge_cases = []

        self.out_path = make_output_dir()

    def get_image_file_path(self, controller, scenario_files):
        base_name = os.path.splitext(os.path.basename(scenario_files))[0]
        return os.path.join(self.out_path, f"{str(controller)}_{base_name}")

    def get_video_file_path(self, controller, scenario_files):
        base_name = os.path.splitext(os.path.basename(scenario_files))[0]
        return os.path.join(self.out_path, f"{str(controller)}_{base_name}")

    def test_controller(self):
        for controler in CONTROLLERS:
            for sc, m in zip(self.scenario_files, self.map_files):
                self.param.map = m
                self.param.vehicle.carla_controller_type = controler
                self.param.pedestrian.carla_controller_type = PEDESTRIAN_CONTROLLER
                self.param.sync = True

                self.reset_server()

                scenario, pps = CommonRoadFileReader(sc).open()
                self.ci.replay(scenario)

                render_trajectory_video(
                    scenario,
                    self.ci.get_simulation_fps(),
                    self.ci._cr_obstacles,
                    self.get_video_file_path(controler, sc),
                    str(controler),
                )

                render_obs_trajectory_and_speed_comparisons(
                    self.ci._cr_obstacles,
                    self.get_image_file_path(controler, sc),
                    str(controler),
                )

    def test_random_cases(self):
        pass
