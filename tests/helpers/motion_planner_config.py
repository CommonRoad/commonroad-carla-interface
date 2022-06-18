import unittest
from copy import deepcopy
from typing import Union

from commonroad.scenario.trajectory import Trajectory

try:
    from motion_planner.motion_planner import MotionPlanner
    from motion_planner_config.configuration import Configuration
except:
    raise ImportError('Can not find MotionPlanner and motion planner config')


class MotionPlannerConfig:

    @classmethod
    def get_motion_planner_config(cls: Configuration) -> Union[Trajectory, None]:
        mp = None
        mp = MotionPlanner(cls)
        return mp
