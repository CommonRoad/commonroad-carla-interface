import logging
from pathlib import Path

from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.writer.file_writer_interface import OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Tag
from crdesigner.verification_repairing.config import MapVerParams
from crdesigner.verification_repairing.map_verification_repairing import verify_and_repair_map
from crdesigner.verification_repairing.verification.formula_ids import LaneletFormulaID

from crcarla.carla_interface import CarlaInterface
from crcarla.helper.config import CarlaParams

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

param = CarlaParams()
ci = CarlaInterface(param)

maps = ["Town10HD", "Town06"]
for carla_map in maps:
    param.map = carla_map

    ci.update_config(param)

    scenario = ci.create_cr_map()
    scenario.author = "Sebastian Maierhofer"
    scenario.affiliation = "Technical University of Munich"
    scenario.source = "CARLA/CommonRoad Scenario Designer"
    scenario.tags = {Tag.URBAN}
    scenario.scenario_id.map_name = f"CARLA{param.map}"
    formulas = [
        LaneletFormulaID.EXISTENCE_RIGHT_ADJ,
        LaneletFormulaID.EXISTENCE_LEFT_ADJ,
        LaneletFormulaID.EXISTENCE_PREDECESSOR,
        LaneletFormulaID.EXISTENCE_SUCCESSOR,
        LaneletFormulaID.NON_PREDECESSOR_AS_SUCCESSOR,
        LaneletFormulaID.NON_SUCCESSOR_AS_PREDECESSOR,
        LaneletFormulaID.POLYLINES_INTERSECTION,
        LaneletFormulaID.RIGHT_SELF_INTERSECTION,
        LaneletFormulaID.LEFT_SELF_INTERSECTION,
        LaneletFormulaID.POTENTIAL_SUCCESSOR,
        LaneletFormulaID.POTENTIAL_PREDECESSOR,
        LaneletFormulaID.POTENTIAL_RIGHT_MERGING_ADJ,
        LaneletFormulaID.POTENTIAL_LEFT_MERGING_ADJ,
        LaneletFormulaID.POTENTIAL_RIGHT_SAME_DIR_PARALLEL_ADJ,
        LaneletFormulaID.POTENTIAL_LEFT_SAME_DIR_PARALLEL_ADJ,
        LaneletFormulaID.POTENTIAL_RIGHT_OPPOSITE_DIR_PARALLEL_ADJ,
        LaneletFormulaID.POTENTIAL_LEFT_OPPOSITE_DIR_PARALLEL_ADJ,
        LaneletFormulaID.POTENTIAL_LEFT_FORKING_ADJ,
        LaneletFormulaID.POTENTIAL_RIGHT_FORKING_ADJ,
    ]
    config = MapVerParams()
    config.verification.formulas = formulas
    scenario.replace_lanelet_network(verify_and_repair_map(scenario.lanelet_network, config)[0])
    CommonRoadFileWriter(scenario, planning_problem_set=PlanningProblemSet()).write_to_file(
        filename=str(Path(__file__).parent.parent / f"scenarios/{scenario.scenario_id}.xml"),
        overwrite_existing_file=OverwriteExistingFile.ALWAYS,
    )
