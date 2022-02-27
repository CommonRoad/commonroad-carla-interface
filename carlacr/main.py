import typer
import time
from carlacr.mode.carla_replay_mode import CarlaReplayMode
import logging
logger = logging.getLogger(__name__)

cli = typer.Typer(
    help="Generates visualization in Carla for Commonroad scenario and map"
)


@cli.command()
def visualize(scenario_file: str = typer.Argument(
    ...,
    help="Path to scenario file"
),
        map_file: str = typer.Argument(
            ...,
            help="Path to map file"
        ),
        veh_id: int = typer.Option(
            -1,
            help="id of the vehicle"
        ),
        saving_video: str = typer.Option(
            None,
            help="path to save the video file"
        ),
        video_name: str = typer.Option(
            None,
            help="name to video file"
        ),
        asMP4: bool = typer.Option(
            False,
            help="as MP4 or gif"
        )):
    startTime = time.time()
    replay = CarlaReplayMode(cr_scenario_path=scenario_file, open_drive_map_path=map_file)

    if veh_id != -1:
        replay.set_ego_vehicle_by_id(veh_id)

    if saving_video:
        replay.visualize(sleep_time=5, saving_video=True, video_path=saving_video, file_name=video_name, asMP4=asMP4)
    else:
        replay.visualize(leep_time=5, saving_video=False, video_path="/home/")
    executionTime = (time.time() - startTime)

    logger.debug('Execution time in seconds: ' + str(executionTime))


if __name__ == "__main__":
    cli()
