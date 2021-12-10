import os
import time
import typer
import time
from carlacr.carla_replay_mode import CarlaReplayMode

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
        )):
    startTime = time.time()
    replay = CarlaReplayMode(scenario_file, map_file)

    if veh_id != -1:
        replay.set_ego_vehicle(replay.obstacle_by_id(veh_id))

    replay.visualize(sleep_time=5, saving_video=False, video_path="/home/")
    executionTime = (time.time() - startTime)

    print('Execution time in seconds: ' + str(executionTime))


if __name__ == "__main__":
    cli()
