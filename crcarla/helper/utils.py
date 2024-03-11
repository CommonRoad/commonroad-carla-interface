import hashlib
import logging
import math
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import List, Tuple, Union

import carla
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import psutil
from commonroad.common.util import make_valid_orientation
from commonroad.geometry.shape import Circle, Rectangle
from commonroad.planning.goal import AngleInterval, GoalRegion, Interval
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import (
    CustomState,
    ExtendedPMState,
    InitialState,
    KSState,
    PMState,
)
from commonroad.visualization.mp_renderer import MPRenderer
from PIL import Image

from crcarla.helper.config import BaseParam
from crcarla.objects.actor import ActorInterface

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


# def _get_nearby_vehicles(self, vehicles, ego, distance_th):
#     """Shows nearby vehicles of the hero actor"""
#     info_text = []
#     if self.hero_actor is not None and len(vehicles) > 1:
#         location = self.hero_transform.location
#         vehicle_list = [x[0] for x in vehicles if x[0].id != self.hero_actor.id]
#
#         def distance(v):
#             return location.distance(v.get_location())
#
#         for n, vehicle in enumerate(sorted(vehicle_list, key=distance)):
#             if n > distance_th:
#                 break
#             vehicle_type = get_actor_display_name(vehicle, truncate=22)
#             info_text.append('% 5d %s' % (vehicle.id, vehicle_type))
#     self._hud.add_info('NEARBY VEHICLES', info_text)


def create_goal_region_from_state(state: Union[KSState, PMState], ks_state: bool = True) -> GoalRegion:
    """
    Creates a CommonRoad goal region object given a state for a planning problem.

    :param state: CommonRoad state.
    :param ks_state: Boolean indicating whether given state is of a kinematic single-track model state.
    :return: CommonRoad goal region.
    """
    if ks_state:
        return GoalRegion(
            [
                CustomState(
                    time_step=Interval(state.time_step, state.time_step),
                    position=Circle(3, state.position),
                    velocity=Interval(max(0.0, state.velocity - 10), state.velocity + 10),
                    orientation=AngleInterval(state.orientation - 0.25, state.orientation + 0.25),
                )
            ]
        )

    velocity = max(state.velocity, state.velocity_y)
    orientation = math.atan2(state.velocity_y, state.velocity)
    return GoalRegion(
        [
            CustomState(
                time_step=Interval(state.time_step, state.time_step),
                position=Circle(10, state.position),
                velocity=Interval(max(0.0, velocity - 10), velocity + 10),
                orientation=AngleInterval(orientation - 0.25, orientation + 0.25),
            )
        ]
    )


def create_cr_vehicle_from_actor(actor: carla.Vehicle, cr_id: int, initial_time_step: int) -> DynamicObstacle:
    """
    Creates CommonRoad dynamic obstacle of type car given a CARLA actor.

    :param actor: CARLA vehicle actor.
    :param cr_id: CommonRoad ID which the dynamic obstacle should have.
    :param initial_time_step: Initial time step which should be used.
    :return: CommonRoad dynamic obstacle of type car.
    """
    vel_vec = actor.get_velocity()
    vel = math.sqrt(vel_vec.x**2 + vel_vec.y**2)
    transform = actor.get_transform()
    location = transform.location
    orientation = -((transform.rotation.yaw * math.pi) / 180)
    length = actor.bounding_box.extent.x * 2
    width = actor.bounding_box.extent.y * 2
    obs_type = extract_obstacle_type(actor.attributes["base_type"].lower())
    return DynamicObstacle(
        cr_id,
        obs_type,
        Rectangle(length, width),
        InitialState(initial_time_step, np.array([location.x, -location.y]), orientation, vel, 0, 0, 0),
    )


def extract_obstacle_type(carla_base_type: str) -> ObstacleType:
    """
    Matches CARLA actor base type to CommonRoad obstacle type.

    :param carla_base_type: CARLA actor base type.
    :return: CommonRoad obstacle type.
    """
    if carla_base_type == "car":
        return ObstacleType.CAR
    if carla_base_type == "bus":
        return ObstacleType.BUS
    if carla_base_type == "truck":
        return ObstacleType.TRUCK
    if carla_base_type == "motorcycle":
        return ObstacleType.MOTORCYCLE
    return ObstacleType.CAR


def create_cr_pm_state_from_actor(actor: carla.Actor, time_step: int) -> ExtendedPMState:
    """
    Creates point-mass model state of a CARLA actor at a time step.

    :param actor: CARLA actor.
    :param time_step: Time step of interest.
    :return: CommonRoad point-mass model state.
    """
    vel_vec = actor.get_velocity()
    acc_vec = actor.get_acceleration()
    vel = math.sqrt(vel_vec.x**2 + vel_vec.y**2)
    acc = math.sqrt(acc_vec.x**2 + acc_vec.y**2)
    transform = actor.get_transform()
    location = transform.location
    orientation = -((transform.rotation.yaw * math.pi) / 180)
    return ExtendedPMState(time_step, np.array([location.x, -location.y]), vel, orientation, acc)


def create_cr_ks_state_from_actor(actor: carla.Vehicle, time_step: int) -> KSState:
    """
    Creates kinematic single-track model state of a CARLA vehicle actor at a time step.

    :param actor: CARLA vehicle actor.
    :param time_step: Time step of interest.
    :return: CommonRoad kinematic single-track model state.
    """
    vel_vec = actor.get_velocity()
    vel = math.sqrt(vel_vec.x**2 + vel_vec.y**2 + vel_vec.z**2)
    transform = actor.get_transform()
    location = transform.location
    rotation = transform.rotation
    orientation = make_valid_orientation(-((rotation.yaw * math.pi) / 180))
    try:
        steer = actor.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
    except RuntimeError:
        steer = 0
    steering_angle = make_valid_orientation(steer * (math.pi / 180))
    return KSState(time_step, np.array([location.x, -location.y]), steering_angle, vel, orientation)


def create_cr_initial_state_from_actor(actor: carla.Vehicle, time_step: int) -> InitialState:
    """
    Creates kinematic single-track model state of a CARLA vehicle actor at a time step.

    :param actor: CARLA vehicle actor.
    :param time_step: Time step of interest.
    :return: CommonRoad initial state.
    """
    vel_vec = actor.get_velocity()
    acc_vec = actor.get_acceleration()
    vel = math.sqrt(vel_vec.x**2 + vel_vec.y**2)
    acc = math.sqrt(acc_vec.x**2 + acc_vec.y**2)
    transform = actor.get_transform()
    location = transform.location
    rotation = transform.rotation
    orientation = make_valid_orientation(-((rotation.yaw * math.pi) / 180))

    return InitialState(time_step, np.array([location.x, -location.y]), orientation, vel, acc, 0, 0)


#   slow_vehicles: {'vehicle.seat.leon', 'vehicle.dodge.charger_2020', 'vehicle.audi.tt', 'vehicle.chevrolet.impala',
#   'vehicle.mercedes.coupe', 'vehicle.lincoln.mkz_2017', 'vehicle.volkswagen.t2_2021', 'vehicle.citroen.c3',
#   'vehicle.bmw.grandtourer', 'vehicle.mini.cooper_s_2021', 'vehicle.nissan.micra',
#   'vehicle.dodge.charger_police_2020', 'vehicle.tesla.model3', 'vehicle.nissan.patrol_2021',
#   'vehicle.jeep.wrangler_rubicon', 'vehicle.audi.etron', 'vehicle.ford.crown', 'vehicle.mini.cooper_s'}


def create_cr_pedestrian_from_walker(actor: carla.Walker, cr_id: int, default_shape: bool = False) -> DynamicObstacle:
    """
    Creates CommonRoad dynamic obstacle of type pedestrian given a CARLA walker.

    :param actor: CARLA walker.
    :param cr_id: CommonRoad ID which the dynamic obstacle should have.
    :param default_shape: Boolean indicating whether default shape should (circle) be used.
    :return: CommonRoad dynamic obstacle of type walker.
    """
    vel_vec = actor.get_velocity()
    vel = math.sqrt(vel_vec.x**2 + vel_vec.y**2)
    transform = actor.get_transform()
    location = transform.location
    rotation = transform.rotation
    length = actor.bounding_box.extent.x * 2
    width = actor.bounding_box.extent.y * 2
    if default_shape:
        shape = Circle(0.4)  # TODO default shape size as config parameter
    elif abs(length) == math.inf or abs(width) == math.inf:
        shape = Circle(0.4)
    elif length == width:
        shape = Circle(length / 2)
    else:
        shape = Rectangle(length, width)
    return DynamicObstacle(
        cr_id,
        ObstacleType.PEDESTRIAN,
        shape,
        InitialState(0, np.array([location.x, -location.y]), -((rotation.yaw * math.pi) / 180), vel, 0, 0, 0),
    )


def calc_max_timestep(sc: Scenario) -> int:
    """
    Calculates maximal time step of current scenario.

    :param sc: scenario to calculate max time step
    :return: length of scenario
    """
    time_steps = [obstacle.prediction.final_time_step for obstacle in sc.dynamic_obstacles]
    return np.max(time_steps) if time_steps else 0


def find_pid_by_name(process_name: str) -> List[int]:
    """
    Get a list of all the PIDs of all the running process whose name contains
    the given string processName

    :param process_name: Name of process for which PID should be extracted.
    :return: List of possible PIDs
    """
    processes = []
    try:
        for proc in psutil.process_iter():
            try:
                if process_name.lower() in proc.name().lower():
                    processes.append(proc.pid)
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                logger.error("Error finding process.")
    except AttributeError:
        logger.error("Error iterating over processes.")
    except KeyError:
        logger.error("Error iterating over processes.")

    return processes


def make_video(path: Path, video_name: str):
    """
    Creates a video of the images recorded by camera sensor using ffmepg.

    @param path: Path to png images stored by camera sensor.
    @param video_name: Name which new video should have.
    """
    tmp_path = path / "_tmp"
    if not path.exists():
        path.mkdir(parents=True, exist_ok=True)
    if not tmp_path.exists():
        tmp_path.mkdir(parents=True, exist_ok=True)
    video_path = path / f"{video_name}.mp4"
    try:
        logger.debug("Start creating video.")
        os.system(
            f"ffmpeg -framerate 10 -hide_banner -loglevel error -pattern_type glob -i '{tmp_path}/*.png'"
            f" -c:v libx264 -pix_fmt yuv420p {video_path}"
        )
        tmp_path.unlink()

        if video_path.exists():
            logger.debug("mp4 created!")
    except Exception as e:
        if video_path.exists():
            logger.debug("mp4 created!")
        else:
            logger.error(e)


def find_carla_distribution(default_carla_paths: List[str]) -> Path:
    """
    Finds path of CARLA executable script based on given default paths.

    :param default_carla_paths: Paths to search for CARLA distribution.
    :return: Detected path.
    """
    if default_carla_paths is None:
        default_carla_paths = BaseParam().default_carla_paths
    for default_path in default_carla_paths:
        if (path := Path(default_path).expanduser()).exists():
            return path
    raise FileNotFoundError("CARLA executable not found.")


def kill_existing_servers(sleep_time: float):
    """
    Kills all running carla servers.

    :param sleep_time: The number of seconds to wait after killing the Carla server.
    """
    for pid in find_pid_by_name("CarlaUE4"):
        logger.info("Kill existing CARLA server with PID %s.", pid)
        os.killpg(os.getpgid(pid), signal.SIGTERM)

    time.sleep(sleep_time)

    for pid in find_pid_by_name("CarlaUE4"):
        logger.warning("CARLA server with PID %s did not terminate. Sending     SIGKILL.", pid)
        os.killpg(os.getpgid(pid), signal.SIGKILL)

    time.sleep(sleep_time)


def id_to_color(id_to_convert: Union[int, str]) -> Tuple[float, float, float]:
    """
    Generates random color from id.

    :param id_to_convert: The id to be converted to a valid rgb color.
    :return: A tuple of three float values representing the red, green, and blue
    """
    id_bytes = str(id_to_convert).encode("utf-8")
    color_hash = hashlib.md5(id_bytes).hexdigest()
    r, g, b = tuple(int(color_hash[i : i + 2], 16) for i in (0, 2, 4))
    r, g, b = r / 255.0, g / 255.0, b / 255.0
    return r, g, b


def render_obstacle_rectangle(state, car_width, car_length, color):
    """
    Renders a rectangular obstacle on a matplotlib plot.

    This function takes in the state of the obstacle, its width and length, and a color,
    and adds a rectangular patch to the current matplotlib axis.
    The position and orientation of the rectangle are determined by the state's position and orientation.


    :param state: The state of the obstacle.
    :param car_width: The width of the obstacle.
    :param car_length: The length of the obstacle.
    :param color: The color of the rectangle.

    """
    xi = state.position[0]
    yi = state.position[1]

    if isinstance(state, KSState):
        yaw = state.orientation
    else:
        yaw = 0

    width = max(car_width, 1)
    length = max(car_length, 1)

    x1 = xi - length / 2 * math.cos(yaw) + width / 2 * math.sin(yaw)
    y1 = yi - length / 2 * math.sin(yaw) - width / 2 * math.cos(yaw)
    rect = matplotlib.patches.Rectangle(
        (x1, y1),
        length,
        width,
        angle=math.degrees(yaw),
        color=color,
        zorder=float("inf"),
    )
    plt.gca().add_patch(rect)


def render_from_trajectory(
    scenario: Scenario,
    actuall_trajectories: list,
    predicted_trajectories: list,
    ids: List[str],
    actor_types: List[str],
    output_file: str,
    vehicle_widths: float,
    vehicle_lengths: float,
    title: str,
):
    """
    Renders a plot of actual and predicted trajectories for a given scenario.

    This function takes in a scenario, lists of actual and predicted trajectories,
    and other parameters, and generates a plot showing the trajectories of the actors
    in the scenario. The plot is saved to the specified output file.


    :param scenario: The scenario to render.
    :param actuall_trajectories: A list of the actual trajectories for the actors in the scenario.
    :param predicted_trajectories: A list of the predicted trajectories for the actors in the scenario.
    :param ids: A list of the IDs for the actors in the scenario.
    :param actor_types: A list of the actor types for the actors in the scenario.
    :param output_file: The path to the file where the plot will be saved.
    :param vehicle_widths: The widths of the vehicles in the scenario.
    :param vehicle_lengths: The lengths of the vehicles in the scenario.
    :param title: The title of the plot.

    """
    rnd = MPRenderer()

    rnd.draw_params.dynamic_obstacle.draw_initial_state = False
    rnd.draw_params.dynamic_obstacle.draw_bounding_box = False
    rnd.draw_params.dynamic_obstacle.draw_shape = False

    scenario.draw(rnd)
    rnd.render()
    legend_elements = []

    for i, actual_trajectory in enumerate(actuall_trajectories):
        actual_label = f"{actor_types[i]} : {ids[i]}"
        predicted_label = f"Predicted : {actor_types[i]} : {ids[i]}"

        actual_color = id_to_color(actual_label)
        predicted_color = id_to_color(predicted_label)

        legend_elements.append(matplotlib.lines.Line2D([0], [0], color=actual_color, lw=4, label=actual_label))
        legend_elements.append(matplotlib.lines.Line2D([0], [0], color=predicted_color, lw=4, label=predicted_label))
        actual_velocity = actual_trajectory.velocity
        legend_elements.append(
            matplotlib.lines.Line2D(
                [0],
                [0],
                color="none",
                lw=4,
                label=f"Actual Velocity: {actual_velocity:.3f}",
            )
        )
        legend_elements.append(matplotlib.lines.Line2D([0], [0], color="none", lw=4, label="Target Velocity: 20"))

        predicted_positions = np.array([t.position for t in predicted_trajectories[i]])
        x = predicted_positions[:, 0]
        y = predicted_positions[:, 1]
        plt.scatter(x, y, color=predicted_color, zorder=float("inf"), s=10)

        render_obstacle_rectangle(
            actual_trajectory,
            vehicle_widths[i],
            vehicle_lengths[i],
            actual_color,
        )

        x_ = actual_trajectory.position[0]
        y_ = actual_trajectory.position[1]
        margin = 60

        plt.xlim(x_ - margin, x_ + margin)
        plt.ylim(y_ - margin, y_ + margin)

    plt.legend(handles=legend_elements, bbox_to_anchor=(1.05, 1), loc="upper left")
    plt.title(title)
    plt.gcf().set_size_inches(10, 8)
    plt.savefig(output_file, bbox_inches="tight")


def render_trajectory_video(
    scenario: Scenario, fps: int, obstacles: List[any], output_file: str, title: str, exclude_pedestrians: bool = True
):
    """
    Renders a video of actual and predicted trajectories for a given scenario.

    This function takes in a scenario, the frames per second (fps) of the video,
    a list of obstacles, and other parameters, and generates a video showing the trajectories
    of the actors in the scenario. The video is saved to the specified output file.


    :param scenario: The scenario to render.
    :param fps: The frames per second of the video.
    :param obstacles: A list of obstacles in the scenario.
    :param output_file: The path to the file where the video will be saved.
    :param title: The title of the video.
    :param exclude_pedestrians: Whether to exclude pedestrians from the video. Defaults to True.

    """
    tmp_dir = Path("carla_tmp_frames")
    tmp_dir.mkdir(parents=True, exist_ok=True)

    if exclude_pedestrians:
        obstacles = [obs for obs in obstacles if "Vehicle" in type(obs).__name__]

    actuall_trajectories = [obs.trajectory for obs in obstacles]
    predicted_trajectories = [obs.cr_obstacle.prediction.trajectory.state_list for obs in obstacles]
    ids = [obs.cr_obstacle.obstacle_id for obs in obstacles]
    actor_types = ["Vehicle" if "Vehicle" in type(obs).__name__ else "Pedestrian" for obs in obstacles]

    # pylint: disable=protected-access
    car_widths = [obs._actor.bounding_box.extent.y * 2 for obs in obstacles]
    # pylint: disable=protected-access
    car_lengths = [obs._actor.bounding_box.extent.x * 2 for obs in obstacles]

    number_of_frames = len(actuall_trajectories[0])
    for i in range(number_of_frames):
        plt.figure()

        actual = [a[i] for a in actuall_trajectories]

        render_from_trajectory(
            scenario,
            actual,
            predicted_trajectories,
            ids,
            actor_types,
            f"{tmp_dir}/frame_{i:04d}.png",
            car_widths,
            car_lengths,
            title,
        )

        plt.close()

    video_file = f"{output_file}.mp4"

    # Resize the images if needed, so ffmpeg can render them -> It requires image_size % 2 = 0
    image_files = tmp_dir.glob("**/*")
    for image_file in image_files:
        image = Image.open(tmp_dir / image_file)

        new_width = image.width
        new_height = image.height

        if image.width % 2 != 0:
            new_width = (image.width // 2) * 2

        if image.height % 2 != 0:
            new_height = (image.height // 2) * 2

        if new_width != image.width or new_height != image.height:
            image = image.resize((new_width, new_height), Image.ANTIALIAS)
            image.save(tmp_dir / image_file)

    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-r",
            str(fps),
            "-i",
            f"{tmp_dir}/frame_%04d.png",
            "-vcodec",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            video_file,
        ],
        check=True,
    )

    tmp_dir.unlink()


def render_obs_trajectory_and_speed_comparisons(
    obstacles: List[ActorInterface], output_file: str, title: str, exclude_pedestrians: bool = True
):
    """
    Renders obstacle trajectories and compares speed to ground truth visually

    :param obstacles: Obstacle of interest.
    :param output_file: File where figure should be stored.
    :param title: Title of figure.
    :param exclude_pedestrians: Boolean indicating whether pedestrians should be excluded.
    """
    fig = plt.figure(figsize=(12, 8))
    fig.suptitle(title)

    ax1 = plt.subplot(211)
    ax2 = plt.subplot(212)

    if exclude_pedestrians:
        obstacles = [obs for obs in obstacles if "Vehicle" in type(obs).__name__]

    for obs in obstacles:
        actual_trajectory = obs.trajectory[1:]
        predicted_trajectory = obs.cr_obstacle.prediction.trajectory.state_list
        obs_id = obs.cr_obstacle.obstacle_id
        actor_type = "Vehicle" if "Vehicle" in type(obs).__name__ else "Pedestrian"
        label = f"{actor_type} : {obs_id}"

        position_differences = [
            math.sqrt(
                (actual_state.position[0] - predicted_state.position[0]) ** 2
                + (actual_state.position[1] - predicted_state.position[1]) ** 2
            )
            for actual_state, predicted_state in zip(actual_trajectory, predicted_trajectory)
        ]
        actual_speeds = [state.velocity for state in actual_trajectory]
        predicted_speeds = [state.velocity for state in predicted_trajectory]

        ax1.plot(position_differences, label=label)
        ax2.plot(actual_speeds, label=f"Actual {label}")
        ax2.plot(predicted_speeds, label=f"Predicted {label}")

    ax1.set_title("Position Difference")
    ax2.set_title("Speed Comparison")

    box1 = ax1.get_position()
    box2 = ax2.get_position()

    ax1.set_position([box1.x0, box1.y0 + box1.height * 0.1, box1.width * 0.8, box1.height])
    ax2.set_position([box2.x0, box2.y0, box2.width * 0.8, box2.height])

    ax1.legend()
    ax2.legend()

    plt.subplots_adjust(hspace=0.4)

    plt.savefig(output_file + ".png")


def init_camera_sensor(
    world: carla.World,
    camera_transform_horizontal: carla.Transform,
    camera_transform_bird: carla.Transform,
    image_path: str,
) -> List[carla.Actor]:
    """
    Initializes cameras for custom images. The default cameras are intended for horizontal and
    birds-eye view for Town10, but they can be placed arbitrarily
    :param world: CARLA world.
    :param camera_transform_horizontal: Transform for "horizontal" camera.
    :param camera_transform_bird:  Transform for "birds-eye" camera
    :param image_path: Path where images should be stored.
    """
    camera_blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
    camera_horizontal = world.spawn_actor(camera_blueprint, camera_transform_horizontal)
    camera_bird = world.spawn_actor(camera_blueprint, camera_transform_bird)
    image_width = camera_blueprint.get_attribute("image_size_x").as_int()
    image_height = camera_blueprint.get_attribute("image_size_y").as_int()
    camera_data = {"image": np.zeros((image_height, image_width, 4))}
    camera_bird.listen(lambda image: camera_callback(image, camera_data, f"{image_path}/images_bird"))
    camera_horizontal.listen(lambda image: camera_callback(image, camera_data, f"{image_path}/images_horizontal"))

    def camera_callback(image, data, path):
        data["image"] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
        image.save_to_disk(f"{path}/{image.frame}.png")

    return [camera_bird, camera_horizontal]
