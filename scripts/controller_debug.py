from commonroad.scenario.state import InitialState
from commonroad.scenario.trajectory import Trajectory
from matplotlib import pyplot as plt
from dataclasses import dataclass


@dataclass(init=False)
class ControllerDebug:
    """Visualization class for controller debugging"""

    def __init__(self, state: InitialState):
        """Initialize of desired values based on the initial state of the vehicle"""
        self._des_vel = [state.velocity]
        self._act_vel = []
        self._des_ori = [state.orientation]
        self._act_ori = []
        self._throttle = []
        self._brake = []

    def add_act_state(self, state: InitialState):
        """
        Add actual state of the vehicle to the history (velocity & orientation)

        :param state: InitialState of the PlanningProblem
        """
        self._act_vel.append(state.velocity)
        self._act_ori.append(state.orientation)

    def add_des_state(self, traj: Trajectory, lookahead: int):
        """
        Add desired state of the vehicle to the history (velocity & orientation)

        :param traj: Trajectory as result of the planning
        :param lookahead: Number of steps to look ahead (take the i-th entry)
        """
        self._des_vel.append(traj.state_list[lookahead].velocity)
        self._des_ori.append(traj.state_list[lookahead].orientation)

    def add_control(self, control):
        """
        Add control to the history (acceleration & breaking)

        :param control: Control as result of the controller (e.g. PID)
        """
        self._throttle.append(control.throttle)
        self._brake.append(control.brake)

    def plot(self):
        """
        Plots the desired and actual states to visualize the controller performance.
        """
        fig, ax = plt.subplots(3, 1, figsize=(10, 15))  # Three rows, one column of plots

        # Plot desired and actual velocities
        ax[0].stem(
            range(len(self._des_vel)), self._des_vel, basefmt=" ", linefmt="b", markerfmt="bo", label="Desired Velocity"
        )
        ax[0].stem(
            range(len(self._act_vel)), self._act_vel, basefmt=" ", linefmt="r", markerfmt="ro", label="Actual Velocity"
        )
        ax[0].set_title("Comparison of Actual vs. Desired Velocity")
        ax[0].set_xlabel("Index")
        ax[0].set_ylabel("Velocity (m/s)")
        ax[0].legend()
        ax[0].grid(True)

        # Plot throttle and brake
        ax[1].bar(range(len(self._throttle)), self._throttle, color="green", label="Throttle")
        ax[1].bar(range(len(self._brake)), [-b for b in self._brake], color="red", label="Brake")
        ax[1].set_title("Throttle and Brake Inputs")
        ax[1].set_xlabel("Index")
        ax[1].set_ylabel("Input Level")
        ax[1].legend()
        ax[1].grid(True)

        # Plot desired and actual orientations
        ax[2].plot(range(len(self._des_ori)), self._des_ori, "b--", label="Desired Orientation")
        ax[2].plot(range(len(self._act_ori)), self._act_ori, "r-", label="Actual Orientation")
        ax[2].set_title("Comparison of Actual vs. Desired Orientation")
        ax[2].set_xlabel("Index")
        ax[2].set_ylabel("Orientation (degrees)")
        ax[2].legend()
        ax[2].grid(True)

        # Adjust layout
        plt.tight_layout()

        # Display the plot
        plt.show()
