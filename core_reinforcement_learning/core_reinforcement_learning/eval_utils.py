#!/usr/bin/env python3
import os
import yaml
import cv2
import ast
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from core_reinforcement_learning.utils import get_transform_angle
from core_reinforcement_learning.human_metrics import HumanMetrics
import numpy as np
import matplotlib.pyplot as plt
import csv
import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib.gridspec as gridspec


def eval_data_callback(locals, globals, node: Node, rl_io_manager):
    """
    Callbacks for evaluating data during training.

    Args:
    ----
        locals: Local variables.
        globals: Global variables.
        node (Node): The ROS 2 node.
        rl_io_manager: The RL I/O manager.

    """
    local = locals["infos"][0]

    if 'total_reward' in local:

        # Calculate mean and standard deviation
        # robot_pose = rl_io_manager.last_robot_odom.flatten()
        # agent_pose_global =  np.array(rl_io_manager.last_agents_global_frame)

        sim_name = node.sim_name
        sim_path = node.sim_path
        robot_pose = (
            local["robot_pose"]
            if "robot_pose" in local
            else rl_io_manager.last_robot_odom.flatten()
        )
        agent_pose_global = (
            np.transpose(np.array(local["agent_pose"]), (1, 0, 2))
            if "agent_pose" in local
            else np.array(rl_io_manager.last_agents_global_frame)
        )
        total_reward_mean = list(
            local["total_reward"]) if "total_reward" in local else 0
        terminal_reward_mean = list(
            local["terminal_reward"]) if "terminal_reward" in local else 0
        goal_distance_reward_mean = list(
            local["goal_distance_reward"]) if "goal_distance_reward" in local else 0
        done_reason = list(local["done_reason"]
                           ) if "done_reason" in local else "None"
        in_interaction_range = list(
            local["in_interaction_range"]) if "in_interaction_range" in local else 0
        social_force_sfm_reward = list(
            local["social_force_sfm_reward"]) if "social_force_sfm_reward" in local else 0
        social_force_deceleration = list(
            local["force_deceleration"]) if "force_deceleration" in local else 0
        social_force_evasion = list(
            local["force_evasion"]) if "force_evasion" in local else 0
        proxemics_reward = list(
            local["proxemics_reward"]) if "proxemics_reward" in local else 0
        sim_time = list(local["sim_time"]) if "sim_time" in local else 0

        # Log the mean and std of the total reward
        """node.get_logger().error(f"Mean reward: {total_reward_mean:.2f}")
        node.get_logger().error(f"Mean clipped reward: {clipped_reward_mean:.2f}")
        node.get_logger().error(f"Mean goal distance reward: {goal_distance_reward_mean:.2f}")
        node.get_logger().error(f"Mean velocity reward: {velocity_reward_mean:.2f}")
        """

        # if done_reason == "Not done":
        if not os.path.exists(f'{sim_path}'):
            os.makedirs(f'{sim_path}')

        with open(f'{sim_path}/reward_data_{sim_name}.csv', mode='a', newline='') as file:
            writer = csv.writer(file)
            # Write the header
            if file.tell() == 0:
                writer.writerow([
                    "total_reward_mean",
                    "terminal_reward_mean",
                    "goal_distance_reward_mean",
                    "social_force_sfm_reward",
                    "social_force_deceleration",
                    "social_force_evasion",
                    "robot_pose_x",
                    "robot_pose_y",
                    "robot_pose_theta",
                    "robot_vel_x",
                    "robot_vel_y",
                    "agent_pose_x_global",
                    "agent_pose_y_global",
                    "agent_pose_vel_x_global",
                    "agent_pose_vel_y_global",
                    "done_reason",
                    "in_interaction_range",
                    "proxemics_reward",
                    "sim_time"

                ])

            writer.writerow([
                total_reward_mean,
                terminal_reward_mean,
                goal_distance_reward_mean,
                social_force_sfm_reward,
                social_force_deceleration,
                social_force_evasion,
                [pose[0] for pose in robot_pose],
                [pose[1] for pose in robot_pose],
                [pose[2] for pose in robot_pose],
                [pose[3] for pose in robot_pose],
                [pose[4] for pose in robot_pose],
                agent_pose_global[:, :, 0].tolist(),
                agent_pose_global[:, :, 1].tolist(),
                agent_pose_global[:, :, 2].tolist(),
                agent_pose_global[:, :, 3].tolist(),
                done_reason,
                in_interaction_range,
                proxemics_reward,
                sim_time
            ])


class Evaluate_Model:
    """Class for evaluating the trained model and generating metrics and plots."""

    def __init__(self, node: Node, **kwargs):
        """
        Initialize the Evaluate_Model class.

        Args:
        ----
            node (Node): The ROS 2 node.
            **kwargs: Additional keyword arguments.

        """
        self.node = node
        self.sim_name = self.node.sim_name
        self.sim_path = self.node.sim_path
        self.seed = self.node.seed
        self.time_step_length = self.node.time_step_length
        self.reward_samples_per_timestep = self.node.reward_samples_per_timestep
        self.num_eval_per_scenario = self.node.num_eval_per_scenario

        # Social Force Params
        self.lambda_ = kwargs.get('lambda_', 0.5)
        self.gamma_ = kwargs.get('gamma_', 0.5)
        self.epsilon = kwargs.get('epsilon', 0.5)
        self.n_prime = kwargs.get('n_prime', 0.5)
        self.n = kwargs.get('n', 0.5)
        self.A = kwargs.get('A', 0.5)
        self.colors = self.get_color_palette()
        self.plot_lim_seed = self.return_plot_lim_seed(self.seed)
        self.map_image, self.resolution, self.origin = self.load_map()

        self.metrics_to_compute = {}
        self.metrics_lists = {}
        self.metrics_to_use = self.node.get_parameters_by_prefix("metrics")
        self.result_file = self.node.get_parameter('result_file').value

    # UTILS

    def get_color_palette(self):
        """
        Get the color palette for plotting.

        Returns
        -------
            dict: A dictionary of color names and their RGB values.

        """
        return {
            "bright_dark_blue": (0.0, 0.23, 0.49),
            "med_blue": (0.0, 0.55, 1.0),
            "pink": (1.0, 0.45, 0.71),
            "purple": (0.78, 0.0, 1.0),
            "green": (0.31, 0.8, 0.55),
            "orange": (1.0, 0.62, 0.23),
            "yellow": (0.98, 0.91, 0.35),
            "red": (0.85, 0.19, 0.2),
            "rubber": (0.2, 0.2, 0.2),
            "grey": (0.5, 0.5, 0.5)
        }

    def rgb_to_bgr(self, rgb_color):
        """
        Convert RGB color to BGR color.

        Args:
        ----
            rgb_color (tuple): RGB color values.

        Returns
        -------
            tuple: BGR color values.

        """
        r, g, b = rgb_color
        bgr = (b * 255, g * 255, r * 255)
        bgr = tuple(map(int, bgr))

        return bgr

    def world_to_map_coordinates(self, world_coords, origin, resolution, map_image):
        """
        Transform world coordinates to map coordinates.

        Args:
        ----
            world_coords (tuple): (x, y) coordinates in the world frame.
            origin (list): Origin of the map in the world frame.
            resolution (float): Resolution of the map (meters per pixel).

        Returns
        -------
            tuple: (x, y) coordinates in the map frame.

        """
        map_x = int((world_coords[0] - origin[0]) / resolution)
        map_y = int((world_coords[1] - origin[1]) / resolution)
        map_y = map_image.shape[0] - map_y
        return (map_x, map_y)

    def enhance_image(self, image):
        """
        Enhance the quality of the map image.

        Args:
        ----
            image (numpy.ndarray): The map image.

        Returns
        -------
            numpy.ndarray: The enhanced map image.

        """
        # Convert to color if the image is grayscale
        if len(image.shape) == 2 or image.shape[2] == 1:
            color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            color_image = image

        # Apply Gaussian blur to smooth the image
        blurred_image = cv2.GaussianBlur(color_image, (5, 5), 0)

        # Enhance contrast using histogram equalization
        lab = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2LAB)
        la, a, b = cv2.split(lab)
        la = cv2.equalizeHist(la)
        enhanced_image = cv2.merge((la, a, b))
        enhanced_image = cv2.cvtColor(enhanced_image, cv2.COLOR_LAB2BGR)

        return enhanced_image

    def load_csv_data(self, file_path):
        """
        Load data from a CSV file.

        Args:
        ----
            file_path (str): Path to the CSV file.

        Returns
        -------
            list: The data in the CSV file.

        """
        with open(file_path, mode='r') as file:
            reader = csv.reader(file)
            data = list(reader)

        return data

    def return_plot_lim_seed(self, seed):
        """
        Return the plot limits based on the seed.

        Args:
        ----
            seed (int): The seed value.

        Returns
        -------
            dict: The plot limits for the given seed.

        """
        plot_lim_seeds = {
            2:  {
                'animation': {
                    'xlim': (-5.5, 4.0),
                    'ylim': (-2.0, 3.5)
                },
            }
        }
        return plot_lim_seeds[2]

    # SOCIAL FORCE FUNCTIONS

    def social_force(self, d, theta):
        """
        Calculate the social force based on distance and angle.

        Args:
        ----
            d (float): Distance between the robot and the agent.
            theta (float): Angle between the robot and the agent.

        Returns
        -------
            tuple: The deceleration force, evasion force, and sum force.

        """
        agent_global_velocity_vector = np.array([0.4, 0.0])
        robot_global_velocity_vector = np.array(
            [-0.4 * np.cos(theta), -0.4 * np.sin(theta)])
        vel_diff = agent_global_velocity_vector - robot_global_velocity_vector

        location_vector = np.array([d * np.cos(theta), d * np.sin(theta)])
        distance_robot_agent = np.linalg.norm(location_vector)
        diff_direction = location_vector / distance_robot_agent
        interaction_vector = self.lambda_ * vel_diff + diff_direction
        interaction_length = np.linalg.norm(interaction_vector)
        interaction_direction = interaction_vector / interaction_length
        normal_direction = np.array(
            [-interaction_direction[1], interaction_direction[0]])
        B = self.gamma_ * interaction_length

        theta_eff = get_transform_angle(
            interaction_direction, diff_direction) + self.epsilon * B
        force_deceleration_amount = - \
            np.exp(-distance_robot_agent / B -
                   (self.n_prime * B * theta_eff)**2)
        force_evasion_amount = - \
            np.sign(theta_eff) * np.exp(-distance_robot_agent /
                                        B - (self.n * B * theta_eff)**2)
        force_deceleration = force_deceleration_amount * interaction_direction
        force_evasion = force_evasion_amount * normal_direction
        sum_force = -self.A * \
            np.linalg.norm(force_deceleration + force_evasion)

        return force_deceleration, force_evasion, sum_force

    def plot_social_interaction(self):
        """Plot the social force interaction over distance and angle."""
        distance_values = np.linspace(0.05, 2, 100)
        # Theta from 0 to 2*pi radians
        theta_values = np.linspace(-np.pi, np.pi, 100)

        # Create meshgrid for the plot
        D, Theta = np.meshgrid(distance_values, theta_values)

        # Compute social force for each combination of distance and angle
        Social_Force = np.zeros_like(D)

        for i in range(D.shape[0]):
            for j in range(D.shape[1]):
                _, _, Social_Force[i, j] = self.social_force(
                    D[i, j], Theta[i, j])

        # Create the plot (using contourf for heatmap-like visualization)
        fig, ax = plt.subplots(
            subplot_kw={'projection': 'polar'}, figsize=(10, 6))
        c = ax.contourf(Theta, D, Social_Force, levels=100, cmap='jet')
        fig.colorbar(c, ax=ax, label='Social Force')
        ax.set_xticks(np.pi/180. * np.linspace(0,  360, 8, endpoint=False))
        ax.set_xticklabels(['0', r'$\pi/4$', r'$\pi/2$', r'$3\pi/4$',
                           r'$\pi$', r'$5\pi/4$', r'$3\pi/2$', r'$7\pi/4$'])
        ax.grid(alpha=0.6)

        # Label the axes
        ax.set_title('Social Force Sweeping Over Distance and Angle')
        ax.set_xlabel('Distance (m)')
        ax.set_ylabel('Angle (radians)')

        plt.savefig('social_interaction.png')

        plt.close()

    def plot_social_animation(self, data_dict: dict):
        """
        Plot the social force animation.

        Args:
        ----
            data_dict (dict): Dictionary containing the data for plotting.

        """
        robot_odom = data_dict['robot']
        agent_odom = data_dict['agents']
        # Create a 2x3 grid of subplots using GridSpec
        fig = plt.figure(figsize=(18, 18))
        gs = gridspec.GridSpec(2, 6, width_ratios=[
                               0.4, 1, 1, 1, 1, 0.4], height_ratios=[2.8, 1.0])

        # Main plot spans both rows in the first column
        ax_main = fig.add_subplot(gs[0, :])
        ax_bar1 = fig.add_subplot(gs[1, 1])
        ax_bar2 = fig.add_subplot(gs[1, 2])
        ax_bar3 = fig.add_subplot(gs[1, 3])
        ax_bar4 = fig.add_subplot(gs[1, 4])

        ax_main.set_aspect('auto')
        ax_main.grid()
        ax_main.set_axisbelow(True)

        # Calculate the extent of the image
        img_height, img_width = self.map_image.shape[:2]
        extent = [
            self.origin[0],
            self.origin[0] + img_width * self.resolution,
            self.origin[1] + img_height * self.resolution,
            self.origin[1],
        ]

        # Display the background image
        ax_main.imshow(self.map_image, extent=extent,
                       cmap='gray', origin='lower')

        robot_quiver = ax_main.quiver(
            [], [], [], [], color=self.colors["red"], scale=1, label='Robot Deceleration Force')
        evasion_quiver = ax_main.quiver(
            [], [], [], [], color=self.colors["purple"], scale=1, label='Robot Evasion Force')
        sum_quiver = ax_main.quiver(
            [], [], [], [], color=self.colors["yellow"], scale=1, label='Sum Force')

        # Create for the robot
        robot_size = 0.25
        wheel_width = 0.1
        wheel_height = 0.05
        robot_square = patches.Rectangle(
            (0, 0), robot_size, robot_size, color=self.colors["orange"], fill=True, label='Robot')
        robot_wheel_1 = patches.Rectangle(
            (0, 0), wheel_width, wheel_height, color=self.colors["rubber"], fill=True)
        robot_wheel_2 = patches.Rectangle(
            (0, 0), wheel_width, wheel_height, color=self.colors["rubber"], fill=True)

        # Create a list to hold agent circles
        # Assuming agent_odom has shape [M, N, 4] where N = time, M = number of agents
        num_agents = agent_odom.shape[0]
        agent_circles = [patches.Circle(
            (0, 0), 0.12, color=self.colors["bright_dark_blue"], fill=True, label='Agent')
            for _ in range(num_agents)]
        for circle in agent_circles:
            ax_main.add_patch(circle)

        interaction_circle = patches.Circle(
            (0, 0), 0.35, color=self.colors["green"], fill=False, linestyle='--',
            linewidth=2, label='In Interaction Range')

        ax_main.add_patch(robot_square)
        ax_main.add_patch(robot_wheel_1)
        ax_main.add_patch(robot_wheel_2)
        ax_main.add_patch(interaction_circle)

        # Initialize vertical bars for force magnitudes
        bar_width = 0.1
        bar_positions = [0.05]  # Positions for the bars
        # Bar1: Goal Cost
        bars1 = ax_bar1.bar(
            bar_positions, [0, 0], width=bar_width, color=self.colors["yellow"])

        # Bar2: Sum of Social Forces
        bars2 = ax_bar2.bar(
            bar_positions, [0, 0], width=bar_width, color=self.colors["red"])

        # Bar3: Velocity Cost
        bars3 = ax_bar3.bar(
            bar_positions, [0, 0], width=bar_width, color=self.colors["purple"])

        # Bar4: Total Reward
        bars4 = ax_bar4.bar(
            bar_positions, [0, 0], width=bar_width, color=self.colors["grey"])

        xticklabels_bar = ['Goal Reward', 'Interaction Reward',
                           'Proxemics Reward', 'Total Reward']

        for i, ax_bar in enumerate([ax_bar1, ax_bar2, ax_bar3, ax_bar4]):

            ax_bar.set_ylim(-1, 1)
            ax_bar.set_xlim(0, 0.1)
            ax_bar.set_yticks([])
            ax_bar.set_xticks(bar_positions)
            ax_bar.set_xticklabels([xticklabels_bar[i]], fontsize=20)

        ax_bar1.set_yticks(np.arange(-1, 1.1, 0.5), fontsize=20)

        ax_bar1.set_ylabel('Force Magnitude', fontsize=20)

        def init():
            robot_quiver.set_offsets(np.array([[0, 0]]))
            evasion_quiver.set_offsets(np.array([[0, 0]]))
            sum_quiver.set_offsets(np.array([[0, 0]]))
            robot_square.set_xy((0, 0))
            robot_wheel_1.set_xy((0, 0))
            robot_wheel_2.set_xy((0, 0))

            # Set initial positions for all agents
            for circle in agent_circles:
                circle.set_center((0, 0))

            interaction_circle.set_center((0, 0))
            interaction_circle.set_visible(False)

            robot_square.set_zorder(1)
            robot_wheel_1.set_zorder(1)
            robot_wheel_2.set_zorder(1)
            evasion_quiver.set_zorder(2)
            sum_quiver.set_zorder(2)
            robot_quiver.set_zorder(2)
            interaction_circle.set_zorder(0)

            return (robot_quiver, evasion_quiver, sum_quiver, robot_square,
                    robot_wheel_1, robot_wheel_2, interaction_circle,
                    *agent_circles, bars1, bars2, bars3, bars4)

        def update(i):

            robot_x, robot_y, robot_yaw = (robot_odom[i, 0],
                                           robot_odom[i, 1], robot_odom[i, 4])

            # Update the positions of all agents
            for j in range(num_agents):
                agent_x = agent_odom[j, i, 0]
                agent_y = agent_odom[j, i, 1]
                agent_circles[j].set_center((agent_x, agent_y))

            robot_square.set_xy(
                (robot_x - robot_size / 2, robot_y - robot_size / 2))
            robot_wheel_1.set_xy(
                (robot_x - robot_size / 2, robot_y + robot_size / 2))
            robot_wheel_2.set_xy(
                (robot_x - robot_size / 2, robot_y - 0.05 - robot_size / 2))

            self.rotate_patch(
                robot_square, (robot_x, robot_y), robot_yaw, ax_main)
            self.rotate_patch(
                robot_wheel_1, (robot_x, robot_y), robot_yaw, ax_main)
            self.rotate_patch(
                robot_wheel_2, (robot_x, robot_y), robot_yaw, ax_main)

            # Update interaction circle visibility
            if data_dict['in_interaction_range'][i]:
                # interaction_circle.set_center((robot_x, robot_y))
                interaction_circle.set_visible(True)
            else:
                interaction_circle.set_visible(False)

            # Update the bar heights
            bars1[0].set_height(data_dict['goal_distance_reward_mean'][i])
            bars2[0].set_height(data_dict['social_force_sfm_reward'][i])
            bars3[0].set_height(data_dict['proxemics_reward'][i])
            bars4[0].set_height(data_dict['total_reward_mean'][i])

            return (robot_quiver, evasion_quiver, sum_quiver, robot_square,
                    robot_wheel_1, robot_wheel_2, interaction_circle,
                    *agent_circles, bars1, bars2, bars3, bars4)

        ani = animation.FuncAnimation(fig, update, frames=len(
            robot_odom), init_func=init, blit=False, interval=100)
        ax_main.set_xlim(
            self.origin[0], self.origin[0] + img_width * self.resolution)
        ax_main.set_ylim(
            self.origin[1], self.origin[1] + img_height * self.resolution)
        plt.tight_layout()
        ax_main.set_xlim(self.origin[0] + 0.0, self.origin[0] + 4.0)
        ax_main.set_ylim(self.origin[1] + 0.0, self.origin[1] + 4.0)
        # ax_main.legend(loc='upper right')
        plt.subplots_adjust(wspace=0.2, hspace=0.2)
        # plt.suptitle(f'Social Force Interaction {self.sim_name}', fontsize= 25)
        # plt.show()

        ani.save(f'{self.sim_path}/animation_{self.sim_name}.mp4', writer='ffmpeg',
                 fps=int(np.ceil(1/self.time_step_length))*self.reward_samples_per_timestep)

    def plot_animation_rewards(self, data_dict: dict):
        """
        Plot the animation of rewards over time.

        Args:
        ----
            data_dict (dict): Dictionary containing the data for plotting.

        """
        robot_odom = data_dict['robot']
        agent_odom = data_dict['agents']
        # Create a 2x3 grid of subplots using GridSpec
        fig = plt.figure(figsize=(14, 22))
        gs = gridspec.GridSpec(2, 1, width_ratios=[
                               1], height_ratios=[1.0, 1.0])

        # Main plot spans both rows in the first column
        ax_main = fig.add_subplot(gs[0, 0])
        ax_plot = fig.add_subplot(gs[1, 0])

        closest_agent_marker = None
        social_force_marker = None
        current_time_marker = None
        current_vel_marker = None

        ax_main.set_aspect('auto')
        # ax_main.grid()
        ax_main.set_axisbelow(True)

        # Calculate the extent of the image
        img_height, img_width = self.map_image.shape[:2]
        extent = [
            self.origin[0],
            self.origin[0] + img_width * self.resolution,
            self.origin[1] + img_height * self.resolution,
            self.origin[1],
        ]

        # Display the background image
        ax_main.imshow(self.map_image, extent=extent,
                       cmap='gray', origin='lower')

        robot_quiver = ax_main.quiver(
            [], [], [], [], color=self.colors["red"], scale=1, label='Robot Deceleration Force')
        evasion_quiver = ax_main.quiver(
            [], [], [], [], color=self.colors["purple"], scale=1, label='Robot Evasion Force')
        sum_quiver = ax_main.quiver(
            [], [], [], [], color=self.colors["yellow"], scale=1, label='Sum Force')

        # Create for the robot
        robot_size = 0.25
        wheel_width = 0.1
        wheel_height = 0.05
        robot_square = patches.Rectangle(
            (0, 0), robot_size, robot_size, color=self.colors["orange"], fill=True, label='Robot')
        robot_wheel_1 = patches.Rectangle(
            (0, 0), wheel_width, wheel_height, color=self.colors["rubber"], fill=True)
        robot_wheel_2 = patches.Rectangle(
            (0, 0), wheel_width, wheel_height, color=self.colors["rubber"], fill=True)

        # Create a list to hold agent circles
        # Assuming agent_odom has shape [M, N, 4] where N = time, M = number of agents
        num_agents = agent_odom.shape[0]
        agent_circles = [patches.Circle(
            (0, 0), 0.12, color=self.colors["bright_dark_blue"], fill=True, label='Agent')
            for _ in range(num_agents)]
        for circle in agent_circles:
            ax_main.add_patch(circle)

        interaction_circle = patches.Circle(
            (0, 0), 0.35, color=self.colors["green"], fill=False,
            linestyle='--', linewidth=2, label='In Interaction Range')

        ax_main.add_patch(robot_square)
        ax_main.add_patch(robot_wheel_1)
        ax_main.add_patch(robot_wheel_2)
        ax_main.add_patch(interaction_circle)
        ax_main.set_yticks([])
        ax_main.set_xticks([])

        ax_plot.set_aspect('auto')
        ax_plot.set_xlabel('Time (s)', fontdict={'fontsize': 40})
        # ax_plot.set_ylabel('Metric (-)', fontdict={'fontsize': 40})
        ax_plot.tick_params(axis='both', which='major', labelsize=30)

        ax_plot.set_axisbelow(True)
        ax_plot.set_ylim(0, 3)

        # Plot the closest distance and the social force5
        ax_plot.plot(data_dict['sim_time'], data_dict['closest_agent_distance'],
                     color=self.colors["red"], label='Agent Distance [m]', linewidth=4)
        ax_plot.plot(data_dict['sim_time'], -data_dict['social_force_sfm_reward'],
                     color=self.colors["purple"], label='Interaction Force Metric[-]', linewidth=4)
        ax_plot.plot(data_dict['sim_time'], data_dict['robot_velocity'],
                     color=self.colors["med_blue"], label='Robot Velocity [m/s]', linewidth=4)

        # ax_plot.legend(loc='upper right', fontsize=28)

        def init():
            robot_quiver.set_offsets(np.array([[0, 0]]))
            evasion_quiver.set_offsets(np.array([[0, 0]]))
            sum_quiver.set_offsets(np.array([[0, 0]]))
            robot_square.set_xy((0, 0))
            robot_wheel_1.set_xy((0, 0))
            robot_wheel_2.set_xy((0, 0))

            # Set initial positions for all agents
            for circle in agent_circles:
                circle.set_center((0, 0))

            interaction_circle.set_center((0, 0))
            interaction_circle.set_visible(False)

            robot_square.set_zorder(1)
            robot_wheel_1.set_zorder(1)
            robot_wheel_2.set_zorder(1)
            evasion_quiver.set_zorder(2)
            sum_quiver.set_zorder(2)
            robot_quiver.set_zorder(2)
            interaction_circle.set_zorder(0)

            return (robot_quiver, evasion_quiver, sum_quiver, robot_square,
                    robot_wheel_1, robot_wheel_2, interaction_circle, *agent_circles)

        def update(i):

            robot_x, robot_y, robot_yaw = robot_odom[i,
                                                     0], robot_odom[i, 1], robot_odom[i, 4]
            nonlocal closest_agent_marker, social_force_marker
            nonlocal current_time_marker, current_vel_marker

            if closest_agent_marker:
                closest_agent_marker.remove()
            if social_force_marker:
                social_force_marker.remove()
            if current_time_marker:
                current_time_marker.remove()
            if current_vel_marker:
                current_vel_marker.remove()
            # Plot marker inside the ax_plot for the closest agent distance and the social force
            closest_agent_marker, = ax_plot.plot(
                data_dict['sim_time'][i], data_dict['closest_agent_distance'][i],
                color=self.colors["red"], marker='o', markersize=15)
            social_force_marker, = ax_plot.plot(
                data_dict['sim_time'][i], -
                data_dict['social_force_sfm_reward'][i],
                color=self.colors["purple"], marker='o', markersize=15)
            current_vel_marker, = ax_plot.plot(
                data_dict['sim_time'][i], data_dict['robot_velocity'][i],
                color=self.colors["med_blue"], marker='o', markersize=15)
            # Plot vertical line at current time
            current_time_marker = ax_plot.axvline(
                x=data_dict['sim_time'][i], color='k', linestyle='--', linewidth=1)

            # Update the positions of all agents
            for j in range(num_agents):
                agent_x = agent_odom[j, i, 0]
                agent_y = agent_odom[j, i, 1]
                agent_circles[j].set_center((agent_x, agent_y))

            robot_square.set_xy(
                (robot_x - robot_size / 2, robot_y - robot_size / 2))
            robot_wheel_1.set_xy(
                (robot_x - robot_size / 2, robot_y + robot_size / 2))
            robot_wheel_2.set_xy(
                (robot_x - robot_size / 2, robot_y - 0.05 - robot_size / 2))

            self.rotate_patch(
                robot_square, (robot_x, robot_y), robot_yaw, ax_main)
            self.rotate_patch(
                robot_wheel_1, (robot_x, robot_y), robot_yaw, ax_main)
            self.rotate_patch(
                robot_wheel_2, (robot_x, robot_y), robot_yaw, ax_main)

            # Update interaction circle visibility
            if data_dict['in_interaction_range'][i]:
                interaction_circle.set_center((robot_x, robot_y))
                interaction_circle.set_visible(True)
            else:
                interaction_circle.set_visible(False)

            return (robot_quiver, evasion_quiver, sum_quiver, robot_square, robot_wheel_1,
                    robot_wheel_2, interaction_circle, *agent_circles)

        ani = animation.FuncAnimation(fig, update, frames=len(
            robot_odom), init_func=init, blit=False, interval=100)

        ax_main.set_xlim(self.origin[0] + 0.5, self.origin[0] + 7.5)
        ax_main.set_ylim(self.origin[1] + 4.0, self.origin[1] + 6.5)

        # ax_main.set_xlim(self.origin[0]+ 0.0, self.origin[0] + 4.0)
        # ax_main.set_ylim(self.origin[1]+ 0.0, self.origin[1] + 4.0)
        # ax_main.legend(loc='center right')
        # plt.subplots_adjust(wspace=0.2, hspace=0.2)
        plt.tight_layout()
        # plt.suptitle(f'Social Force Interaction {self.sim_name}', fontsize= 25)
        # plt.show()

        ani.save(f'{self.sim_path}/animation_reward_{self.sim_name}.mp4', writer='ffmpeg',
                 fps=int(np.ceil(1/self.time_step_length))*self.reward_samples_per_timestep)

    def find_critical_points(self, data_dict: dict) -> list:
        """
        Find the critical points based on closest distance and maximum social force.

        Args:
        ----
            data_dict (dict): Dictionary containing the data.

        Returns
        -------
            list: List of critical points.

        """
        closest_distance = np.argmin(data_dict['closest_agent_distance'])
        max_social_force = np.argmin(data_dict['social_force_sfm_reward'])

        return [0, closest_distance, max_social_force, len(data_dict['robot']) - 1]

    def rotate_patch(self, patch, center, angle, ax_main):
        """
        Rotate a patch around a center point.

        Args:
        ----
            patch: The patch to rotate.
            center (tuple): The center point for rotation.
            angle (float): The angle of rotation.
            ax_main: The main axis for plotting.

        """
        trans = plt.matplotlib.transforms.Affine2D(
        ).rotate_around(center[0], center[1], angle)
        patch.set_transform(trans + ax_main.transData)

    def generate_trajectory_plot(self, data_dict: dict, closest_agent: int,
                                 max_social_force_agent: int):
        """
        Generate a plot of the robot and agent trajectories.

        Args:
        ----
            data_dict (dict): Dictionary containing the data for plotting.
            closest_agent (int): Index of the closest agent.
            max_social_force_agent (int): Index of the agent with maximum social force.

        """
        robot_odom = data_dict['robot']
        agent_odom = data_dict['agents']

        critical_points = self.find_critical_points(data_dict)
        sorted_critical_points = sorted(critical_points)

        fig = plt.figure(figsize=(16, 7))

        gs = gridspec.GridSpec(1, 2, width_ratios=[1, 1], height_ratios=[1])

        ax_plot = fig.add_subplot(gs[0, 0])
        ax_plot.set_aspect('auto')
        ax_plot.set_xlabel('Time (s)', fontdict={'fontsize': 18})
        ax_plot.set_ylabel('Metric (-)', fontdict={'fontsize': 18})
        ax_plot.tick_params(axis='both', which='major', labelsize=16)

        ax_plot.set_axisbelow(True)
        ax_plot.set_ylim(0, 3)

        # Plot the closest distance and the social force5
        ax_plot.plot(data_dict['sim_time'], data_dict['closest_agent_distance'],
                     color=self.colors["red"], label='Agent Distance [m]')
        ax_plot.plot(data_dict['sim_time'][critical_points[1]], data_dict['closest_agent_distance']
                     [critical_points[1]], marker='o', markersize=10, color=self.colors["red"],
                     label='Min Distance')

        ax_plot.plot(data_dict['sim_time'], -data_dict['social_force_sfm_reward'],
                     color=self.colors["purple"], label='Interaction Force Reward [-]')
        ax_plot.plot(data_dict['sim_time'][critical_points[2]],
                     -data_dict['social_force_sfm_reward']
                     [critical_points[2]], marker='o', markersize=10,
                     color=self.colors["purple"], label='Max Interaction Force')

        ax_plot.plot(data_dict['sim_time'], data_dict['robot_velocity'],
                     color=self.colors["med_blue"], label='Robot Velocity [m/s]')

        # ax_plot.legend(loc='upper right', fontsize=16)

        ax_map = fig.add_subplot(gs[0, 1])
        ax_map.set_aspect('auto')
        ax_map.set_axisbelow(True)

        # Calculate the extent of the image
        img_height, img_width = self.map_image.shape[:2]
        extent = [
            self.origin[0],
            self.origin[0] + img_width * self.resolution,
            self.origin[1] + img_height * self.resolution,
            self.origin[1],
        ]

        # Display the background image
        ax_map.imshow(self.map_image, extent=extent,
                      cmap='gray', origin='lower')
        # Create for the robot
        robot_size = 0.25
        wheel_width = 0.1
        wheel_height = 0.05

        # Check if robot pose is in interaction zone
        in_interaction_range = data_dict['in_interaction_range']
        robot_x_in_interaction = [pose[0] for i, pose in enumerate(
            robot_odom) if in_interaction_range[i]]
        robot_y_in_interaction = [pose[1] for i, pose in enumerate(
            robot_odom) if in_interaction_range[i]]

        robot_x_out_interaction = [pose[0]
                                   for i, pose in enumerate(robot_odom)]
        robot_y_out_interaction = [pose[1]
                                   for i, pose in enumerate(robot_odom)]

        ax_map.plot(robot_x_out_interaction, robot_y_out_interaction,
                    color=self.colors["orange"], linestyle='dashed',
                    label='Robot Trajectory', zorder=1)
        ax_map.plot(robot_x_in_interaction, robot_y_in_interaction,
                    color=self.colors["orange"], label='Robot Trajectory', zorder=2)

        agent_x_in_interaction = [[pose[0] for i, pose in enumerate(
            agent_odom[j]) if in_interaction_range[i]] for j in range(agent_odom.shape[0])]
        agent_y_in_interaction = [[pose[1] for i, pose in enumerate(
            agent_odom[j]) if in_interaction_range[i]] for j in range(agent_odom.shape[0])]

        agent_x_out_interaction = [[pose[0] for i, pose in enumerate(
            agent_odom[j])] for j in range(agent_odom.shape[0])]
        agent_y_out_interaction = [[pose[1] for i, pose in enumerate(
            agent_odom[j])] for j in range(agent_odom.shape[0])]

        # Remove from agent_x_out_interaction the points that are in interaction range

        self.node.get_logger().error(f"Agent{ agent_odom.shape}")
        for j in range(agent_odom.shape[0]):
            ax_map.plot(agent_x_out_interaction[j], agent_y_out_interaction[j],
                        color=self.colors["bright_dark_blue"], linestyle='dashed',
                        label='Agent Trajectory', zorder=1)
            ax_map.plot(agent_x_in_interaction[j], agent_y_in_interaction[j],
                        color=self.colors["bright_dark_blue"], label='Agent Trajectory', zorder=20)

        for crit_index, crit_value in enumerate(sorted_critical_points):
            alpha_patches = 0.2 + 0.8 * \
                (1 - (len(sorted_critical_points) - 1 -
                 crit_index) / (len(sorted_critical_points)))

            robot_x, robot_y, robot_yaw = (robot_odom[crit_value, 0],
                                           robot_odom[crit_value, 1], robot_odom[crit_value, 4])

            robot_square = patches.Rectangle(
                (0, 0), robot_size, robot_size, color=self.colors["orange"], fill=True,
                label='Robot', alpha=alpha_patches)
            robot_wheel_1 = patches.Rectangle(
                (0, 0), wheel_width, wheel_height, color=self.colors["rubber"], fill=True,
                alpha=alpha_patches)
            robot_wheel_2 = patches.Rectangle(
                (0, 0), wheel_width, wheel_height, color=self.colors["rubber"], fill=True,
                alpha=alpha_patches)

            robot_square.set_xy(
                (robot_x - robot_size / 2, robot_y - robot_size / 2))
            robot_wheel_1.set_xy(
                (robot_x - robot_size / 2, robot_y + robot_size / 2))
            robot_wheel_2.set_xy(
                (robot_x - robot_size / 2, robot_y - 0.05 - robot_size / 2))

            self.rotate_patch(
                robot_square, (robot_x, robot_y), robot_yaw, ax_map)
            self.rotate_patch(
                robot_wheel_1, (robot_x, robot_y), robot_yaw, ax_map)
            self.rotate_patch(
                robot_wheel_2, (robot_x, robot_y), robot_yaw, ax_map)

            # Create a list to hold agent circles
            # Assuming agent_odom has shape [M, N, 4] where N = time, M = number of agents
            num_agents = agent_odom.shape[0]
            agent_circles = [patches.Circle((0, 0), 0.13, color=self.colors["bright_dark_blue"],
                                            fill=True, label='Agent', alpha=alpha_patches)
                             for _ in range(num_agents)]

            for j in range(num_agents):
                agent_x = agent_odom[j, crit_value, 0]
                agent_y = agent_odom[j, crit_value, 1]
                agent_circles[j].set_center((agent_x, agent_y))

            for circle in agent_circles:
                ax_map.add_patch(circle)

            # Draw directional arrow from robot to agent

            if np.argmin(data_dict['closest_agent_distance']) == crit_value:
                agent_x = agent_odom[closest_agent, crit_value, 0]
                agent_y = agent_odom[closest_agent, crit_value, 1]
                dir_vector = np.array([agent_x - robot_x, agent_y - robot_y])
                dir_vector = dir_vector / np.linalg.norm(dir_vector) * 0.5

                ax_map.plot(robot_x, robot_y, 'o',
                            color=self.colors['red'], markersize=4)
                ax_map.arrow(robot_x, robot_y, dir_vector[0], dir_vector[1], head_width=0.1,
                             head_length=0.1, fc=self.colors['red'], ec=self.colors['red'])

            elif np.argmin(data_dict['social_force_sfm_reward']) == crit_value:
                agent_x = agent_odom[0, crit_value, 0]
                agent_y = agent_odom[0, crit_value, 1]
                dir_vector = np.array([agent_x - robot_x, agent_y - robot_y])
                dir_vector = dir_vector / np.linalg.norm(dir_vector) * 0.5

                ax_map.plot(robot_x, robot_y, 'o',
                            color=self.colors['purple'], markersize=4)
                ax_map.arrow(robot_x, robot_y, dir_vector[0], dir_vector[1], head_width=0.1,
                             head_length=0.1, fc=self.colors['purple'], ec=self.colors['purple'])
            else:
                agent_x = agent_odom[max_social_force_agent, crit_value, 0]
                agent_y = agent_odom[max_social_force_agent, crit_value, 1]
                dir_vector = np.array([agent_x - robot_x, agent_y - robot_y])
                dir_vector = dir_vector / np.linalg.norm(dir_vector) * 0.5

                ax_map.arrow(robot_x, robot_y, dir_vector[0], dir_vector[1], head_width=0.1,
                             head_length=0.1, fc=self.colors['green'], ec=self.colors['green'])

            ax_map.add_patch(robot_square)
            ax_map.add_patch(robot_wheel_1)
            ax_map.add_patch(robot_wheel_2)
            ax_map.set_xlim(self.origin[0] + 1.5, self.origin[0] + 10.5)
            ax_map.set_ylim(self.origin[1] + 3.0, self.origin[1] + 7.0)
            ax_map.set_axis_off()
        bbox = ax_map.get_tightbbox(fig.canvas.get_renderer())
        bbox = bbox.transformed(fig.dpi_scale_trans.inverted())

        bbox_left = ax_plot.get_tightbbox(fig.canvas.get_renderer())
        bbox_left = bbox_left.transformed(fig.dpi_scale_trans.inverted())

        plt.savefig(
            f'{self.sim_path}/trajectory_{self.sim_name}_map.png', bbox_inches=bbox)
        plt.savefig(
            f'{self.sim_path}/trajectory_{self.sim_name}_plot.png', bbox_inches=bbox_left)

        plt.tight_layout()
        # plt.savefig(f'{self.sim_path}/trajectory_{self.sim_name}.png')
        plt.show()

    def load_map(self):
        """
        Load a PGM map and return the cv2 image, resolution, and origin.

        Returns
        -------
            tuple: The map image, resolution, and origin.

        """
        package_path = get_package_share_directory('core_gazebo_world')
        map_yaml_file = self.node.get_parameter('map_yaml_file').value

        path = os.path.join(package_path, f'{map_yaml_file}.yaml')
        with open(path, 'r') as file:
            map_metadata = yaml.safe_load(file)

        # Remove last path of map_yaml_file
        map_yaml_file_dir = os.path.dirname(map_yaml_file)
        image_path = os.path.join(
            package_path, map_yaml_file_dir, (map_metadata['image']))
        map_image = cv2.imread(image_path)
        resolution = map_metadata['resolution']
        origin = map_metadata['origin']
        return map_image, resolution, origin

    def calculate_metrics(self, data_dict: dict):
        """
        Calculate all the metrics used to define the performance of the algorithm.

        Args:
        ----
            data_dict (dict): Dictionary containing the data.

        """
        human_metrics = HumanMetrics(data_dict, self.num_eval_per_scenario)
        for metric in human_metrics.metrics.keys():
            use_metric = self.metrics_to_use[metric].value
            if use_metric:
                self.metrics_to_compute[metric] = 0.0

        # Calculate all the metrics
        self.metrics_lists['time_stamps'] = [
            i * self.time_step_length for i in range(data_dict['robot'].shape[0])]
        self.node.get_logger().error(f"robot: {data_dict['robot'].shape}")
        for metric in self.metrics_to_compute.keys():
            metric_value = human_metrics.metrics[metric]()
            self.metrics_to_compute[metric] = metric_value[0]
            if len(metric_value) > 1:
                self.metrics_lists[metric] = metric_value[1]
        self.store_metrics(f'{self.sim_path}/{self.result_file}')

    def store_metrics(self, result_file):
        """
        Store the calculated metrics in a file.

        Args:
        ----
            result_file (str): Path to the result file.

        """
        list_file = result_file
        # add extension if it does not have it
        if not result_file.endswith(".txt"):
            result_file += '.txt'
            list_file += '_steps_' + str(self.sim_name) + '.txt'
        else:
            list_file = list_file[:-4]
            list_file += '_steps_' + str(self.sim_name) + '.txt'

        file_was_created = os.path.exists("metrics.txt")

        # open the file
        file = open("metrics.txt", 'a+')
        if (file is None):
            self.get_logger().error("RESULT METRICS FILE NOT CREATED! FILE: %s" % result_file)

        # if the file is new, create a header
        if not file_was_created:
            file.write('experiment_tag')
            file.write('\t')
            for m in self.metrics_to_compute.keys():
                file.write(m)
                file.write('\t')
            file.write('\n')

        # write the data
        file.write(self.sim_name)
        file.write('\t')
        for v in self.metrics_to_compute.values():
            file.write(str(v))
            file.write('\t')
        file.write('\n')
        file.close()

    def plot_reward_data(self):
        """Plot the reward data from the `reward_data.csv` file."""
        data = self.load_csv_data(
            f'{self.sim_path}/reward_data_{self.sim_name}.csv')
        # Load map data from the core_gazebo_world package

        # Extract the data
        data = data[1:]  # Skip the header

        # Column mappings for readability
        columns = {
            'total_reward_mean': 0,
            'terminal_reward_mean': 1,
            'goal_distance_reward_mean': 2,
            'social_force_sfm_reward': 3,
            'social_force_deceleration': 4,
            'social_force_evasion': 5,
            'robot_pose_x': 6,
            'robot_pose_y': 7,
            'robot_pose_theta': 8,
            'robot_pose_vel_x': 9,
            'robot_pose_vel_y': 10,
            'agent_pose_x_global': 11,
            'agent_pose_y_global': 12,
            'agent_pose_vel_x_global': 13,
            'agent_pose_vel_y_global': 14,
            'done_reason': 15,
            'in_interaction_range': 16,
            'proxemics_reward': 17,
            'sim_time': 18
        }

        # Parse numeric columns

        # Parse non-numeric columns
        data_dict = {}
        data_dict['total_reward_mean'] = np.array([np.fromstring(
            row[columns['total_reward_mean']].strip('[]'), sep=',') for row in data]).flatten()
        data_dict['terminal_reward_mean'] = np.array([np.fromstring(
            row[columns['terminal_reward_mean']].strip('[]'), sep=',') for row in data]).flatten()
        data_dict['goal_distance_reward_mean'] = np.array([np.fromstring(
            row[columns['goal_distance_reward_mean']].strip('[]'), sep=',')
            for row in data]).flatten()
        data_dict['social_force_sfm_reward'] = np.array([np.fromstring(
            row[columns['social_force_sfm_reward']].strip('[]'), sep=',')
            for row in data]).flatten()
        data_dict['social_force_deceleration'] = np.array([ast.literal_eval(
            row[columns['social_force_deceleration']]) for row in data]).reshape(-1, 2)
        data_dict['social_force_evasion'] = np.array([ast.literal_eval(
            row[columns['social_force_evasion']]) for row in data]).reshape(-1, 2)
        data_dict['robot_pose_x'] = np.array([np.fromstring(
            row[columns['robot_pose_x']].strip('[]'), sep=',') for row in data]).flatten()
        data_dict['robot_pose_y'] = np.array([np.fromstring(
            row[columns['robot_pose_y']].strip('[]'), sep=',') for row in data]).flatten()
        data_dict['robot_pose_theta'] = np.array([np.fromstring(
            row[columns['robot_pose_theta']].strip('[]'), sep=',') for row in data]).flatten()
        data_dict['robot_pose_vel_x'] = np.array([np.fromstring(
            row[columns['robot_pose_vel_x']].strip('[]'), sep=',') for row in data]).flatten()
        data_dict['robot_pose_vel_y'] = np.array([np.fromstring(
            row[columns['robot_pose_vel_y']].strip('[]'), sep=',') for row in data]).flatten()
        data_dict['proxemics_reward'] = np.array([np.fromstring(
            row[columns['proxemics_reward']].strip('[]'), sep=',') for row in data]).flatten()

        data_dict['done_reason'] = np.array(
            [ast.literal_eval(row[columns['done_reason']]) for row in data]).flatten()
        data_dict['in_interaction_range'] = np.array([ast.literal_eval(
            row[columns['in_interaction_range']]) for row in data]).flatten()
        data_dict['agent_pose_x_global'] = np.array([ast.literal_eval(
            row[columns['agent_pose_x_global']])
            for row in data]).transpose(1, 0, 2).reshape(2, -1)

        data_dict['agent_pose_y_global'] = np.array([ast.literal_eval(
            row[columns['agent_pose_y_global']])
            for row in data]).transpose(1, 0, 2).reshape(2, -1)
        data_dict['agent_pose_vel_x_global'] = np.array([ast.literal_eval(
            row[columns['agent_pose_vel_x_global']])
            for row in data]).transpose(1, 0, 2).reshape(2, -1)
        data_dict['agent_pose_vel_y_global'] = np.array([ast.literal_eval(
            row[columns['agent_pose_vel_y_global']])
            for row in data]).transpose(1, 0, 2).reshape(2, -1)
        data_dict['sim_time'] = np.array([np.fromstring(
            row[columns['sim_time']].strip('[]'), sep=',') for row in data]).flatten()

        # Robot and agent odometry arrays
        robot_odom = np.array([data_dict['robot_pose_x'], data_dict['robot_pose_y'],
                               data_dict['robot_pose_vel_x'], data_dict['robot_pose_vel_y'],
                               data_dict['robot_pose_theta']])
        agent_odom = np.array([data_dict['agent_pose_x_global'], data_dict['agent_pose_y_global'],
                              data_dict['agent_pose_vel_x_global'],
                              data_dict['agent_pose_vel_y_global']])
        agent_odom = np.transpose(agent_odom, (1, 2, 0))
        robot_odom = np.transpose(robot_odom, (1, 0))

        data_dict['robot'] = robot_odom
        data_dict['agents'] = agent_odom

        # Obtain closest distance to the agent by norm of agen pose
        agent_pose_x_robot_frame = np.array(
            data_dict['agent_pose_x_global']) - np.array(data_dict['robot_pose_x'])
        agent_pose_y_robot_frame = np.array(
            data_dict['agent_pose_y_global']) - np.array(data_dict['robot_pose_y'])
        closest_agent_distance = np.sqrt(
            agent_pose_x_robot_frame**2 + agent_pose_y_robot_frame**2)

        robot_velocity = np.sqrt(
            data_dict['robot_pose_vel_x']**2 + data_dict['robot_pose_vel_y']**2).flatten()
        data_dict['robot_velocity'] = robot_velocity

        self.calculate_metrics(data_dict)

        closest_agent = np.argmin(closest_agent_distance)
        closest_agent_2d = np.unravel_index(
            closest_agent, closest_agent_distance.shape)

        data_dict['closest_agent_distance'] = closest_agent_distance[closest_agent_2d[0]]

        # self.generate_trajectory_plot(data_dict, closest_agent_2d[0], closest_agent_2d[0])
        # self.plot_social_animation(data_dict)
        self.plot_animation_rewards(data_dict)
