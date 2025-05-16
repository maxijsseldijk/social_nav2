#!/usr/bin/env python3

import numpy as np
import math
import rclpy
from geometry_msgs.msg import Pose

# Teaching Robot Navigation Behaviors to Optimal RRT Planners
# Noé Pérez-Higueras, Fernando Caballero & Luis Merino


class HumanMetrics:
    def __init__(self, data_dict, num_eval):
        self.agents = data_dict['agents']
        self.timestep = data_dict['sim_time'][0]
        self.robot_radius = 0.125
        self.robot = data_dict['robot']
        self.social_force = data_dict['social_force_sfm_reward']
        self.at_goal = self.get_episodes(data_dict['done_reason'] == 'at_goal')
        self.terminal_reward = data_dict['terminal_reward_mean']
        self.goal_reward = data_dict['goal_distance_reward_mean']

        self.robot_linear_velocity = data_dict['robot_velocity']

        self.num_eval = num_eval

        self.metrics = {
            # N. Perez-Higueras, F. Caballero, and L. Merino, “Teaching Robot Nav-
            # igation Behaviors to Optimal RRT Planners,” International Journal of
            # Social Robotics, vol. 10, no. 2, pp. 235–249, 2018.
            'time_to_reach_goal': self.total_time,
            'path_length': self.robot_path_length,
            'cumulative_heading_changes': self.cumulative_heading_changes,
            'avg_distance_to_closest_person': self.avg_closest_person,
            'minimum_distance_to_people': self.minimum_distance_to_people,
            'maximum_distance_to_people': self.maximum_distance_to_people,
            'intimate_space_intrusions': self.intimate_space_intrusions,
            'personal_space_intrusions': self.personal_space_intrusions,
            'social_space_intrusions': self.social_space_intrusions,
            'group_intimate_space_intrusions': self.group_intimate_space_intrusions,
            'group_personal_space_intrusions': self.group_personal_space_intrusions,
            'group_social_space_intrusions': self.group_social_space_intrusions,

            'std_dev_cumulative_heading_changes': self.std_deviation_sum_cum,
            'std_sum_social_force': self.std_deviation_sum_social_force,
            'std_minimum_distance_to_people': self.std_deviation_mean_min_dist,
            'std_avg_robot_linear_speed': self.std_deviation_mean_robot_speed,
            'std_avg_robot_acceleration': self.std_deviation_mean_robot_acceleration,

            # N. Tsoi, A. Xiang, P. Yu, S. S. Sohn, G. Schwartz, S. Ramesh,
            # M. Hussein, A. W. Gupta, M. Kapadia, and M. V ́azquez, “Sean 2.0:
            # Formalizing and generating social situations for robot navigation,”
            # IEEE Robotics and Automation Letters, vol. 7, no. 4, pp. 11 047–
            # 11 054, 2022
            #   - 'Total Path Length' (meters): similar to 'path_length'
            #   - 'Path Irregularity': (radians): total rotations in the robot's
            #       traveled path greater than the total rotations in the search-based
            #       path from the starting pose.
            #   - 'Path Efficiency': (meters): ratio between robot's traveled path and
            #       geodesic distance of the search-based path from the starting pose.

            # true when the robot's final pose is within a specified distance of the goal.
            # The final distance threshold is easily adjustable by the user, but defaults
            # to 1.2m.
            'completed': self.goal_reached,
            # (meters): the closest the robot passes to the target position.
            'minimum_distance_to_target': self.minimum_goal_distance,
            # (meters): distance between the last robot position and the target position.
            'final_distance_to_target': self.final_goal_distance,
            #   - 'Robot on Person Personal Distance Violation': number of times a robot
            # approaches a person within the personal distance of the robot.
            # Similar to 'personal_space_intrusions'
            #   - 'Person on Robot Personal Distance Violation': number of times a person
            # approaches the robot within the personal distance of the robot.
            #   - 'Intimate Distance Violation': number of times the robot approached within
            # the intimate distance of a person.
            #   - 'Person on Robot Intimate Distance Violation': number of times a person
            # approaches the robot within the intimate distance of the robot.
            'robot_on_person_collision': self.robot_on_person_collision,
            'person_on_robot_collision': self.person_on_robot_collision,
            'time_not_moving': self.time_not_moving,
            # TODO: 'static_obstacle_collision': self.static_obs_collision,
            # number of times the robot collides with a static obstacle.

            # SocNavBench: A Grounded Simulation Testing Framework for Evaluating Social Navigation
            # ABHIJAT BISWAS, ALLAN WANG, GUSTAVO SILVERA,
            # AARON STEINFELD, and HENNY AD-MONI, Carnegie Mellon University
            'avg_robot_linear_speed': self.avg_robot_linear_speed,
            'avg_robot_angular_speed': self.avg_robot_angular_speed,
            'avg_acceleration': self.avg_acceleration,
            'avg_overacceleration': self.avg_overacceleration,

            # Learning a Group-Aware Policy for Robot Navigation
            # Kapil Katyal ∗1,2 , Yuxiang Gao ∗2 , Jared Markowitz 1 , Sara Pohland 3 ,
            #  Corban Rivera 1 , I-Jeng Wang 1 , Chien-Ming Huang 2
            'avg_pedestrian_velocity': self.avg_pedestrian_velocity,
            'avg_closest_pedestrian_velocity': self.avg_closest_pedestrian_velocity,

            'avg_social_force': self.avg_social_force,
            'sum_social_force': self.sum_social_force,
            'max_social_force': self.max_social_force,
            'avg_goal_reward': self.avg_goal_reward,
        }

    @staticmethod
    def euclidean_distance(pose, pose1):
        return math.sqrt((pose[0] - pose1[0])**2 + (pose[1] - pose1[1])**2)

    @staticmethod
    def get_episodes(done_reasons):
        done_reasons = np.array(done_reasons)
        true_indices = np.where(done_reasons)[0]

        # Iterate through the array and find the last element of each sequence
        last_elements = [0]

        for i in range(len(true_indices) - 1):
            if true_indices[i + 1] != true_indices[i] + 1:
                last_elements.append(true_indices[i])

        # Add the last element of the array as it is always the last of its sequence
        if len(true_indices) > 0:
            last_elements.append(true_indices[-1])
        return last_elements

    @staticmethod
    def normalize_angle(ang):
        while (ang <= -math.pi):
            ang += 2 * math.pi
        while (ang > math.pi):
            ang -= 2 * math.pi
        return ang

    def get_group_center(self, group_id, distance):
        group = []
        for agent in self.agents:
            if agent.group_id == group_id:
                pose = Pose()
                pose.position.x = agent.position.position.x + \
                    (distance * math.cos(agent.yaw))
                pose.position.y = agent.position.position.y + \
                    (distance * math.sin(agent.yaw))
                group.append(pose)

        interaction_center = Pose()
        for p in group:
            interaction_center.position.x += p.position.x
            interaction_center.position.y += p.position.y

        interaction_center.position.x = float(
            interaction_center.position.x / len(group))
        interaction_center.position.y = float(
            interaction_center.position.y / len(group))
        return interaction_center

    def indicator_function(self, norm, k):
        if k == 'intimate':
            return 1 if norm < 0.35 else 0
        elif k == 'personal':
            return 1 if 0.35 <= norm < 1.2 else 0
        elif k == 'social':
            return 1 if 1.2 <= norm < 3.5 else 0
        elif k == 'public':
            return 1 if norm >= 3.5 else 0
        else:
            return 0

    def get_time_stamps(self):
        time_list = []
        t0 = rclpy.time.Time.from_msg(self.agents[0].header.stamp)
        for a in self.agents:
            t = rclpy.time.Time.from_msg(a.header.stamp)
            dur = (t - t0).to_msg()
            s = float(dur.sec + dur.nanosec / 1e9)
            time_list.append(s)
        return time_list

    def total_time(self):
        dur = self.robot.shape[0] * self.timestep
        print('\nTime_to_reach_goal computed: %.2f secs' % dur)
        return [dur / self.num_eval]

    def robot_path_length(self):
        path_length = 0.0
        for i in range(len(self.robot) - 1):
            path_length += self.euclidean_distance(
                self.robot[i + 1], self.robot[i])
        print('Path_length computed: %.2f m' % path_length)
        return [path_length / self.num_eval]

    def cumulative_heading_changes(self):
        chc_list = [0.0]
        chc = 0
        for i in range(len(self.robot) - 1):
            norm = self.normalize_angle(
                self.robot[i][4] - self.robot[i + 1][4])
            if norm < 0.0:
                norm *= -1
            chc += norm
            chc_list.append(norm)

        print('Cumulative_heading_changes: %.2f rads' % chc)
        return [chc / self.num_eval, chc_list]

    def avg_closest_person(self):
        min_dist_list = []
        avg_dist = 0
        for i in range(self.robot.shape[0]):
            min_dist = 10000
            for agent in self.agents:
                d = self.euclidean_distance(
                    self.robot[i], agent[i]) - 2 * self.robot_radius
                if d < min_dist:
                    min_dist = d
                    if min_dist < 0.0:
                        min_dist = 0.0
            if self.agents.shape[1] > 0:
                avg_dist += min_dist
                min_dist_list.append(min_dist)

        avg_dist = avg_dist / len(self.robot)
        print('Average_closest_person: %.2f m' % avg_dist)
        return [avg_dist, min_dist_list]

    def minimum_distance_to_people(self):
        min_distance = list()
        for i in range(self.robot.shape[0]):
            agent_distances = list()
            for agent in self.agents:
                d = self.euclidean_distance(
                    self.robot[i], agent[i]) - 2 * self.robot_radius
                if d < 0.0:
                    d = 0.0
                agent_distances.append(d)

            min_distance.append(min(agent_distances))
        min_dist = min(min_distance)
        print('Minimum_distance_to_people: %.2f m' % min_dist)
        return [min_dist, min_distance]

    def maximum_distance_to_people(self):
        max_distance = list()
        for i in range(self.robot.shape[0]):
            for agent in self.agents:
                max_distance.append(self.euclidean_distance(
                    self.robot[i], agent[i]) - self.robot_radius)

        max_dist = max(max_distance)
        print('Maximum_distance_to_people: %.2f m' % max_dist)
        return [max_dist]

    def space_intrusions(self, k):
        space_intrusions = 0
        space_intrusions_list = [0] * len(self.robot)

        for i in range(self.robot.shape[0]):
            min_dist = 10000
            for agent in self.agents:
                d = self.euclidean_distance(
                    self.robot[i], agent[i]) - 2 * self.robot_radius
                if d < min_dist:
                    min_dist = d
                    if min_dist < 0.0:
                        min_dist = 0.0
            indicator = self.indicator_function(min_dist, k)
            if indicator == 1:
                space_intrusions += 1
                space_intrusions_list[i] = 1

        space_intrusions = space_intrusions / len(self.robot)
        percentage = space_intrusions * 100.0

        return percentage, space_intrusions_list

    def intimate_space_intrusions(self):
        percentage, slist = self.space_intrusions('intimate')
        print('Intimate_space_intrusions: %.2f %% of the total time' % percentage)
        return [percentage, slist]

    def personal_space_intrusions(self):
        percentage, slist = self.space_intrusions('personal')
        print('Personal_space_intrusions: %.2f %% of the total time' % percentage)
        return [percentage, slist]

    def social_space_intrusions(self):
        percentage, slist = self.space_intrusions('social')
        print('Social_space_intrusions: %.2f %% of the total time' % percentage)
        return [percentage, slist]

    def detect_groups(self):
        group_ids = []
        for a in self.agents[0].agents:
            if a.group_id != -1 and a.group_id not in group_ids:
                group_ids.append(a.group_id)
        return group_ids

    def group_space_intrusions(self, k):
        group_ids = self.detect_groups()
        if len(group_ids) == 0:
            return [0.0]

        d = 1.5
        space_intrusions = 0
        group_list = [0] * len(self.robot)
        for i in range(self.robot.shape[0]):
            min_dist = 10000
            for id in group_ids:
                group_center = self.get_group_center(id, d)
                dist = self.euclidean_distance(
                    self.robot[i].position, group_center.position) - self.robot[i].radius
                if dist < min_dist:
                    min_dist = dist
            indicator = self.indicator_function(min_dist, k)
            if indicator == 1:
                space_intrusions += 1
                group_list[i] = 1

        space_intrusions = space_intrusions / len(self.robot)
        percentage = space_intrusions * 100.0

        return [percentage, group_list]

    def group_intimate_space_intrusions(self):
        r = self.group_space_intrusions('intimate')
        print(
            'Group_intimate_space_intrusions: %.2f %% of the total time' % r[0])
        return r

    def group_personal_space_intrusions(self):
        r = self.group_space_intrusions('personal')
        print(
            'Group_personal_space_intrusions: %.2f %% of the total time' % r[0])
        return r

    def group_social_space_intrusions(self):
        r = self.group_space_intrusions('social')
        print('Group_social_space_intrusions: %.2f %% of the total time' %
              r[0])
        return r

    def collisions(self):
        def calculate_angle_diff(robot, agent, agent_yaw):
            nrx = (robot[0] - agent[0]) * math.cos(agent_yaw) + \
                (robot[1] - agent[1]) * math.sin(agent_yaw)
            nry = -(robot[0] - agent[0]) * math.sin(agent_yaw) + \
                (robot[1] - agent[1]) * math.cos(agent_yaw)
            return math.atan2(nry, nrx)

        def calculate_robot_angle_diff(robot, agent):
            nrx = (agent[0] - robot[0]) * math.cos(robot[4]) + \
                (agent[1] - robot[1]) * math.sin(robot[4])
            nry = -(agent[0] - robot[0]) * math.sin(robot[4]) + \
                (agent[1] - robot[1]) * math.cos(robot[4])
            return math.atan2(nrx, nry)

        robot_coll_list = [0] * len(self.robot)
        person_coll_list = [0] * len(self.robot)
        robot_collisions = 0
        person_collisions = 0

        for i in range(self.robot.shape[0]):
            for agent in self.agents:
                distance = self.euclidean_distance(
                    self.robot[i], agent[i]) - 2 * self.robot_radius
                if distance < 0.2:
                    agent_yaw = math.atan2(agent[i][3], agent[i][2])
                    alpha = calculate_angle_diff(
                        self.robot[i], agent[i], agent_yaw)
                    alpha2 = calculate_robot_angle_diff(
                        self.robot[i], agent[i])

                    robot_speed = self.robot[i][3]
                    agent_speed = (agent[i][3]**2 + agent[i][2]**2)**0.5

                    if abs(alpha) < abs(alpha2) and robot_speed > agent_speed:
                        robot_collisions += 1
                        robot_coll_list[i] = 1
                    elif abs(alpha) > abs(alpha2) and robot_speed < agent_speed:
                        person_collisions += 1
                        person_coll_list[i] = 1
                    elif abs(alpha) < abs(alpha2) and robot_speed < agent_speed:
                        robot_collisions += 1
                        robot_coll_list[i] = 1
                    elif abs(alpha) > abs(alpha2) and robot_speed > agent_speed:
                        person_collisions += 1
                        person_coll_list[i] = 1
                    elif abs(alpha) == abs(alpha2) and robot_speed == agent_speed:
                        robot_collisions += 1
                        person_collisions += 1
                        robot_coll_list[i] = 1
                        person_coll_list[i] = 1

        return robot_collisions, person_collisions, robot_coll_list, person_coll_list

    def robot_on_person_collision(self):
        collision = self.collisions()
        print('Robot_on_person_collision: %i ' % collision[0])
        return [collision[0], collision[2]]

    def person_on_robot_collision(self):
        collision = self.collisions()
        print('Person_on_robot_collision: %i' % collision[1])
        return [collision[1], collision[3]]

    def time_not_moving(self):
        not_moving = [0] * len(self.robot)
        time_step = self.timestep

        count = 0
        for index, robot_instance in enumerate(self.robot):
            if robot_instance[2] < 0.01 and abs(robot_instance[3] < 0.02):
                count += 1
                not_moving[index] = 1
        time_stopped = time_step * count
        print('Time stopped: %i secs' % time_stopped)
        return [time_stopped, not_moving]

    def goal_reached(self):
        mind = 0.0
        if len(self.robot[-1].goals):
            for g in self.robot[-1].goals:
                d = self.euclidean_distance(
                    self.robot[-1].position, g) - self.robot[-1].goal_radius
                if d < mind:
                    return [True]
        return [False]

    def final_goal_distance(self):
        min_dist = 10000
        if len(self.robot[-1].goals):
            for g in self.robot[-1].goals:
                d = self.euclidean_distance(self.robot[-1].position, g)
                if d < min_dist:
                    min_dist = d
        return [min_dist]

    def minimum_goal_distance(self):
        min_dist = 10000
        for r in self.robot:
            if len(r.goals):
                for g in r.goals:
                    d = self.euclidean_distance(r.position, g)
                    if d < min_dist:
                        min_dist = d
        return [min_dist]

    def avg_robot_linear_speed(self):
        speed_list = []
        speed = 0
        for i in range(len(self.robot) - 1):
            speed_list.append(self.robot_linear_velocity[i])
            speed += self.robot_linear_velocity[i]

        speed = speed / len(self.robot)
        print('Average_robot_speed: %.2f m/s' % speed)
        return [speed, speed_list]

    def avg_robot_angular_speed(self):
        speed_list = []
        speed = 0
        for i in range(len(self.robot) - 1):
            speed_list.append(self.robot_linear_velocity[i])
            speed += self.robot_linear_velocity[i]

        speed = speed / len(self.robot)
        # print('Average_angular_robot_speed: %.2f rad/s' % speed)
        return [speed, speed_list]

    def avg_acceleration(self):
        acceleration = 0
        acceleration_list = [0.0]
        for i in range(len(self.robot) - 1):
            dv = self.robot_linear_velocity[i +
                                            1] - self.robot_linear_velocity[i]
            dt = self.timestep
            if dt != 0.0:
                accel = dv / dt
                acceleration += np.abs(accel)
                acceleration_list.append(np.abs(accel))
            else:
                acceleration_list.append(0.0)

        acceleration = acceleration / len(self.robot)
        print('Average_robot_acceleration: %.5f m/s^2' % acceleration)
        return [acceleration, acceleration_list]

    def avg_overacceleration(self):
        jerk = 0
        jerk_list = [0.0]
        for i in range(len(self.robot) - 1):
            dv = self.robot[i + 1][2] - self.robot[i][2]
            dt = self.timestep
            if dt != 0.0:
                acceleration = dv / dt
                jerk += np.abs(acceleration / dt)
                jerk_list.append(acceleration / dt)
            else:
                jerk_list.append(0.0)

        jerk = jerk / len(self.robot)
        print('Average_robot_jerk(over_acceleration): %.5f m/s^3' % jerk)
        return [jerk, jerk_list]

    def avg_pedestrian_velocity(self):
        speed = 0
        speed_list = []
        for i in range(self.agents.shape[1]):
            speed2 = 0.0
            for agent in self.agents:
                speed += np.linalg.norm(agent[i][2:4])
                speed2 += np.linalg.norm(agent[i][2:4])
            speed_list.append(speed2 / self.agents.shape[0])

        speed = speed / (self.agents.shape[1] * self.agents.shape[0])
        print('Average_Pedestrian_speed: %.2f m/s' % speed)
        return [speed, speed_list]

    def avg_closest_pedestrian_velocity(self):
        speed = 0
        speed_list = []
        for i in range(self.robot.shape[0]):
            min_dist = 10000
            closest_index = np.nan
            for agent_index, agent in enumerate(self.agents):
                d = self.euclidean_distance(self.robot[i], agent[i])
                if d < min_dist:
                    min_dist = d
                    closest_index = agent_index
                    if min_dist < 0.0:
                        min_dist = 0.0

            speed += np.linalg.norm(self.agents[closest_index][i][2:4])
            speed_list.append(self.agents[closest_index][i][2])

        speed = speed / self.robot.shape[0]
        print('Speed average closest person: %.2f m/s' % speed)
        return [speed, speed_list]

    def avg_social_force(self):
        social_force = 0
        social_force_list = []
        for i in range(self.robot.shape[0]):
            social_force += self.social_force[i]
            social_force_list.append(self.social_force[i])

        social_force = social_force / len(self.robot)
        print('Average_social_force: %.2f N' % social_force)
        return [social_force, social_force_list]

    def avg_goal_reward(self):
        reward = 0
        reward_list = []
        for i in range(self.robot.shape[0]):
            reward += self.goal_reward[i]
            reward_list.append(self.goal_reward[i])

        reward = reward / len(self.robot)
        print('Average_goal_reward: %.2f' % reward)
        return [reward, reward_list]

    def max_social_force(self):
        max_force = min(self.social_force)
        print('Max_social_force: %.2f N' % max_force)
        return [max_force]

    def sum_social_force(self):
        sum_force = sum(self.social_force)
        print('Sum_social_force: %.2f N' % sum_force)
        return [sum_force / self.num_eval, self.social_force]

    def std_deviation_sum(self, data):
        # Calculate the standard deviation on an episode by finding where
        #  the at_goal is specified for data['done_reason']
        data = np.array(data)
        goal_indices = self.at_goal
        sum_episodes = []
        for i in range(len(goal_indices) - 1):
            sum_episodes.append(
                np.sum(data[goal_indices[i]:goal_indices[i + 1]]))
        print('Sum_episodes: ', sum_episodes)
        print('Mean: %.2f' % np.mean(sum_episodes))
        std = np.std(sum_episodes)
        print('Standard deviation: %.2f' % std)
        return [std]

    def std_deviation_mean(self, data):

        data = np.array(data)
        goal_indices = self.at_goal
        mean_episodes = []
        for i in range(len(goal_indices) - 1):
            mean_episodes.append(
                np.mean(data[goal_indices[i]:goal_indices[i + 1]]))
        print('Mean_episodes: ', mean_episodes)
        print('Mean: %.2f' % np.mean(mean_episodes))
        std = np.std(mean_episodes)
        print('Standard deviation: %.2f' % std)
        return [std]

    def std_deviation_mean_min(self, data):
        data = np.array(data)
        goal_indices = self.at_goal
        mean_episodes = []
        for i in range(len(goal_indices) - 1):
            mean_episodes.append(
                np.min(data[goal_indices[i]:goal_indices[i + 1]]))
        print('Mean_episodes: ', mean_episodes)
        print('Mean_min_distance: %.2f' % np.mean(mean_episodes))
        std = np.std(mean_episodes)
        print('Standard deviation: %.2f' % std)
        return [std]

    def std_deviation_sum_social_force(self):
        return self.std_deviation_sum(self.social_force)

    def std_deviation_mean_min_dist(self):
        return self.std_deviation_mean_min(self.minimum_distance_to_people()[1])

    def std_deviation_mean_robot_speed(self):
        return self.std_deviation_mean(self.robot_linear_velocity)

    def std_deviation_mean_robot_acceleration(self):
        return self.std_deviation_mean(self.avg_acceleration()[1])

    def std_deviation_sum_cum(self):
        return self.std_deviation_sum(self.cumulative_heading_changes()[1])

    def avg_pedestrian_angle(self):
        pass

    def path_irregularity(self):
        pass

    def path_efficiency(self):
        pass

    def static_obs_collision(self):
        pass
