import time
from tkinter import W
from socket import *
import torch
from gym.spaces import MultiDiscrete
from.airsim_socket import CustomAirsimClient
from .drone_dynamics import DroneDynamicsAirsim
import airsim
import gym
from gym import spaces
import numpy as np
from pathlib import Path
from os import remove
import math
import cv2

# 可视化
from PyQt5 import QtCore
from PyQt5.QtCore import pyqtSignal

# 继承gym的环境。用gym的框架套住airsim


def judge_dis(point, agent, dis):
    if math.sqrt(pow(point[0]-agent.x, 2) + pow(point[1] - agent.y, 2)) < dis:
        return True
    else:
        return False


def plot_target(name, client, color=None):
    if color is None:
        color = [1.0, 0.0, 0.0, 1.0]
    getpos = client.simGetObjectPose(name)
    pos = [getpos.position.x_val, getpos.position.y_val, getpos.position.z_val]
    a = [airsim.Vector3r(pos[0] - 30, pos[1], -50)]
    b = [airsim.Vector3r(pos[0], pos[1] - 30, -50)]
    c = [airsim.Vector3r(pos[0] + 30, pos[1], -50)]
    d = [airsim.Vector3r(pos[0], pos[1] + 30, -50)]
    client.simPlotLineList(a + b + b + c + c + d + d + a, thickness=50.0, duration=0.5,
                              color_rgba=color,
                              is_persistent=False)



class AirSimDroneEnv(gym.Env, QtCore.QThread):
    # action_signal = pyqtSignal(int, np.ndarray) # action: [v_xy, v_z, yaw_rate])
    # state_signal = pyqtSignal(int, np.ndarray) # state_raw: [dist_xy, dist_z, relative_yaw, v_xy, v_z, yaw_rate]
    # attitude_signal = pyqtSignal(int, np.ndarray, np.ndarray) # attitude (pitch, roll, yaw  current and command)
    # reward_signal = pyqtSignal(int, float, float) # step_reward, total_reward
    # pose_signal = pyqtSignal(np.ndarray, np.ndarray, np.ndarray, np.ndarray)  # goal_pose start_pose current_pose trajectory
    def __init__(self, cfg) -> None:
        super().__init__()
        self.goal_choose_name = None
        self.socket_server = socket(AF_INET, SOCK_STREAM)
        self.socket_server.bind((cfg.get('options', 'socket_server_ip'), cfg.getint('options', 'socket_server_port')))
        self.socket_server.listen(200)  # 最大连接数
        # ip_list =cfg.get('options', 'ip')

        self.client = CustomAirsimClient(cfg.get('options', 'ip').split(','), self.socket_server)
        time.sleep(3)
        np.set_printoptions(formatter={'float': '{: 4.2f}'.format}, suppress=True)
        torch.set_printoptions(profile="short", sci_mode=False, linewidth=1000)
        print("init airsim-gym-env.")
        #conig
        self.current_step = 0
        # self.max_episode_steps = 1000
        self.model = None
        self.data_path = None
        self.shared_reward = False
        self.cfg = cfg
        self.navigation_3d = cfg.getboolean('options', 'navigation_3d')
        self.velocity_step = cfg.getfloat('options', 'velocity_step')
        self.acc_step = cfg.getfloat('options', 'acc_step')
        self.yaw_step = cfg.getfloat('options', 'yaw_step')
        self.vz_step = cfg.getfloat('options', 'vz_step')
        self.az_step = cfg.getfloat('options', 'az_step')
        self.depth_image_request = airsim.ImageRequest('0', airsim.ImageType.DepthPerspective, True, False)
        self.acc_xy_max = cfg.getfloat('multirotor', 'acc_xy_max')
        self.v_xy_max = cfg.getfloat('multirotor', 'v_xy_max')
        self.fixed_v_xy = cfg.getfloat('multirotor', 'v_xy_max')
        self.v_xy_min = cfg.getfloat('multirotor', 'v_xy_min')
        self.v_z_max = cfg.getfloat('multirotor', 'v_z_max')
        self.yaw_rate_max_deg = cfg.getfloat('multirotor', 'yaw_rate_max_deg')
        self.yaw_rate_max_rad = math.radians(self.yaw_rate_max_deg)
        # self.init_yaw_degree = np.array(cfg.get('options', 'init_yaw_degree').split()).astype(int)

        self.max_vertical_difference = 5
        self.dt = cfg.getfloat('multirotor', 'dt')
        self.env_name = cfg.get('options', 'env_name')
        self.num_of_drones = cfg.getint('options', 'num_of_drone')
        if self.navigation_3d:
            self.state_feature_length = 5 * self.num_of_drones
        else:
            self.state_feature_length = 3 * self.num_of_drones
        self.axy_n = np.zeros(self.num_of_drones)
        self.vxy_n = np.zeros(self.num_of_drones)
        self.yaw_acc_n = np.zeros(self.num_of_drones)
        self.yaw_sp_n = np.zeros(self.num_of_drones)
        self.vx_n = self.vy_n = self.v_z_n = np.zeros(self.num_of_drones)
        self.ax_n = self.ay_n = self.a_z_n = np.zeros(self.num_of_drones)
        self.keyboard_debug = cfg.getboolean('options', 'keyboard_debug')
        self.generate_q_map = cfg.getboolean('options', 'generate_q_map')
        self.world_clock = cfg.getint('options', 'world_clock')
        self.discrete_grid_x = cfg.getint('options', 'discrete_grid_x')
        self.discrete_grid_y = cfg.getint('options', 'discrete_grid_y')
        print('Environment: ', self.env_name, "num_of_drones: ", self.num_of_drones)
        self.agents = []
        # self.client = airsim.MultirotorClient(ip=cfg.get('options', 'ip'))
        self.trajectory_list = []
        self.coverage_area = np.zeros(self.discrete_grid_x * self.discrete_grid_x)
        self.cur_state = None
        self.init_pose = []
        self.vehicle_names = self.client.listVehicles()
        self.client.reset()
        for i in range(self.num_of_drones):
            self.agents.append(DroneDynamicsAirsim(self.cfg, self.client, i + 1, self.vehicle_names[i]))
            self.trajectory_list.append([])
            self.agents[i].reset(np.random.randint(0, 360), 0)
            self.init_pose.append(self.client.simGetObjectPose(self.agents[i].name).position)

        # TODO cfg
        self.work_space_x = [-1648, 44]
        self.work_space_y = [-1438, 564]
        self.work_space_z = [-100, 100]
        self.work_space_x_length = self.work_space_x[1] - self.work_space_x[0]
        self.work_space_y_length = self.work_space_y[1] - self.work_space_y[0]
        self.work_space_z_length = self.work_space_z[1] - self.work_space_z[0]
        self.max_episode_steps = 2000

        # trainning state
        self.episode_num = 0
        self.total_step = 0
        self.step_num = 0
        self.cumulated_episode_reward = np.zeros(self.num_of_drones)
        self.previous_distance_from_des_point = []
        self.goal_position = [0, 0, 0]

        # other settings
        self.avoid_distance = cfg.getint('environment', 'crash_distance') / 200
        self.accept_radius = cfg.getint('environment', 'accept_radius')
        self.max_depth_meters = cfg.getint('environment', 'max_depth_meters')

        self.screen_height = cfg.getint('environment', 'screen_height')
        self.screen_width = cfg.getint('environment', 'screen_width')
        self.observation_space = []
        self.share_observation_space = []
        self.goal_name = []
        self.SceneObjects = self.client.client.simListSceneObjects()
        self.goal_num = 0
        for each in self.SceneObjects:
            if 'people' in each:
                self.goal_name.append(each)
                self.goal_num += 1
        self.action_space = []
        for each in range(self.num_of_drones):
            self.observation_space.append(spaces.Box(
                low=0, high=1, shape=(29, ), dtype=np.float32))
            self.action_space.append(self.agents[0].action_space)
        self.share_observation_space = [spaces.Box(
            low=0, high=1, shape=(29 * self.num_of_drones, ),
            dtype=np.float32) for _ in range(self.num_of_drones)]
        self.pre_obs_n = []
        self.pre_reward = []
        # for i, agent in enumerate(self.agents):
        #     self.pre_obs_n.append(self._get_obs(agent))
        self.vxy_n = np.array([self.v_xy_max] * self.num_of_drones)


    def seed(self, seed=None):
        if seed is None:
            np.random.seed(1)
        else:
            np.random.seed(seed)
    #
    # def fly_forward(self, agent):


    def step(self, action_n):
        self.current_step += 1
        obs_n = []
        reward_n = []
        done_n = []
        info_n = []
        plot_target(self.goal_name[self.goal_choose_name], self.client)
        # set action for each agent

        self.compute_velocity(action_n, self.pre_obs_n)
        # self.client.simPause(False)
        # a = time.time()

        # self.client.simPause(False)
        fi = []
        for i, agent in enumerate(self.agents):
            if not agent.is_crash:
                fi.append(self._set_action(i, agent))
        # for f in fi:
        #     f.join()
        # self.client.simPause(True)
        # b = time.time()
        # print('flying_using_time:',b - a)

        # if i % int(1 // self.dt) == 0:
        #     self.trajectory_list[i].append(agent.get_position())

        # time.sleep(0.1 / self.world_clock)
        # self.client.simPause(Tr
        # ue)
        # advance world state
        # self.world.step()  # core.step()
        # record observation for each agent
        self.cur_state = self.get_all_state()
        for i, agent in enumerate(self.agents):
            agent.x = self.cur_state[i].position.x_val
            agent.y = self.cur_state[i].position.y_val
            agent.z = self.cur_state[i].position.z_val
            _ = self.is_crashed(agent)
            obs_n.append(self._get_obs(agent))
            # print(obs_n[i])
            done_n.append(self._get_done(agent))
            reward_n.append(self._get_reward(obs_n[i], agent, action_n[i]))
            info = self._get_info(agent, reward_n[i])
            if done_n[i]:
                print(info)
            info_n.append(info)
            # else:
            #     obs_n.append(self.observation_space[i].low)
            #     # print(obs_n[i])
            #     done_n.append(False)
            #     reward_n.append(0)
            #     info_n.append('None')

        # for flag in done_n:
        #     if flag:
        #         for each in range(len(done_n)):
        #             done_n[each] = True

        self.cumulated_episode_reward += np.array(reward_n)

        """all agents get total reward in cooperative case, if shared reward,
        all agents have the same reward, and reward is sum"""
        reward = np.sum(reward_n)
        if self.shared_reward:
            reward_n = [[reward]] * self.num_of_drones

        # TODO debug 部分摧毁直接结束场景
        # if sum(done_n) > 10:
        #     for i in range(len(done_n)):
        #         done_n[i] = 1
        # print('api_using_time:', time.time() - b)
        self.pre_obs_n = obs_n
        self.pre_reward = reward_n
        return obs_n, reward_n, done_n, info_n

    def get_all_state(self):
        cur_state = []
        for each in self.agents:
            tmp = self.client.getMultirotorState(each.name).kinematics_estimated
            # tmp.position += self.init_pose[each.id - 1]
            pose = self.client.simGetObjectPose(each.name)
            plot_target(each.name, self.client, [0.0, 1.0, 0.0, 1.0])
            tmp.position = pose.position
            cur_state.append(tmp)
        return cur_state

    def compute_velocity(self, actions, obs_n):
        action_n = actions.copy()
        # action_n = np.array(action_n)
        action_n -= 5
        self.yaw_acc_n = np.radians(action_n[:, -1] * self.yaw_step)
        yaw_n = []
        for each in range(0, self.num_of_drones):
            yaw_n.append(self.agents[each].get_attitude()[2])
        yaw_n = np.array(yaw_n)
        self.yaw_sp_n = yaw_n + self.yaw_acc_n * self.dt

        for each in range(self.num_of_drones):
            relative_yaw = self.agents[each].get_relative_yaw()
            distance_id = str(int(np.degrees(relative_yaw) // 10))
            if self.client.getDistanceSensorData("Distance_" + distance_id, self.agents[each].name).distance / \
                    200 > self.avoid_distance - 0.05:
                self.agents[each].avoid_state = 0
                self.agents[each].train_flag = 0
                actions[each] = np.clip(relative_yaw // np.radians(self.yaw_step), -5, 5)
                self.yaw_sp_n[each] = relative_yaw + \
                                      airsim.to_eularian_angles(self.cur_state[each].orientation)[2]
                actions[each] += 5
                continue
            if obs_n[each][0] < self.avoid_distance:
                self.agents[each].train_flag = 1
                self.vxy_n[each] = self.fixed_v_xy
                self.agents[each].avoid_state = 1
                # self.yaw_sp_n[each] += relative_yaw / 2
            else:
                self.vxy_n[each] = self.fixed_v_xy
                self.agents[each].avoid_state = 2
                if self.agents[each].avoid_state:
                    self.yaw_sp_n[each] = self.agents[each].get_attitude()[2]
                    actions[each] = 5
                else:
                    actions[each] = np.clip(relative_yaw // np.radians(self.yaw_step), -5, 5)
                    self.yaw_sp_n[each] = relative_yaw + \
                                          airsim.to_eularian_angles(self.cur_state[each].orientation)[2]
                    actions[each] += 5

        self.yaw_sp_n[self.yaw_sp_n > math.radians(180)] -= math.pi * 2
        self.yaw_sp_n[self.yaw_sp_n < math.radians(-180)] += math.pi * 2
        if self.current_step < 5:
            self.vxy_n[0] = self.v_xy_max * self.current_step / 5
        self.vx_n = self.vxy_n * np.cos(self.yaw_sp_n)
        self.vy_n = self.vxy_n * np.sin(self.yaw_sp_n)


    # set env action for a particular agent
    def _set_action(self, i, agent):
        if self.navigation_3d:
            return self.client.moveByVelocityAsync(self.vx_n[i], self.vy_n[i], -self.v_z_n[i], self.dt,
                                            vehicle_name=agent.name,
                                            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                                            yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=math.degrees(
                                               self.yaw_sp_n[i])))
        else:
            if agent.wait_step > 0:
                agent.wait_step -= 1
                return self.client.hoverAsync(agent.name)
            else:
                return self.client.moveByVelocityZAsync(self.vx_n[i],  self.vy_n[i], - agent.start_position[2], self.dt,
                                                        vehicle_name=agent.name,
                                                        drivetrain=airsim.DrivetrainType.ForwardOnly,
                                                        yaw_mode=airsim.YawMode(is_rate=False))
            # , yaw_or_rate = math.degrees(self.yaw_sp_n[i])
            # , yaw_or_rate = math.degrees(self.yaw_sp_n[i])
            # print(self.vx_n, self.vy_n, math.degrees(self.yaw_sp_n[0]))

    def _get_obs_with_state(self, agent):
        image = self.get_depth_image(agent)
        image_resize = cv2.resize(image, (self.screen_width, self.screen_height))
        image_scaled = image_resize * 100
        self.min_distance_to_obstacles = image_scaled.min()
        image_scaled = -np.clip(image_scaled, 0, self.max_depth_meters) / self.max_depth_meters * 255 + 255
        image_uint8 = image_scaled.astype(np.uint8)

        assert image_uint8.shape[0] == self.screen_height and image_uint8.shape[
            1] == self.screen_width, 'image size not match'

        # 2. get current state (relative_pose, velocity)
        state_feature_array = np.zeros((self.screen_height, self.screen_width))
        state_feature = agent._get_state_feature()

        assert (self.state_feature_length == state_feature.shape[
            0]), 'state_length {0} is not equal to state_feature_length {1}' \
            .format(self.state_feature_length, state_feature.shape[0])
        state_feature_array[0, 0:self.state_feature_length] = state_feature

        # 3. generate image with state
        image_with_state = np.array([image_uint8, state_feature_array])
        # image_with_state = image_with_state.swapaxes(0, 2)
        # image_with_state = image_with_state.swapaxes(0, 1)

        return image_with_state.reshape(-1,)

    def _get_obs(self, agent):
        # if agent.is_crash:
        #     return np.ones(9)
        # index = agent.id - 1
        # yaw_rate_sp = self.cur_state[index].angular_velocity.z_val
        # yaw_rate_norm = (yaw_rate_sp / self.yaw_rate_max_rad / 2 + 0.5)

        # x_norm = (self.cur_state[index].position.x_val - self.work_space_x[0]) / (
        #             self.work_space_x[1] - self.work_space_x[0])
        # y_norm = (self.cur_state[index].position.y_val - self.work_space_y[0]) / (
        #             self.work_space_y[1] - self.work_space_y[0])
        # state_feature = np.array([x_norm, y_norm, yaw_rate_norm])
        distance_sensors = []
        for each in range(36):
            distance_sensors.append(
                self.client.getDistanceSensorData("Distance" + str(each), agent.name).distance)
        distance_fblr_norm = np.array([distance_sensors[0:9],
                                       distance_sensors[18:27],
                                       distance_sensors[27:36]]) / 200

        # yaw = airsim.to_eularian_angles(self.cur_state[agent.id - 1].orientation)[2]
        # yaw_norm = yaw / math.pi / 2 + 0.5
        # effective_direction_norm_min = effective_direction_min / 360
        # effective_direction_norm_max = effective_direction_max / 360
        # tmp = np.append(state_feature,
        #                 [yaw_norm] + distance_fblr_norm).clip(0, 1)
        # distance = agent.get_distance_to_goal_2d()
        # distance_norm = distance / agent.goal_distance
        relative_yaw = agent.get_relative_yaw()
        relative_yaw_norm = (relative_yaw / math.pi / 2 + 0.5)
        tmp = np.append(distance_fblr_norm, relative_yaw_norm).clip(0, 1)
        # agent.
        # goal_norm = [0, 0]
        # goal_norm[0] = (self.goal_position[0] - self.work_space_x[0]) / (self.work_space_x[1] - self.work_space_x[0])
        # goal_norm[1] = (self.goal_position[1] - self.work_space_y[0]) / (self.work_space_y[1] - self.work_space_y[0])
        # goal_norm[2] = (self.goal_position[2] - self.work_space_z[0]) / (self.work_space_z[1] - self.work_space_z[0])
        relative_dis_norm = math.sqrt(pow(self.goal_position[0] - agent.x, 2) +
                                      pow(self.goal_position[1] - agent.x, 2)) / agent.goal_distance
        return np.append(tmp, relative_dis_norm).clip(0, 1)

    def is_duplicate(self, agent, index):
        if not self.coverage_area[index] or self.coverage_area[index] == agent.id:
            return False
        else:
            return True

    def is_new_area(self, agent, index):

        if not self.coverage_area[index]:
            self.coverage_area[index] = agent.id
            return True
        else:
            return False

    def _get_reward(self, obs, agent, action):
        if not agent.train_flag:
            distance_now = agent.get_distance_to_goal_2d()
            agent.previous_distance_from_des_point = distance_now
            return 0
        reward = 0

        if min(obs[0:9]) < self.avoid_distance - 0.15:
            # if agent.last_min_distance != 0:
            #     distance_change = agent.last_min_distance-min_distance
            #     reward -= distance_change * 4000
            # else:
            reward -= np.power((self.avoid_distance-min(obs[0:9])) * 5, 2) * 10

        # if agent.avoid_state == 1 and obs[0] > self.avoid_distance:
        #     reward += 2
            # agent.last_min_distance = min_distance
        # else:
        #     agent.last_min_distance = 0
        distance_now = agent.get_distance_to_goal_2d()
        reward_distance = (agent.previous_distance_from_des_point - distance_now) * 0.1
        # normalized to 100 according to goal_distance
        agent.previous_distance_from_des_point = distance_now
        self.is_crashed(agent)
        reward += reward_distance
        if agent.is_in_desired_pose():
            reward = 5000
        # print("reward_distance: ",reward_distance)
        # print("reward: ", reward)
        # if reward == -10:
        #     return self._get_reward( obs, agent, action)
        return reward

    def _get_done(self, agent):
        # is_not_inside_workspace_now = self.is_not_inside_workspace(agent)
        has_reached_des_pose = agent.is_in_desired_pose()

        # We see if we are outside the Learning Space
        episode_done = has_reached_des_pose or \
                       agent.is_crash or \
                       self.current_step >= self.max_episode_steps
        return episode_done

    def is_crashed(self, agent):
        is_crashed = False
        collision_info = self.client.simGetCollisionInfo(vehicle_name=agent.name)
        if collision_info.has_collided:
            is_crashed = True
            agent.is_crash = True
        return is_crashed

    def _get_info(self, agent, reward):
        info = {
            'id': agent.id,
            # 'is_success': agent.is_in_desired_pose(),
            'is_crash': agent.is_crash,
            'is_not_in_workspace': self.is_not_inside_workspace(agent),
            'step_num': self.current_step,
            'individual_reward': reward,
            'has_reached_des_pose': agent.is_in_desired_pose()
        }
        return info

    def reset(self):

        # reset state
        self.client.reset()
        # self.goal_position = np.random.randn(3)*100
        self.goal_choose_name = np.random.randint(self.goal_num)
        pos = self.client.simGetObjectPose(self.goal_name[self.goal_choose_name])
        self.goal_position = [pos.position.x_val, pos.position.y_val, pos.position.z_val]
        print(self.goal_name[self.goal_choose_name])
        # self.init_pose = []
        fi = []
        # self.coverage_area = np.zeros(self.discrete_grid_y * self.discrete_grid_x)
        sample_aera = 0
        # self.client.simPause(False)
        for agent in self.agents:
            # 随机初始位置
            # sample_aera = np.random.randint(0, 7)
            agent.goal_position = self.goal_position
            fi.append(agent.reset(np.random.randint(0, 360), sample_aera))
        for f in fi:
            f.join()
        time.sleep(2)
        # self.client.simPause(True)
        # time.sleep(2/self.world_clock)
        self.cur_state = self.get_all_state()
        self.episode_num += 1
        self.cumulated_episode_reward = np.zeros(self.num_of_drones)
        self.current_step = 0
        self.trajectory_list = []

        # time.sleep(2)
        obs_n = []
        self.pre_reward = []
        for agent in self.agents:
            obs_n.append(self._get_obs(agent))
            self.pre_reward.append(0)
            self.trajectory_list.append([])
        self.pre_obs_n = obs_n
        # for _ in range(8):
        #     action_n = []
        #     for each in self.action_space:
        #         action_n.append(each.sample())
        #     action_n = np.array(action_n)
        #     self.step(action_n)
        return obs_n

    # def render(self):
    #     return self.get_obs()

    def is_not_inside_workspace(self, agent):
        """
        Check if the Drone is inside the Workspace defined
        """
        is_not_inside = False
        current_position = [agent.x, agent.y, agent.z]
        if current_position[0] < self.work_space_x[0] or current_position[0] > self.work_space_x[1] or \
            current_position[1] < self.work_space_y[0] or current_position[1] > self.work_space_y[1] or \
                current_position[2] < self.work_space_z[0] or current_position[2] > self.work_space_z[1]:
            is_not_inside = True

        return is_not_inside

    # #被捕
    # def is_captured(self):
    #     try:
    #         my_file = Path(hp.env_communicate_path + "swapFile")
    #         if my_file.is_file():
    #             self.active_bomb = True
    #             file = open(hp.env_communicate_path +'swapReading','w')
    #             file.write("reading")
    #             remove("./swapFile")
    #             print("niSiLe")
    #             file.close()
    #             remove("./swapReading")
    #             return True
    #     finally:
    #         pass
    #     return False

    def get_depth_image(self, agent):
        # rgb_image_request = airsim.ImageRequest('0', airsim.ImageType.Scene, False, False)
        # print(agent.name +':getting image!。。。。。')
        responses = self.client.simGetImages([self.depth_image_request], vehicle_name=agent.name)
        # responses = self.client.simGetImages([depth_image_request], vehicle_name=agent.name)
        # print('done!')

        depth_img = airsim.list_to_2d_float_array(responses[0].image_data_float, responses[0].width, responses[0].height)

        return depth_img

    def bomb(self):
        if self.active_bomb:
            self.active_bomb = False
            pass

    def get_raw_image(self, agent, pos):
        response = self.client.simGetImages([airsim.ImageRequest(pos, airsim.ImageType.Scene, False, False)],
                                       vehicle_name=agent.name)
        if response[0] is None:
            print("received empty data: data interface at " + agent.name)
        else:
            # get numpy array
            img1d = np.fromstring(response[0].image_data_uint8, dtype=np.uint8)
            # reshape array to 4 channel image array H X W X 4
            img_rgb = img1d.reshape(response[0].height, response[0].width, 3)
            return img_rgb