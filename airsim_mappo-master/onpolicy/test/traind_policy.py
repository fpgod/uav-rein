from onpolicy.envs.airsim_envs.airsim_socket import CustomAirsimClient
from socket import *
import pandas as pd
import time
import airsim
import threading
import numpy as np
from fixed_policy import *
import torch
import configparser
from onpolicy.envs.airsim_envs.airsim_env import AirSimDroneEnv
from onpolicy.algorithms.r_mappo.algorithm.r_actor_critic import R_Actor, R_Critic


class Myconf(configparser.ConfigParser):
    def __init__(self, defaults=None):
        configparser.ConfigParser.__init__(self, defaults=None)

    def optionxform(self, optionstr: str) -> str:
        return optionstr


def _t2n(x):
    return x.detach().cpu().numpy()


class TrainedPolicy:
    def __init__(self, file_name,  actor, airsim_env:AirSimDroneEnv):
        _ = pd.read_table(file_name, sep='\t', header=None)
        mission_list = []
        for points in _[0]:
            mission_list.append(points.split(" "))
        self.plot_client = airsim.MultirotorClient('127.0.0.1')
        self.actor = actor
        self.env = airsim_env
        self.height = -20
        self.velocity = 2
        self.destroy_distance = 40
        self.remained_vehicle = self.env.client.assigned_blueprint.copy()
        self.attack_flag = np.zeros(len(self.remained_vehicle), dtype=object)
        self.enemy_position = {}
        self.mission_points = {}
        time.sleep(2)
        for mission_point, bp_name in zip(mission_list, self.env.client.vehicle_dict):
            self.mission_points[bp_name] = RecurrentList(np.array(mission_point, dtype=float) / 100)
        self.position_list = []
        self.obs_n = self.reset()
        self.done_n = None
        self.destroyed_enemy = []
        for agent in self.env.agents:
            agent.goal_position = self.mission_points[agent.name].data

        time.sleep(1)

    def reset(self):
        s = self.env.reset()
        for bp_name in self.remained_vehicle:
            self.position_list.append(self.env.client.simGetObjectPose(bp_name).position)
        return s

    def fly_run(self):
        _ = threading.Thread(target=self.fly_to_target, args=[])
        _.start()

    def fly_to_target(self):
            rnn_states_actor_n = []
            rnn_states_actor = np.zeros(
                (1, 1, 64),
                dtype=np.float32)
            masks = np.ones((1, 1), dtype=np.float32)
            for i in range(len(self.remained_vehicle)):
                rnn_states_actor_n.append(rnn_states_actor)

            action_n = np.zeros(len(self.remained_vehicle))
            while True:
                position_list = []
                for i in range(len(self.remained_vehicle)):
                    obs_tmp = self.obs_n[i].copy()
                    obs_tmp = np.reshape(obs_tmp, (1, 29))
                    tmp, _, rnn_states_actor_n[i] = actor1(obs_tmp, rnn_states_actor_n[i], masks)
                    action_n[i] = int(tmp[0][0])
                    rnn_states_actor_n[i] = np.array(np.split(_t2n(rnn_states_actor_n[i]), 1))
                    rnn_states_actor_n[i] = np.reshape(rnn_states_actor_n[i], (1, 1, 64))
                    position_list.append(self.env.client.simGetObjectPose(self.remained_vehicle[i]).position)
                self.position_list = position_list
                action_n = np.reshape(action_n, (num_agents, 1))
                self.obs_n, _, self.done_n, _ = env.step(action_n)

    def find_first_enemy(self, agent_id_list, enemy_class: FixedPolicy):
        find = 0
        goal = []

        for bp_name in enemy_class.remained_vehicle:
            self.enemy_position[bp_name] = enemy_class.client.simGetObjectPose(bp_name).position
            goal.append(self.enemy_position[bp_name])
        for i in range(2):
            self.env.agents[i].goal_position = [goal[i].x_val, goal[i].y_val, 0]

        while not find:
            time.sleep(1)
            for i in agent_id_list:
                enemy_name = enemy_class.remained_vehicle[0]
                dis = goal[i].distance_to(self.position_list[i])
                if dis < self.destroy_distance:
                    self.attack_flag[i] = enemy_name
                    self.env.agents[i].goal_name = enemy_name
                    self.destroyed_enemy.append(enemy_name)
                    find = 1
                    break

        for i in range(len(self.remained_vehicle)):
            if not self.attack_flag[i]:
                bp_name = enemy_class.remained_vehicle[np.random.randint(0, len(enemy_class.remained_vehicle))]
                pose = self.enemy_position[bp_name]
                self.env.agents[i].goal_position[0] = pose.x_val
                self.env.agents[i].goal_position[1] = pose.y_val
                self.env.agents[i].goal_name = bp_name

    def assign_goal(self,  enemy_class: FixedPolicy):
        # 分配没有任务的无人机随机指派点
        for i in range(len(self.attack_flag)):
            if not self.attack_flag[i] and self.done_n[i]:
                bp_name = enemy_class.remained_vehicle[np.random.randint(0, len(enemy_class.remained_vehicle))]
                pose = self.enemy_position[bp_name]
                self.env.agents[i].goal_position[0] = pose.x_val
                self.env.agents[i].goal_position[1] = pose.y_val
                self.env.agents[i].goal_name = bp_name

    def assign_attack(self, drone_num=2):
        for _, enemy in enumerate(self.attack_flag):
            if enemy:
                cur_drone = drone_num - len(np.where(self.attack_flag == enemy)[0])
                for i in range(cur_drone):
                    self.find_nearest_unassigned_agent(_)

    def find_nearest_unassigned_agent(self, i):
        # 找到最近的友方无人机进行协同打击
        pose_of_i = self.position_list[i]
        min_distance = 100000
        index = i
        for j, airsim_pose in enumerate(self.position_list):
            tmp_dis = pose_of_i.distance_to(airsim_pose)
            if not self.attack_flag[j] and tmp_dis < min_distance:
                index = j
                min_distance = tmp_dis
        self.attack_flag[index] = self.attack_flag[i]
        self.env.agents[index].goal_name = self.env.agents[i].goal_name

    def attack_enemy(self, enemy_class: FixedPolicy):
        # 持续追踪打击
        for self_id, enemy in enumerate(self.attack_flag):
            if enemy:
                pose = self.enemy_position[enemy]
                self.env.agents[self_id].goal_position[0] = pose.x_val
                self.env.agents[self_id].goal_position[1] = pose.y_val
                self.plot_attack(np.array([self_id]), self.env.agents[self_id].goal_name, [0.0, 0.0, 1.0, 1.0])

    def get_nearest_enemy(self, i):
        pose1 = self.position_list[i]
        min_dis = 100000
        target_name = None
        for enemy_bp_name in self.enemy_position:
            tmp_dis = pose1.distance_to(self.enemy_position[enemy_bp_name])
            if tmp_dis < min_dis:
                target_name = enemy_bp_name
                min_dis = tmp_dis
        return target_name, min_dis

    def detect_destroy_distance(self,  enemy_class: FixedPolicy):
        for i in range(len(self.remained_vehicle)):
            if not self.attack_flag[i] and not self.env.agents[i].wait_step:
                if self.remained_vehicle[i] in self.destroyed_enemy:
                    continue
                enemy_name, dis = self.get_nearest_enemy(i)
                if dis < self.destroy_distance:
                    self.attack_flag[i] = enemy_name
                    self.env.agents[i].goal_name = enemy_name
                    self.destroyed_enemy.append(enemy_name)
        # 满足条件后触发导弹攻击
        for _, enemy in enumerate(self.attack_flag):
            if enemy:
                friendly_force = np.where(self.attack_flag == enemy)[0]
                target_enemy_pose = self.enemy_position[enemy]
                cnt = 0
                for agent in friendly_force:
                    if self.position_list[agent].distance_to(target_enemy_pose) < self.destroy_distance:
                        cnt += 1
                if cnt >= 2:
                    self.attack_flag[friendly_force] = 0
                    _ = threading.Thread(target=self.destroy_enemy, args=[friendly_force, enemy_class, enemy])
                    _.start()

    def destroy_enemy(self, friendly_force, enemy_class: FixedPolicy, enemy):
        # enemy_class.client.simSetVehiclePose(self.enemy_position[enemy], True, enemy)
        enemy_class.client.enableApiControl(False, enemy)
        for i in friendly_force:
            self.env.agents[i].wait_step = 10
        while self.env.agents[friendly_force[0]].wait_step > 0:
            time.sleep(0.5)
            self.plot_attack(friendly_force, enemy)
        enemy_class.destroy_vehicle(enemy)
        self.enemy_position.pop(enemy)
    # def update_enemy_pose(self, enemy_class: FixedPolicy):
    #     while True:
    #         for bp_name in enemy_class.remained_vehicle:
    #             self.enemy_position[bp_name] = enemy_class.client.simGetObjectPose(bp_name)

    def attack_run(self, enemy_class: FixedPolicy):
        self.find_first_enemy(np.array([0, 1]), enemy_class)

        while len(enemy_class.remained_vehicle):
            for bp_name in enemy_class.remained_vehicle:
                self.enemy_position[bp_name] = enemy_class.client.simGetObjectPose(bp_name).position
            self.assign_attack()
            self.assign_goal(enemy_class)
            self.attack_enemy(enemy_class)
            self.detect_destroy_distance(enemy_class)
            time.sleep(1)

    def plot_attack(self, friendly_force, enemy_name, color = None):
        if color is None:
            color = [1.0, 1.0, 0.0, 1.0]
        target_pose = self.enemy_position[enemy_name]
        for _ in friendly_force:
            self.plot_client.simPlotLineList([self.position_list[_], target_pose], thickness=150.0, duration=0.2,
                                              color_rgba=color,
                                              is_persistent=False)
            # time.sleep(0.1)


if __name__ == "__main__":
    default_cfg = 'D:/crazyflie-simulation/airsim_mappo/onpolicy/envs/airsim_envs/cfg/default.cfg'
    cfg = Myconf()
    cfg.read(default_cfg)
    for each in cfg.items("algorithm"):
        cfg.__dict__[each[0]] = each[1]
    if cfg.getboolean('algorithm', 'cuda') and torch.cuda.is_available():
        print("choose to use gpu...")
        device = torch.device("cuda:0")
        torch.set_num_threads(cfg.getint('algorithm', 'n_training_threads'))
        if cfg.getboolean('algorithm', 'cuda_deterministic'):
            torch.backends.cudnn.benchmark = False
            torch.backends.cudnn.deterministic = True
    else:
        print("choose to use cpu...")
        device = torch.device("cpu")
        torch.set_num_threads(cfg.getint('algorithm', 'n_training_threads'))

    # seed
    torch.manual_seed(cfg.getint('algorithm', 'seed'))
    torch.cuda.manual_seed_all(cfg.getint('algorithm', 'seed'))
    np.random.seed(cfg.getint('algorithm', 'seed'))

    # env init
    env = AirSimDroneEnv(cfg)
    num_agents = cfg.getint('options', 'num_of_drone')

    config = {
        "cfg": cfg,
        "envs": env,
        "num_agents": num_agents,
        "device": device
    }

    # load model
    policy_actor_state_dict = torch.load(str(cfg.get("algorithm", 'model_dir')) + '/actor.pt')
    actor1 = R_Actor(config['cfg'], config['envs'].observation_space[0], config['envs'].action_space[0], config['device'])
    actor1.load_state_dict(policy_actor_state_dict)

    patrol_drones = FixedPolicy("patrol_20.txt", ['172.19.0.5'], 9699)
    attack_drones = TrainedPolicy("mission_point.txt", actor1, env)
    patrol_drones.fly_run()
    attack_drones.fly_run()
    attack_drones.attack_run(patrol_drones)

    # a.fly_run()
