import numpy as np
from pathlib import Path
import torch
import configparser
from onpolicy.config import get_config
from onpolicy.envs.mpe.MPE_env import MPEEnv
from onpolicy.envs.env_wrappers import SubprocVecEnv, DummyVecEnv
from onpolicy.envs.airsim_envs.airsim_env import AirSimDroneEnv
from onpolicy.algorithms.r_mappo.algorithm.r_actor_critic import R_Actor, R_Critic
import pandas as pd


class Myconf(configparser.ConfigParser):
    def __init__(self, defaults=None):
        configparser.ConfigParser.__init__(self, defaults=None)

    def optionxform(self, optionstr: str) -> str:
        return optionstr


def _t2n(x):
    return x.detach().cpu().numpy()



if __name__ == '__main__':
    default_cfg = 'D:/crazyflie-simulation/airsim_mappo/onpolicy/envs/airsim_envs/cfg/default.cfg'
    cfg = Myconf()
    cfg.read(default_cfg)
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
    envs = AirSimDroneEnv(cfg)
    num_agents = cfg.getint('options', 'num_of_drone')

    config = {
        "cfg": cfg,
        "envs": envs,
        "num_agents": num_agents,
        "device": device
    }

    # load model
    policy_actor_state_dict = torch.load(str(cfg.get("algorithm", 'model_dir')) + '/actor.pt')
    actor1 = R_Actor(config['cfg'], config['envs'].observation_space[0], config['envs'].action_space[0], config['device'])
    actor1.load_state_dict(policy_actor_state_dict)

    rnn_states_actor_n = []
    rnn_states_actor = np.zeros(
        (1, 1, 64),
        dtype=np.float32)
    masks = np.ones((1, 1), dtype=np.float32)
    obs_n = []
    for i in range(num_agents):
        obs_n.append(envs._get_obs(envs.agents[i]))
        rnn_states_actor_n.append(rnn_states_actor)

    action_n = np.zeros(num_agents)

    ############start########################################################################
    # first visited point
    file_name = 'mission_point.txt'
    _ = pd.read_table(file_name, sep='\t', header=None)
    mission_list = []
    for each in _[0]:
        mission_list.append(each.split(" "))
    mission_list = np.array(mission_list, dtype=float) / 100
    start_point = [-1356, -48, -2]
    height = -10

    # enemy patrol point

    _ = pd.read_table('patrol_20.txt', sep='\t', header=None)
    enemy_list = []
    for each in _[0]:
        enemy_list.append(each.split(" "))
    enemy_list = np.array(mission_list, dtype=float) / 100

    envs.reset()

    d1arrive = False
    arrivel = np.zeros(20)
    personpos = envs.agents[0].client.simGetObjectPose(envs.goal_name[0])
    while True:
        if d1arrive:
            goal_t = []
            for pos in dlist:
                for j in range(10):
                    if arrivel[2*j] == 1 and arrivel[2*j + 1] == 1:
                        envs.agents[2*j].goal_position = [float(pos[0])/ 100, float(pos[1])/ 100, -height]
                        envs.agents[2*j + 1].goal_position = [float(pos[0])/ 100, float(pos[1])/ 100, -height]
                        arrivel[2*j] = 0
                        envs.agents[2*j].battle_flag = 0
                        arrivel[2*j + 1] = 0
                        envs.agents[2*j + 1].battle_flag = 0
                        goal_t.append(pos)

            for pos in goal_t:
                dlist.remove(pos)

            for i in range(num_agents):
                if envs.agents[i].is_in_desired_pose():
                    arrivel[i] = 1
                    envs.agents[i].battle_flag = 1

        else:
            for i in range(num_agents):
                name = envs.agents[i].airsim_name
                x = float(tarP.loc[name]['p1'][0]) / 100
                y = float(tarP.loc[name]['p1'][1]) / 100

                envs.agents[i].goal_position = [x, y, -height]

            if envs.agents[0].is_in_desired_pose():
                d1arrive = True
                envs.agents[0].battle_flag = 1
                arrivel[0] = 1
                goal_t = []

                for j in range(5):
                    pos = dlist[j]
                    envs.agents[2*j].goal_position = [float(pos[0]) / 100, float(pos[1]) / 100, -height]
                    envs.agents[2*j + 1].goal_position = [float(pos[0]) / 100, float(pos[1]) / 100, -height]
                    goal_t.append(pos)

                for k in range(10, 20):
                    envs.agents[k].goal_position = [personpos.position.x_val, personpos.position.y_val,
                                                    personpos.position.z_val]

                for pos in goal_t:
                    dlist.remove(pos)

        for i in range(num_agents):
            obs_tmp = obs[i].copy()
            obs_tmp = np.reshape(obs_tmp, (1, 29))
            tmp, _, rnn_states_actor_n[i] = actor1(obs_tmp, rnn_states_actor_n[i], masks)
            action_n[i] = int(tmp[0][0])
            rnn_states_actor_n[i] = np.array(np.split(_t2n(rnn_states_actor_n[i]), 1))
            rnn_states_actor_n[i] = np.reshape(rnn_states_actor_n[i], (1, 1, 64))

        action_n = np.reshape(action_n, (num_agents, 1))

        obs, rewards, dones, infos = envs.step(action_n)


