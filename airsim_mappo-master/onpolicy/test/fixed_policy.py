from onpolicy.envs.airsim_envs.airsim_socket import CustomAirsimClient
from socket import *
import pandas as pd
import time
import airsim
import threading
import numpy as np


class RecurrentList(object):
    """循环列表"""
    def __init__(self, data):
        self.data = data
        self.ptr = 0
        self.length = len(self.data)

    def next(self):
        self.ptr += 2
        if self.ptr >= self.length:
            self.ptr = 0
        return self.data[self.ptr], self.data[self.ptr+1]


class FixedPolicy:
    def __init__(self, file_name, ip, port):
        _ = pd.read_table(file_name, sep='\t', header=None)
        self.patrol_list = []
        for each in _[0]:
            self.patrol_list.append(each.split(" "))
        self.socket_server = socket(AF_INET, SOCK_STREAM)
        self.socket_server.bind(('127.0.0.1', port))
        self.socket_server.listen(200)  # 最大连接数
        self.client = CustomAirsimClient(ip, self.socket_server, plot_flag=True)
        self.height = -20
        self.velocity = 10
        self.destroy_distance = 40
        time.sleep(2)
        self.mission_points = {}
        for mission_point, bp_name in zip(self.patrol_list, self.client.vehicle_dict):
            self.mission_points[bp_name] = RecurrentList(np.array(mission_point, dtype=float)/100)
        time.sleep(1)
        self.remained_vehicle = self.client.listVehicles()
        self.reset()

    def reset(self):
        for bp_name in self.mission_points:
            pos = airsim.Pose()
            pos.position.x_val = self.mission_points[bp_name].data[0]
            pos.position.y_val = self.mission_points[bp_name].data[1]
            pos.position.z_val = self.height
            self.client.simSetVehiclePose(pos, ignore_collision=True, vehicle_name=bp_name)
            self.client.enableApiControl(True, vehicle_name=bp_name)

    def fly_patrol(self, bp_name):
        patrol_client = airsim.MultirotorClient(self.client.vehicle_dict[bp_name].client.ip)
        airsim_name = self.client.vehicle_dict[bp_name].airsim_name
        while True:
            x, y = self.mission_points[bp_name].next()
            _ = patrol_client.moveToPositionAsync(x,
                                                  y,
                                                  self.height, self.velocity, vehicle_name=airsim_name,
                                                  drivetrain=airsim.DrivetrainType.ForwardOnly,
                                                  yaw_mode=airsim.YawMode(is_rate=False))
            _.join()

    def fly_detect(self, bp_name):
        cnt = 0
        while True:
            _ = self.client.get_info(bp_name)
            print(_)
            if cnt > 100:
                self.client.enableApiControl(False, bp_name)
            time.sleep(1)
            cnt += 1

    def fly_run(self):
        fly_list = []
        for each in self.mission_points:
            fly_list.append(threading.Thread(target=self.fly_patrol, args=[each]))
            fly_list[-1].start()

        # test_list = []
        # for each in self.start:
        #     test_list.append(threading.Thread(target=self.fly_detect, args=[each]))
        #     test_list[-1].start()

    def get_remain_pose(self):
        pos = []
        for each in self.remained_vehicle:
            pos = self.client.simGetObjectPose(each)
        return np.array(pos)

    def destroy_vehicle(self, bp_name):
        tmp_client = airsim.MultirotorClient(self.client.vehicle_dict[bp_name].client.ip)
        airsim_name = self.client.vehicle_dict[bp_name].airsim_name
        pose = tmp_client.simGetObjectPose(airsim_name)
        pose.position.x_val = 9999
        pose.position.y_val = 9999
        tmp_client.simSetVehiclePose(pose, True, vehicle_name=airsim_name)
        self.remained_vehicle.remove(bp_name)


if __name__ == "__main__":
    a = FixedPolicy("patrol_20.txt", 9699)
    time.sleep(1)
    a.fly_run()
