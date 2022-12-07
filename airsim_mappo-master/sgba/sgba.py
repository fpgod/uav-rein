import airsim
import numpy as np
import time
import com_bug_controller
import bug_2_controller
import i_bug_controller
import alg_1_controller
import alg_2_controller
import wall_following_controller
import blind_bug_controller
import gradient_bug_controller
import gradient_bug_embedded_controller
import i_bug_2_controller
from scipy.stats._continuous_distns import beta
import wall_following
import re


dronenum = 10

client = airsim.MultirotorClient()

def get_distance(clt, dnum):
    distance_sensors = []
    for droneid in range(dnum):
        name = "cf" + str(101 + droneid)
        tmp = []
        for sensorid in range(36):
            tmp.append(clt.getDistanceSensorData("Distance"+str(sensorid), name).distance)
        distance_sensors.append(tmp)
    return distance_sensors


class SGBA_Drone():
    cmdVelPub = None
    goalAnglePub = None
    bug_type = "com_bug"
    bug_controller = com_bug_controller.ComBugController()
    RRT = receive_rostopics.RecieveROSTopic()
    WF = wall_following.WallFollowing()
    reset_bug = False
    random_environment = False
    odometry = [0, 0]
    twist = Twist()
    noise_level = 0.4
    odometry = PoseStamped()
    odometry_perfect = PoseStamped()
    previous_time = 0
    send_stop = False
    outbound = True
    outbound_angle = 0
    start_time = 0
    opened_file = False
    pos_save = []
    angle_goal = 0

    # select the appropiate controller by the appropiate bug
    def getController(self, argument):
        switcher = {
            "com_bug": com_bug_controller.ComBugController(),
            "bug_2": bug_2_controller.Bug2Controller(),
            "i_bug": i_bug_controller.IBugController(),
            "i_bug_2": i_bug_2_controller.IBug2Controller(),
            "alg_1": alg_1_controller.Alg1Controller(),
            "alg_2": alg_2_controller.Alg2Controller(),
            "wf": wall_following_controller.WallFollowController(),
            "blind_bug": blind_bug_controller.BlindBugController(),
            "gradient_bug": gradient_bug_controller.GradientBugController(),
            "gradient_embedded_bug": gradient_bug_embedded_controller.GradientBugController(),

        }

        return switcher.get(argument, False)

    def __init__(self, client, id, heading = 0):
        self.client = client
        self.name = "cf" + str(100 + id)
        self.goal_position = [0, 0, 0]
        self.goal_distance = 10
        # [0:'rotate to goal',1:'forward',2:'wall Follow',3:'move out of the way']
        self.state = 1
        self.now_heading = np.radians(heading)
        self.goal_heading = np.radians(heading)
        self.vel_heading = np.radians(heading)
        self.height = 6
        self.range_wall = 3.0
        self.range_front = 3.0
        self.dt = 0.5
        self.direction = 0 #0:right,1:left
        self.ref_distance_from_wall = 1
        self.pre_sensor = []

    def enable(self):
        # pose = self.client.simGetObjectPose(self.name)
        # pose.position.x_val = self.pose_offset[sample_area][0]
        # pose.position.y_val = self.pose_offset[sample_area][1]
        # yaw_noise = math.pi * 2 * np.random.random()
        # pose.orientation = airsim.to_quaternion(0, 0, yaw_noise)
        # self.client.simSetVehiclePose(pose, True, vehicle_name=self.name)
        self.client.enableApiControl(True, vehicle_name=self.name)
        self.client.armDisarm(True, vehicle_name=self.name)
        self.client.moveByRollPitchYawZAsync(0, 0, self.now_heading, -self.height, 2, vehicle_name=self.name)
        time.sleep(1)

    def get_sensor(self):
        #0:front,9:right,18:back,27:left
        tmp = []
        for sensorid in range(36):
            tmp.append(self.client.getDistanceSensorData("Distance"+str(sensorid), self.name).distance)
        print(tmp)
        sensor_4 = [tmp[0],tmp[9],tmp[18],tmp[27]]
        print(sensor_4)
        return sensor_4

    def wall_follow(self, sensor):
        # self.now_heading = airsim.utils.to_eularian_angles(self.client.simGetObjectPose(self.name).orientation)[2]

        if sensor[0] < self.range_front:
            print('find wall')
            if self.direction == 0:
                print(self.now_heading)
                print(1)
                self.now_heading = self.now_heading + np.radians(10)
                # if self.now_heading > np.radians(180):
                #     self.now_heading = self.now_heading - np.pi * 2
                # if self.now_heading < np.radians(-180):
                #     self.now_heading = self.now_heading + np.pi * 2
                self.client.moveByRollPitchYawZAsync(0, 0, self.now_heading, -self.height, 0.2,
                                                     vehicle_name=self.name).join()
            else:
                print(self.now_heading)
                print(2)
                self.now_heading = self.now_heading - np.radians(10)
                # if self.now_heading > np.radians(180):
                #     self.now_heading = self.now_heading - np.pi * 2
                # if self.now_heading < np.radians(-180):
                #     self.now_heading = self.now_heading + np.pi * 2
                self.client.moveByRollPitchYawZAsync(0, 0, self.now_heading, -self.height, 0.2,
                                                     vehicle_name=self.name).join()
        else:
            print('follow wall')
            if self.direction == 0:
                if sensor[1] < self.ref_distance_from_wall:
                    self.now_heading = self.now_heading + np.radians(5)
                    self.client.moveByRollPitchYawZAsync(0, 0, self.now_heading, -self.height, 0.2,
                                                         vehicle_name=self.name).join()
                    self.vel_heading = airsim.utils.to_eularian_angles(self.client.simGetObjectPose(self.name).orientation)[2]
                    self.client.moveByVelocityAsync(np.cos(self.vel_heading), np.sin(self.vel_heading), -0.1, self.dt, vehicle_name=self.name).join()
                elif sensor[1] > self.ref_distance_from_wall and sensor[1] < self.range_wall + 4:
                    self.now_heading = self.now_heading - np.radians(5)
                    self.client.moveByRollPitchYawZAsync(0, 0, self.now_heading, -self.height, 0.2,
                                                         vehicle_name=self.name).join()
                    self.vel_heading = \
                    airsim.utils.to_eularian_angles(self.client.simGetObjectPose(self.name).orientation)[2]
                    self.client.moveByVelocityAsync(np.cos(self.vel_heading), np.sin(self.vel_heading), -0.1, self.dt,
                                                    vehicle_name=self.name).join()
                else:
                    self.vel_heading = \
                    airsim.utils.to_eularian_angles(self.client.simGetObjectPose(self.name).orientation)[2]
                    self.client.moveByVelocityAsync(np.cos(self.vel_heading), np.sin(self.vel_heading), -0.1, self.dt,
                                                    vehicle_name=self.name).join()
                    self.now_heading = self.goal_heading
                    self.client.moveByRollPitchYawZAsync(0, 0, self.now_heading, -self.height, 0.2,
                                                         vehicle_name=self.name).join()
                    self.state = 1
            else:
                if sensor[3] < self.ref_distance_from_wall:
                    self.now_heading = self.now_heading - np.radians(5)
                    self.client.moveByRollPitchYawZAsync(0, 0, self.now_heading, -self.height, 0.2,
                                                         vehicle_name=self.name).join()
                    self.vel_heading = \
                    airsim.utils.to_eularian_angles(self.client.simGetObjectPose(self.name).orientation)[2]
                    self.client.moveByVelocityAsync(np.cos(self.vel_heading), np.sin(self.vel_heading), -0.1, self.dt,
                                                    vehicle_name=self.name).join()
                elif sensor[3] > self.ref_distance_from_wall and sensor[1] < self.range_wall + 4:
                    self.now_heading = self.now_heading + np.radians(5)
                    self.client.moveByRollPitchYawZAsync(0, 0, self.now_heading, -self.height, 0.2,
                                                         vehicle_name=self.name).join()
                    airsim.utils.to_eularian_angles(self.client.simGetObjectPose(self.name).orientation)[2]
                    self.client.moveByVelocityAsync(np.cos(self.vel_heading), np.sin(self.vel_heading), -0.1, self.dt,
                                                    vehicle_name=self.name).join()
                else:
                    self.vel_heading = \
                    airsim.utils.to_eularian_angles(self.client.simGetObjectPose(self.name).orientation)[2]
                    self.client.moveByVelocityAsync(np.cos(self.vel_heading), np.sin(self.vel_heading), -0.1, self.dt,
                                                    vehicle_name=self.name).join()
                    self.now_heading = self.goal_heading
                    self.client.moveByRollPitchYawZAsync(0, 0, self.now_heading, -self.height, 0.2,
                                                         vehicle_name=self.name).join()
                    self.state = 1

    def run(self):
        while(True):
            if (self.state == 1):
                print('forward')
                # self.now_heading = airsim.utils.to_eularian_angles(self.client.simGetObjectPose(self.name).orientation)[2]
                sensor = self.get_sensor()

                if(sensor[0] < self.range_front):
                    if(sensor[1] < self.range_wall and sensor[1] < sensor[3]):#right
                        self.direction = 0
                        self.state = 2
                    else:#left
                        self.direction = 1
                        self.state = 2
                else:
                    self.vel_heading = \
                    airsim.utils.to_eularian_angles(self.client.simGetObjectPose(self.name).orientation)[2]
                    self.client.moveByVelocityAsync(np.cos(self.vel_heading), np.sin(self.vel_heading), -0.1, self.dt,
                                                    vehicle_name=self.name).join()
                self.pre_sensor = sensor
            elif (self.state == 2):
                sensor = self.get_sensor()

                self.wall_follow(sensor)
                self.pre_sensor = sensor
            time.sleep(0.5)


cf101 = SGBA_Drone(client, 1, 170)
cf101.enable()
cf101.run()
