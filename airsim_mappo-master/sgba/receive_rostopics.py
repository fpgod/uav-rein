#!/usr/bin/env python

"""
Demo controller for controlling an ARGoS simulated robot via argos_bridge.
The algorithm implemented here is a simple state machine designed to push
pucks to the walls.  If the proximity sensor detects an obstacle (other robot
or wall) then it moves away from it for a fixed period.
"""
import rospy
import roslib
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion
import math, random
import numpy
import time
from argos_bridge.msg import Puck
from argos_bridge.msg import PuckList
from argos_bridge.msg import Proximity
from argos_bridge.msg import ProximityList
from argos_bridge.msg import Rangebearing
from argos_bridge.msg import RangebearingList
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import Twist
from scipy.stats._continuous_distns import beta
from sklearn import linear_model, datasets
import matplotlib.pyplot as plt


class RecieveROSTopic:
    closestObs = None
    lowestValue = 1000.0
    range_left = 1000.0
    range_right = 1000.0
    range_front_left =  1000.0
    range_front_right =  1000.0

    range_middle =  1000.0



    pose_bot = PoseStamped()
    pose_tower = PoseStamped()



    argos_time = 0
    heading=0;
    angle_wall=2000.0
    real_distance_to_wall =1000.0
    border_obstacle_left = 0.0;
    border_obstacle_right = 0.0;
    range_obstacle_right = 0.0;
    range=1000.0
    bearing = 2000.0
    odometry=0;
    rssi_tower = 0;
    closest_RAB = 2000
    closest_RAB_index = 0
    RAB_list = []
    

    
    
    goal_angle_list = [0,0,0,0,0,0,0,0,0]
    made_it_list = [False,False,False,False,False,False,False,False,False]

    
    
    def goal_angle_bot1_callback(self,bot):#, bot3, bot4, bot5, bot6, bot7, bog8, bot9):
        self.goal_angle_list[0]=bot.goal_angle.data

    def goal_angle_bot2_callback(self,bot):#, bot3, bot4, bot5, bot6, bot7, bog8, bot9):
        self.goal_angle_list[1]=bot.goal_angle.data

    def goal_angle_bot3_callback(self,bot):#, bot3, bot4, bot5, bot6, bot7, bog8, bot9):
        self.goal_angle_list[2]=bot.goal_angle.data
        
    def goal_angle_bot4_callback(self,bot):#, bot3, bot4, bot5, bot6, bot7, bog8, bot9):
        self.goal_angle_list[3]=bot.goal_angle.data
        
    def goal_angle_bot5_callback(self,bot):#, bot3, bot4, bot5, bot6, bot7, bog8, bot9):
        self.goal_angle_list[4]=bot.goal_angle.data

    def goal_angle_bot6_callback(self,bot):#, bot3, bot4, bot5, bot6, bot7, bog8, bot9):
        self.goal_angle_list[5]=bot.goal_angle.data
    def goal_angle_bot7_callback(self,bot):#, bot3, bot4, bot5, bot6, bot7, bog8, bot9):
        self.goal_angle_list[6]=bot.goal_angle.data
    def goal_angle_bot8_callback(self,bot):#, bot3, bot4, bot5, bot6, bot7, bog8, bot9):
        self.goal_angle_list[7]=bot.goal_angle.data
    def goal_angle_bot9_callback(self,bot):#, bot3, bot4, bot5, bot6, bot7, bog8, bot9):
        self.goal_angle_list[8]=bot.goal_angle.data
    def made_it_bot1_callback(self,bot):#, bot3, bot4, bot5, bot6, bot7, bog8, bot9):
        self.made_it_list[0]=bot.data

    def made_it_bot2_callback(self,bot):#, bot3, bot4, bot5, bot6, bot7, bog8, bot9):
        self.made_it_list[1]=bot.data

    def made_it_bot3_callback(self,bot):#, bot3, bot4, bot5, bot6, bot7, bog8, bot9):
        self.made_it_list[2]=bot.data
        
    def made_it_bot4_callback(self,bot):#, bot3, bot4, bot5, bot6, bot7, bog8, bot9):
        self.made_it_list[3]=bot.data
        
    def made_it_bot5_callback(self,bot):#, bot3, bot4, bot5, bot6, bot7, bog8, bot9):
        self.made_it_list[4]=bot.data

    def made_it_bot6_callback(self,bot):#, bot3, bot4, bot5, bot6, bot7, bog8, bot9):
        self.made_it_list[5]=bot.data
    def made_it_bot7_callback(self,bot):#, bot3, bot4, bot5, bot6, bot7, bog8, bot9):
        self.made_it_list[6]=bot.data
    def made_it_bot8_callback(self,bot):#, bot3, bot4, bot5, bot6, bot7, bog8, bot9):
        self.made_it_list[7]=bot.data
    def made_it_bot9_callback(self,bot):#, bot3, bot4, bot5, bot6, bot7, bog8, bot9):
        self.made_it_list[8]=bot.data
        
    # Collect current heading and odometry from position sensor
    def pose_callback(self,pose):
        (roll,pitch,yaw) = euler_from_quaternion([pose.pose.orientation.x, \
        pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        self.heading = yaw;
        self.odometry = math.cos(yaw)*pose.pose.position.x - math.sin(yaw)*pose.pose.position.y
        self.pose_bot = pose


    def pose_callback_tower(self,pose):
        (roll,pitch,yaw) = euler_from_quaternion([pose.pose.orientation.x, \
        pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        self.pose_tower = pose


    # Collect range and bearing to goal (or other bots)
    def rab_callback(self,rab_list):
        self.RAB_list=[]
        for it in range(0,rab_list.n):
            self.RAB_list.append(rab_list.Rangebearings[it].range)
            self.range = rab_list.Rangebearings[it].range
            angle = rab_list.Rangebearings[it].angle
            self.bearing = self.wrap_pi(angle)
            
            
    def rssi_tower_callback(self,rssi):
        self.rssi_tower = rssi


    # Several functions for calculating
    def prox_callback(self, proxList):

        # Find the closest obstacle (other robot or wall).  The closest obstacle
        # is the one with the greatest 'value'.
        self.closestObs = None
        self.lowestValue = 1000.0
        self.range_left = 1000.0
        self.range_right = 1000.0
        self.range_front =  1000.0
        self.angle_wall = 2000.0
        self.real_distance_to_wall =1000.0
        self.border_obstacle_left=0;


        self.range_right = self.numRangeMax(proxList.proximities[3].value);
        self.range_left = self.numRangeMax(proxList.proximities[1].value);
        self.range_front_left = self.numRangeMax( proxList.proximities[23].value);
        self.range_front_right = self.numRangeMax( proxList.proximities[4].value);
        self.range_middle = self.numRangeMax( proxList.proximities[13].value);
        self.argos_time = proxList.header.seq


        #Calculate the neares obstacle in the proximity front wedge sesor
        self.calculateLowestValue(proxList)

        #Calculate the angle towards the right side of the border
        self.calculateRightObstacleBorder(proxList)

        #calculate the angle of which the bot approaches a wall, to get the perpendicular distance
        self.calculateWallRANSAC(proxList)


    def getRangeLeft(self):
        return self.range_left
    def getRangeRight(self):
        return self.range_right
    def getRangeFrontLeft(self):
        return self.range_front_left
    def getRangeFrontRight(self):
        return self.range_front_right
    def getRangeMiddle(self):
        return self.range_middle
    def getArgosTime(self):
        return self.argos_time
    def getLowestValue(self):
        return self.lowestValue;
    def getUWBRange(self):
        return self.range;
    def getUWBBearing(self):
        return self.bearing;
    def getRealDistanceToWall(self):
        return self.real_distance_to_wall
    def getAngleToWall(self):
        return self.angle_wall
    def getHeading(self):
        return self.heading
    def getOdometry(self):
        return self.odometry
    def getLeftObstacleBorder(self):
        return self.border_obstacle_left
    def getRightObstacleBorder(self):
        return self.border_obstacle_right
    def getRightObstacleBorderRange(self):
        return self.range_obstacle_right
    def getPoseBot(self):
        return self.pose_bot
    def getPoseTower(self):
        return self.pose_tower
    def getRSSITower(self):
        return self.rssi_tower.data
    def getClosestRAB(self):
        self.calculateLowestRAB()
        return self.closest_RAB, self.closest_RAB_index
    def getGoalAngleID(self, id):
        return self.goal_angle_list[id-1]
    def getMadeItID(self,id):
        return self.made_it_list[id-1]


    def calculateLowestRAB(self):
        self.closest_RAB = 2000
        for it in range(0,len(self.RAB_list)):
            if self.RAB_list[it] < self.closest_RAB:
                    self.closest_RAB = self.RAB_list[it]
                    self.closest_RAB_index = it
                    
    def calculateLowestValue(self,proxList):
        for it in range(4,len(proxList.proximities)):
            if self.numRangeMax(proxList.proximities[it].value) < self.lowestValue:
                    self.closestObs = proxList.proximities[it]
                    self.lowestValue = self.numRangeMax(proxList.proximities[it].value)                
                    
    def calculateRightObstacleBorder(self,proxList):
        deg=numpy.linspace(-0.52,0.52,num=20)
        for it in list(reversed(range(4,len(proxList.proximities)))):
            if self.numRangeMax(proxList.proximities[it].value)<2.0:
                self.border_obstacle_right = deg[it-4]
                self.range_obstacle_right = self.numRangeMax(proxList.proximities[it].value);

    def calculateWallRANSAC(self, proxList):
        if self.closestObs is not None:
            X=numpy.empty((0,0))[numpy.newaxis];
            Y=numpy.empty((0,0))[numpy.newaxis];
            deg_new=numpy.empty((0,0))[numpy.newaxis];

            deg=numpy.linspace(-0.52,0.52,num=20)

            # if the ranges are within range, put them in a list for ransac
            for it in range(4,len(proxList.proximities)):
                if self.numRangeMax(proxList.proximities[it].value) <2.0:
                   # print("Y",self.numRangeMax(proxList.proximities[it].value))
                   # print("X",math.sin(deg[it-4])*proxList.proximities[it].value)
                    Y= numpy.append(Y,self.numRangeMax(proxList.proximities[it].value))
                    X= numpy.append(X,math.sin(deg[it-4])*proxList.proximities[it].value)
                    deg_new=numpy.append(deg_new,deg[it-4]);

            #If there are more than 5 samples, continue
            if len(X)>5:

                X=numpy.reshape(X,(-1, 1))
                Y=numpy.reshape(Y,(-1, 1))
                deg_new=numpy.reshape(deg_new,(-1,1))

                Xlocal = X;
                Ylocal = Y;

                coefs = numpy.empty((0,0))
                intercepts = numpy.empty((0,0))
                scores = numpy.empty((0,0))

                coef1 = 0
                intercept1 = 0
                for it  in range(0,2):

                    ransac = linear_model.RANSACRegressor()
                    ransac.fit(Xlocal, Ylocal )
                    inlier_mask = ransac.inlier_mask_
                    outlier_mask = numpy.logical_not(inlier_mask)
                    line_y_ransac = ransac.predict(Xlocal)
                    coefs=numpy.append(coefs,ransac.estimator_.coef_[0][0])
                    intercepts=numpy.append(intercepts,ransac.estimator_.intercept_[0])
                    scores = numpy.append(scores,ransac.score(Xlocal,Ylocal))
                    #plt.plot(Xlocal,Ylocal)
                    #plt.hold(True)
                    #plt.plot(Xlocal,line_y_ransac);
                    axes = plt.gca()
                    axes.set_xlim([-1,1])
                    axes.set_ylim([0,2])


                    Ylocal=Ylocal[outlier_mask]
                    Xlocal=Xlocal[outlier_mask]
                    Xlocal=numpy.reshape(Xlocal,(-1, 1))
                    Ylocal=numpy.reshape(Ylocal,(-1, 1))
                    if len(Xlocal)<5:
                        break
                #plt.hold(False)
                #plt.pause(0.01)

               # if len(coefs)<2:
                self.angle_wall = math.atan(coefs[0])

                sum_distance = 0
                for it in range(0,len(X)):
                    sum_distance = sum_distance+Y[it][0]*math.cos(-self.angle_wall - deg_new[it])

                self.real_distance_to_wall = sum_distance/len(X)
                #else:
                   # intersect = coefs[0]*(intercepts[1]-intercepts[0])/(coefs[0]-coefs[1])+intercepts[0]
                    #print("intersect is: ", intersect)
                  #  self.real_distance_to_wall =intersect
                   # self.real_distance_to_wall =math.atan(coefs[0])

                   # self.real_distance_to_wall = self.range_middle*math.sin(1.57+self.angle_wall)

#                     if intersect > (intercepts[0]+intercepts[1])/2:
#                         print "outward corner"
#                         if(numpy.sign(coefs[0])==-1):
#                             self.angle_wall = math.atan(coefs[0])
#                         elif (numpy.sign(coefs[1])==-1):
#                             self.angle_wall = math.atan(coefs[1])
#                         else:
#                             self.angle_wall = 2000.0
#                     if intersect < (intercepts[0]+intercepts[1])/2:
#                         print "inward corner"
#                         if(numpy.sign(coefs[0])==1):
#                             self.angle_wall = math.atan(coefs[0])
#                         elif (numpy.sign(coefs[1])==1):
#                             self.angle_wall = math.atan(coefs[1])
#                         else:
#                             self.angle_wall = 2000.0
#
#                 if self.angle_wall is not 2000.0:
#                     self.real_distance_to_wall = self.range_middle*math.sin(1.57+self.angle_wall)
#                 else:
#                     self.real_distance_to_wall==1000.0

                #print("WALL ANGLE IS: ", self.angle_wall)
                #print("real_distance_is: ", self.real_distance_to_wall)


                """
                self.real_distance_to_wall = 1000.0;
                sum_distance = 0;

                for it in range(0,len(X)):
                    sum_distance = sum_distance+Y[it][0]*math.cos(-self.angle_wall - deg_new[it])

                print("found lines: ",len(coefs))
                intersect = 0;
                if len(coefs)==2:
                    intersect = coefs[0]*(intercepts[1]-intercepts[0])/(coefs[0]-coefs[1])+intercepts[0]
                    print("intersect is: ", intersect)
                    self.real_distance_to_wall = intersect
                    self.angle_wall = coefs[1]
                else:
                    if scores[0] is not 1.0:
                        for it in range(0,len(X)):
                            sum_distance = sum_distance+Y[it][0]*math.cos(-self.angle_wall - deg_new[it])

                        self.real_distance_to_wall = sum_distance/len(X)
                        self.angle_wall = coefs[0]
                """




    def wrap_pi(self, angles_rad):
        return numpy.mod(angles_rad+numpy.pi, 2.0 * numpy.pi) - numpy.pi
    def numRangeMax(self, value = 0.0):
        if value == 0.0:
            return 1000.0
        else:
            return value
