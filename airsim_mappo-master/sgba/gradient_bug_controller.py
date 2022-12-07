#!/usr/bin/env python

"""
Com_bug_controller
"""
import rospy
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
from neat_ros.srv import StartSim
import matplotlib.pyplot as plt


from geometry_msgs.msg import Twist
from scipy.stats._continuous_distns import beta
import wall_following
import receive_rostopics

from copy import deepcopy




class GradientBugController:

    WF=wall_following.WallFollowing()
    RRT = receive_rostopics.RecieveROSTopic()

    distance_to_wall = 0;
    first_rotate = True
    direction = 1
    last_bearing = 0
    hitpoint = PoseStamped()
    stateStartTime=0
    state = "ROTATE_TO_GOAL"



    def __init__(self):

        #Get Desired distance from the wall
        self.distance_to_wall=self.WF.getWantedDistanceToWall();
        self.direction = self.WF.getDirectionTurn();
        self.hitpoint = PoseStamped();
        self.first_rotate = True;
        self.last_bearing = 0;
        self.stateStartTime = 0;
        self.state =  "ROTATE_TO_GOAL"
        self.pose_tower = PoseStamped();
        self.first_run = 1
        self.current_UWB_bearing = 2000
        self.last_range = 0.0
        self.current_range = 0.0
        self.last_range_diff = 0.0

        self.last_heading_rate = 2.5;
        self.first_gradient = 1
        self.diff_diff_range = 0.0
        self.diff_range = 0.0

        self.mem_range = 2000;
        self.mem_heading = 1000
        self.heading_before_turning = 0;

        self.rotated_half_once = False
        self.mem_hit_point_range = 2000



    def stateMachine(self,RRT,odometry):


        self.RRT = RRT

        self.current_range = self.RRT.getUWBRange()

        if self.first_gradient:
            self.last_range = self.current_range
            self.first_gradient =0
        self.diff_range = (self.current_range-self.last_range)/100.0
        self.diff_diff_range = self.last_range_diff-self.diff_range;



        range_front = 1000.0
        range_side = 1000.0
        if self.direction is 1:
            range_front=self.RRT.getRangeFrontLeft()
            range_side=self.RRT.getRangeLeft()
        elif self.direction is -1:
            range_front=self.RRT.getRangeFrontRight()
            range_side=self.RRT.getRangeRight()

        bot_pose = PoseStamped();
        bot_pose.pose.position.x = odometry.pose.position.x;
        bot_pose.pose.position.y = odometry.pose.position.y;

        if self.first_run:
            self.bot_init_position = self.RRT.getPoseBot();
            pose_tower_abs = self.RRT.getPoseTower();
            self.pose_tower.pose.position.x = pose_tower_abs.pose.position.x - self.bot_init_position.pose.position.x;
            self.pose_tower.pose.position.y = pose_tower_abs.pose.position.y - self.bot_init_position.pose.position.y;
            self.stateStartTime = deepcopy(self.RRT.getArgosTime())
            if( abs(pose_tower_abs.pose.position.x)> 0.0):
                self.first_run = 0


        else:
            rel_x =  self.pose_tower.pose.position.x-bot_pose.pose.position.x  ;
            rel_y =   self.pose_tower.pose.position.y - bot_pose.pose.position.y ;
            theta = -1*self.RRT.getHeading();

            rel_loc_x = rel_x*numpy.math.cos(theta)-rel_y*numpy.math.sin(theta)
            rel_loc_y = rel_x*numpy.math.sin(theta)+rel_y*numpy.math.cos(theta)

            self.current_UWB_bearing =  numpy.arctan2(rel_loc_y,rel_loc_x)

            #self.current_UWB_bearing = self.RRT.getUWBBearing();

        print self.mem_heading
        # Handle State transition
        if self.state == "FORWARD":
            if self.RRT.getRealDistanceToWall()<self.distance_to_wall+0.1: #If an obstacle comes within the distance of the wall
                self.mem_heading = self.RRT.getHeading()


                if self.RRT.getAngleToWall()>0:
                    self.direction = -1
                   # self.last_heading_rate = 2.5;

                else:
                    self.direction = 1
                  #  self.last_heading_rate = -2.5;


                if self.mem_hit_point_range < self.RRT.getUWBRange():
                    self.direction =  -1*self.direction
                   # self.last_heading_rate = -1*self.last_heading_rate;

                self.mem_hit_point_range = deepcopy(self.RRT.getUWBRange() +100)


                self.transition("WALL_FOLLOWING")

        elif self.state == "WALL_FOLLOWING":
            #bot_pose = self.RRT.getPoseBot();
            #If wall is lost by corner, rotate to goal again
            if range_front>=2.0 and self.diff_range < 0 and\
            ((self.logicIsCloseTo(self.hitpoint.pose.position.x, bot_pose.pose.position.x,0.05)!=True ) or \
            (self.logicIsCloseTo(self.hitpoint.pose.position.y, bot_pose.pose.position.y,0.05)!=True)):
                self.transition("ROTATE_TO_GOAL")
                self.last_bearing = deepcopy(self.current_UWB_bearing)
                self.WF.init()
            if(self.RRT.getUWBRange()>self.mem_range and self.rotated_half_once == False):
                self.transition("ROTATE_180")
                self.WF.init()
                self.direction = -1*self.direction
                self.heading_before_turning = self.RRT.getHeading()
        elif self.state=="ROTATE_TO_GOAL":
            if self.logicIsCloseTo(self.diff_range,-0.035,0.0008)  :
                self.first_rotate = False
                self.mem_range =  deepcopy(self.RRT.getUWBRange())+200
                self.transition("FORWARD")
            if self.RRT.getRealDistanceToWall()<self.distance_to_wall+0.1:
                self.transition("WALL_FOLLOWING")
                self.first_rotate = False
        elif self.state=="ROTATE_180":
            if math.fabs(self.wrap_pi(self.RRT.getHeading()-self.heading_before_turning))>3.04:
                self.rotated_half_once = True
                self.transition("TURN_COMP")
        elif self.state=="TURN_COMP":
            if (self.RRT.getArgosTime() - self.stateStartTime)<2:
                self.transition("WALL_FOLLOWING")




        # Handle actions
        if self.state == "FORWARD":
            twist=self.WF.twistForward() #Go forward with maximum speed
            #twist = self.headingGradientDescentRange()

        elif self.state == "WALL_FOLLOWING":
            # Wall following controller of wall_following.py
            twist = self.WF.wallFollowingController(range_side,range_front,
                                                    self.RRT.getLowestValue(),self.RRT.getHeading(),self.RRT.getArgosTime(),self.direction)
        elif self.state=="ROTATE_TO_GOAL":
            #First go forward for 2 seconds (to get past any corner, and then turn
            #twist = self.headingGradientDescentRange()


            if (self.RRT.getArgosTime() - self.stateStartTime)<self.WF.getDistanceAroundCorner90()/0.35 * 10:
                twist=self.headingZigZag()
            else:
                twist = self.headingGradientDescentRange()

        elif self.state=="ROTATE_180":
            twist = self.WF.twistTurnInCorner(-self.direction)
        elif self.state=="TURN_COMP":
            twist = self.WF.twistTurnInCorner(-self.direction)


        print self.state

       #self.cmdVelPub.publish(twist)
        self.lastTwist = twist


        self.last_range_diff = deepcopy(self.diff_range)
        self.last_range = deepcopy(self.current_range)

        return twist


    # Transition state and restart the timer
    def transition(self, newState):
        self.state = newState
        self.stateStartTime = self.RRT.getArgosTime()

    # See if a value is within a margin from the wanted value
    def logicIsCloseTo(self, real_value = 0.0, checked_value =0.0, margin=0.05):

        if real_value> checked_value-margin and real_value< checked_value+margin:
            return True
        else:
            return False



    def headingGradientDescentRange(self):
        v = 1

        command = self.last_heading_rate
        print(self.current_range,self.diff_range,self.diff_diff_range)

        if(self.diff_diff_range>0):
            command = -1*self.last_heading_rate

        print('command',command)

#         if abs(self.diff_diff_range)< 0.0005:
#             print "zero command"
#             command = self.diff_diff_range

        #command = 0
        #w = (command + self.last_heading_rate)/2
#         command = 1000*self.diff_diff_range
#
#         if abs(command)>2.5:
#             command = 2.5 * command/abs(command)
#
        w = command;


        self.last_heading_rate = deepcopy(w)

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w

        return twist

    def headingZigZag(self):
        v = 1
        command = -1*self.last_heading_rate
        w = command;
        self.last_heading_rate = deepcopy(w)

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w

        return twist


    def wrap_pi(self, angles_rad):
        return numpy.mod(angles_rad+numpy.pi, 2.0 * numpy.pi) - numpy.pi
