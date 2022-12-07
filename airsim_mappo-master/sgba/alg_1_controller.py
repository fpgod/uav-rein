#!/usr/bin/env python

"""
Com_bug_controller
"""
import rospy
import math, random
import numpy
import time
import random

from argos_bridge.msg import Puck
from argos_bridge.msg import PuckList
from argos_bridge.msg import Proximity
from argos_bridge.msg import ProximityList
from argos_bridge.msg import Rangebearing
from argos_bridge.msg import RangebearingList
from geometry_msgs.msg import PoseStamped
from neat_ros.srv import StartSim
from copy import deepcopy



from geometry_msgs.msg import Twist
from scipy.stats._continuous_distns import beta
import wall_following 
import receive_rostopics

class Alg1Controller:
    state = "ROTATE_TO_GOAL"
    stateStartTime=0

    WF=wall_following.WallFollowing()
    RRT = receive_rostopics.RecieveROSTopic()
    distance_to_wall = 0;
    
    bot_init_position = PoseStamped()
    pose_tower = PoseStamped()
    bot_tower_slope = 0;
    
    hitpoint = PoseStamped()
    
    heading_before_turning = 0
    hit_points = []
    
    direction = 1
    init_direction = 1
    first_run = 1

    rotated_half_once = False
    


    def __init__(self):
        #Get Desired distance from the wall
        self.distance_to_wall=self.WF.getWantedDistanceToWall();
        # FIX THIS HACK!
        self.direction = 1;#self.WF.getDirectionTurn();
        self.init_direction = 1;#self.WF.getDirectionTurn();
        self.hitpoint = PoseStamped();
        self.bot_init_position = PoseStamped()
        self.pose_tower = PoseStamped()
        self.bot_tower_slope = 0;
        self.hit_points = []
        self.last_bearing = 0;
        self.stateStartTime = 0;
        self.obstacle_is_hit = 0;
        self.heading_before_turning = 0;
        self.state =  "ROTATE_TO_GOAL"
        self.previous_leave_point =1000.0;
        self.pose_tower = PoseStamped();
        self.first_run = 1
        self.current_UWB_bearing = 2000
        self.current_UWB_range =  1000
        self.rand_FN = 0;

    
    def stateMachine(self,RRT,odometry,falsepositive_ratio,falsenegative_ratio):   
        self.RRT = RRT

        range_front = 1000.0
        range_side = 1000.0
        
        bot_pose = self.RRT.getPoseBot();
        
        if self.direction is 1:
            range_front=self.RRT.getRangeFrontLeft()
            range_side=self.RRT.getRangeLeft()
        elif self.direction is -1:
            range_front=self.RRT.getRangeFrontRight()
            range_side=self.RRT.getRangeRight()    
    
        bot_tower_slope_run = 0;
        
        bot_pose = PoseStamped();
        bot_pose.pose.position.x = odometry.pose.position.x;
        bot_pose.pose.position.y = odometry.pose.position.y;
        
        if self.first_run:
            self.bot_init_position = self.RRT.getPoseBot();
            pose_tower_abs = self.RRT.getPoseTower();
            self.pose_tower.pose.position.x = pose_tower_abs.pose.position.x - self.bot_init_position.pose.position.x;
            self.pose_tower.pose.position.y = pose_tower_abs.pose.position.y - self.bot_init_position.pose.position.y;
            if math.fabs(self.pose_tower.pose.position.x -self.bot_init_position.pose.position.x)>0:
                self.bot_tower_slope = (self.pose_tower.pose.position.y -self.bot_init_position.pose.position.y)/(self.pose_tower.pose.position.x -self.bot_init_position.pose.position.x);
                self.first_run = 0
        else:
            #bot_pose = self.RRT.getPoseBot();
            #self.pose_tower= self.RRT.getPoseTower();

            bot_tower_slope_run = (self.pose_tower.pose.position.y -bot_pose.pose.position.y)/(self.pose_tower.pose.position.x -bot_pose.pose.position.x);
            rel_x =  self.pose_tower.pose.position.x-bot_pose.pose.position.x  ;
            rel_y =   self.pose_tower.pose.position.y - bot_pose.pose.position.y ; 
            theta = -1*self.RRT.getHeading();

            rel_loc_x = rel_x*numpy.math.cos(theta)-rel_y*numpy.math.sin(theta)
            rel_loc_y = rel_x*numpy.math.sin(theta)+rel_y*numpy.math.cos(theta)
            self.current_UWB_bearing =  numpy.arctan2(rel_loc_y,rel_loc_x)
            self.current_UWB_range = math.sqrt(rel_loc_x**2 +rel_loc_y**2)

        # Handle State transition
        if self.state == "FORWARD": 
            if self.RRT.getRealDistanceToWall()<self.distance_to_wall+0.1: #If an obstacle comes within the distance of the wall
                #self.hitpoint = self.RRT.getPoseBot();
                self.hitpoint.pose.position.x = odometry.pose.position.x;
                self.hitpoint.pose.position.y = odometry.pose.position.y;
                self.previous_hit_point = self.current_UWB_range   
                rand_FP = random.random()
                self.rand_FN = random.random()
                if ((self.checkHitPoints(self.hitpoint) and self.rand_FN>falsenegative_ratio) or rand_FP<falsepositive_ratio) :
                    print "already hit point!"
                    self.rotated_half_once = True
                    self.direction = -1*self.direction
                else:
                    print "Did not hit point"
                self.rand_FN = random.random()
                self.transition("WALL_FOLLOWING")
        elif self.state == "WALL_FOLLOWING": 
            bot_pose = PoseStamped();
            bot_pose.pose.position.x = odometry.pose.position.x;
            bot_pose.pose.position.y = odometry.pose.position.y;
            if self.logicIsCloseTo(self.bot_tower_slope, bot_tower_slope_run,0.02) and \
             rel_x>0 and  self.current_UWB_range<self.previous_hit_point and\
            ((self.logicIsCloseTo(self.hitpoint.pose.position.x, bot_pose.pose.position.x,0.5)!=True) or \
            (self.logicIsCloseTo(self.hitpoint.pose.position.y, bot_pose.pose.position.y,0.5)!=True)):
                self.transition("ROTATE_TO_GOAL")
                self.last_bearing = self.current_UWB_bearing
                self.WF.init()
                self.hit_points.append(deepcopy(self.hitpoint))
                print "saved hitpoint"
            print("already rotated", self.rotated_half_once)
            rand_FP = random.random()
            if ((self.checkHitPoints(bot_pose) and self.rand_FN>falsenegative_ratio) or rand_FP<falsepositive_ratio) and self.rotated_half_once == False and \
            ((self.logicIsCloseTo(self.hitpoint.pose.position.x, bot_pose.pose.position.x,self.WF.getLocationPrecision())!=True ) or \
            (self.logicIsCloseTo(self.hitpoint.pose.position.y, bot_pose.pose.position.y,self.WF.getLocationPrecision())!=True)):
                self.transition("ROTATE_180")
                self.WF.init()
                self.direction = -1*self.direction
                self.heading_before_turning = self.RRT.getHeading() 
        elif self.state=="ROTATE_TO_GOAL":
            if self.logicIsCloseTo(0,self.current_UWB_bearing,0.05) :
                self.rotated_half_once = False
                self.direction = self.init_direction
                self.transition("FORWARD")
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
        elif self.state == "WALL_FOLLOWING":
            # Wall following controller of wall_following.py
            twist = self.WF.wallFollowingController(range_side,range_front,
                                                    self.RRT.getLowestValue(),self.RRT.getHeading(),self.RRT.getArgosTime(),self.direction)     


                

        elif self.state=="ROTATE_TO_GOAL":
            #First go forward for 2 seconds (to get past any corner, and then turn
            
            if self.last_bearing>0:
                twist = self.WF.twistTurnInCorner(-1)
            else:
                twist = self.WF.twistTurnInCorner(1)


#             if (self.RRT.getArgosTime() - self.stateStartTime)<20:
#                 twist=self.WF.twistForward()
#             else:
#                 twist = self.WF.twistTurnAroundCorner(self.distance_to_wall+0.2)
        elif self.state=="ROTATE_180":
            twist = self.WF.twistTurnInCorner(-self.direction)
        elif self.state=="TURN_COMP":
            twist = self.WF.twistTurnInCorner(-self.direction)

#     
        print self.state
                
        self.lastTwist = twist
        return  twist
        

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
        
    
    def checkHitPoints(self,bot_pose):
        for i in range(0,len(self.hit_points)):
            if ((self.logicIsCloseTo(self.hit_points[i].pose.position.x, bot_pose.pose.position.x,self.WF.getLocationPrecision()) ) and \
            (self.logicIsCloseTo(self.hit_points[i].pose.position.y, bot_pose.pose.position.y,self.WF.getLocationPrecision()))):
                return True
        return False
            
    def wrap_pi(self, angles_rad):
        return numpy.mod(angles_rad+numpy.pi, 2.0 * numpy.pi) - numpy.pi  

