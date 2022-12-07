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


from geometry_msgs.msg import Twist
from scipy.stats._continuous_distns import beta
import wall_following 
import receive_rostopics

class BlindBugController:
    state = "ROTATE_TO_GOAL"
    stateStartTime=0

    WF=wall_following.WallFollowing()
    RRT = receive_rostopics.RecieveROSTopic()
    distance_to_wall = 0;
    first_rotate = True

    hitpoint = PoseStamped()
    heading_before_turning = 0
    hit_points = []
    direction = 1
    init_direction = 1
    
    last_diff_heading = 0
    
    time_side= -1;
    heading_target = 0;


    rotated_half_once = False

    def __init__(self):

        self.distance_to_wall=self.WF.getWantedDistanceToWall();
        self.direction = self.WF.getDirectionTurn();
        self.init_direction = self.WF.getDirectionTurn();
        self.stateStartTime = 0;
        self.first_rotate = True;
        self.heading_before_turning = 0;
        self.time_side = -1
        self.heading_target = 0;
        self.last_diff_heading = 0;
        self.state =  "ROTATE_TO_GOAL"
    
    def stateMachine(self, RRT):   
        
        self.RRT = RRT
        range_front = 1000.0
        range_side = 1000.0

        
        if self.direction is 1:
            range_front=self.RRT.getRangeFrontLeft()
            range_side=self.RRT.getRangeLeft()
        elif self.direction is -1:
            range_front=self.RRT.getRangeFrontRight()
            range_side=self.RRT.getRangeRight()
            
            
        diff_heading = self.wrap_pi(self.heading_target-self.RRT.getHeading())
        
        print self.time_side
        print diff_heading
        print self.direction 
        # Handle State transition
        if self.state == "FORWARD": 
            if self.RRT.getRealDistanceToWall()<self.distance_to_wall+0.1: #If an obstacle comes within the distance of the wall
                if self.time_side>0:
                    self.direction = -1
                else:
                    self.direction = 1
                self.transition("WALL_FOLLOWING")
        elif self.state == "WALL_FOLLOWING":
            if self.direction is 1:
                self.time_side = self.time_side + 1*math.sin(diff_heading)
            elif self.direction is -1:
                self.time_side =  self.time_side + 1*math.sin(diff_heading)
            #If wall is lost by corner, rotate to goal again
            if range_front>=2.0:
                self.transition("ROTATE_TO_GOAL")
                self.last_diff_heading = diff_heading
                self.WF.init()
        elif self.state=="ROTATE_TO_GOAL":
            if self.logicIsCloseTo(self.heading_target,self.RRT.getHeading(),0.05) and self.first_rotate== False:
                self.transition("FORWARD")
            if self.logicIsCloseTo(0,self.RRT.getUWBBearing(),0.05) and self.first_rotate:
                self.heading_target = self.RRT.getHeading();
                self.first_rotate = False
                self.direction = self.init_direction
                self.transition("FORWARD")
            else:
                if self.RRT.getRealDistanceToWall()<self.distance_to_wall+0.1:
                    self.transition("WALL_FOLLOWING")
                    self.first_rotate = False
        elif self.state=="ROTATE_180":
            if math.fabs(self.wrap_pi(self.RRT.getHeading()-self.heading_before_turning))>3.04:
                self.rotated_half_once = True
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
  
            if self.first_rotate or\
              (self.last_diff_heading<0 and self.direction == 1) or\
              (self.last_diff_heading>0 and self.direction == -1):                
                twist = self.WF.twistTurnInCorner(self.direction)
            else:
                if (self.RRT.getArgosTime() - self.stateStartTime)<self.WF.getDistanceAroundCorner90()/0.35 * 10:
                    twist=self.WF.twistForward()
                else:
                    twist = self.WF.twistTurnAroundCorner(self.distance_to_wall+0.4,self.direction)
        elif self.state=="ROTATE_180":
            twist = self.WF.twistTurnInCorner(self.direction)

    
        print self.state
                
            
        self.lastTwist = twist
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
        
    def twistRotateToGoal(self):
        v = 0
        w = self.MAX_ROTATION_SPEED * numpy.sign(self.RRT.getUWBBearing())
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist

    
    def wrap_pi(self, angles_rad):
        return numpy.mod(angles_rad+numpy.pi, 2.0 * numpy.pi) - numpy.pi  
        
        


