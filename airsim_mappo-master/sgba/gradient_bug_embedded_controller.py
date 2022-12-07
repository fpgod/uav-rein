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

import sys
sys.path.append('/home/knmcguire/Software/catkin_ws/src/gradient_bug/scripts/bug_algorithms')
import gradient_bug_v1 
import gradient_bug_v2 
import gradient_bug_v3 




class GradientBugController:

    WF = wall_following.WallFollowing()
    RRT = receive_rostopics.RecieveROSTopic()
    GB =  gradient_bug_v1.GradientBugController()
    GB2 =  gradient_bug_v2.GradientBugController()
    GB3 =  gradient_bug_v3.GradientBugController()

    distance_to_wall = 0;
    rssi_goal_angle_adjust = 0
    goal_angle = 0
    
    bot_is_close = False



    def __init__(self):

        #Get Desired distance from the wall
        self.distance_to_wall=self.WF.getWantedDistanceToWall();
        self.pose_tower = PoseStamped();
        self.first_run = 1

        #Init embedded gradient bug
       # self.GB.init(self.distance_to_wall,self.WF.getMaximumForwardSpeed(),self.WF.getMaximumRotationSpeed())
        #self.GB2.init(self.distance_to_wall,self.WF.getMaximumForwardSpeed(),self.WF.getMaximumRotationSpeed())
        self.GB3.init(self.distance_to_wall,self.WF.getMaximumForwardSpeed(),self.WF.getMaximumRotationSpeed())
        self.current_UWB_range = 0
        self.current_UWB_bearing = 0
        self.rssi_goal_angle_adjust = 0
   



    def stateMachine(self,RRT,odometry,outbound = False, outbound_angle = 0, own_id=1):
        
        
        # Get recieve ros topics
        self.RRT = RRT


        # Get bot position from odometry (with drift)
        bot_pose = PoseStamped();
        bot_pose.pose.position.x = odometry.pose.position.x;
        bot_pose.pose.position.y = odometry.pose.position.y;
        
        #If it is the first loop of the run
        if self.first_run:
            self.GB3.init(self.distance_to_wall,self.WF.getMaximumForwardSpeed(),self.WF.getMaximumRotationSpeed(),outbound_angle)

            self.bot_init_position = self.RRT.getPoseBot(); # Get the initial position of the bot 
            pose_tower_abs = self.RRT.getPoseTower();   # Get the position of the tower
            #Get the relative position to the tower (CHECK IF THIS IS STILL POSSIBLE!)
            self.pose_tower.pose.position.x = pose_tower_abs.pose.position.x - self.bot_init_position.pose.position.x;
            self.pose_tower.pose.position.y = pose_tower_abs.pose.position.y - self.bot_init_position.pose.position.y;
            

            
            # THIS MUST BE GONE IF WE ARE DOING HOMING AKA STARTING FROM TOWER!!!
            #if( abs(pose_tower_abs.pose.position.x)> 0.0):
            self.first_run = False


        else:


            # Get the position to the tower with the bot's odometry
            rel_x =  self.pose_tower.pose.position.x -bot_pose.pose.position.x  
            rel_y =   self.pose_tower.pose.position.y- bot_pose.pose.position.y 
            
            # Rotate the relative position tot tower to the bot's heading
            # SEE IF THIS IS STILL NECESSARY!
            theta = self.wrap_pi(-self.RRT.getHeading())
            rel_loc_x = rel_x*numpy.math.cos(theta)-rel_y*numpy.math.sin(theta)
            rel_loc_y = rel_x*numpy.math.sin(theta)-rel_y*numpy.math.cos(theta)
            
            # Make adjusted odometry (FOR DEBUGGIN ONLY!)
            save_pos_rel_x = 12+ bot_pose.pose.position.x - self.pose_tower.pose.position.x
            save_pos_rel_y = 12+ bot_pose.pose.position.y - self.pose_tower.pose.position.y ;
            adjusted_theta = self.wrap_pi(self.rssi_goal_angle_adjust)
            rel_x_adjust = save_pos_rel_x*numpy.math.cos(adjusted_theta)-save_pos_rel_y*numpy.math.sin(adjusted_theta)
            rel_y_adjust = save_pos_rel_x*numpy.math.sin(adjusted_theta)+save_pos_rel_y*numpy.math.cos(adjusted_theta)
            #numpy.savetxt('rel_loc_x.txt',[rel_x_adjust],delimiter=',')
            #numpy.savetxt('rel_loc_y.txt',[rel_y_adjust],delimiter=',')

            # Get angle to goal and angle to goal
            self.current_UWB_angle =  self.wrap_pi(numpy.arctan2(rel_y,rel_x))
            self.current_UWB_range =12- math.sqrt(math.pow(rel_y,2)+math.pow(rel_x,2))
            
            
        

        
        
        # the bearing value sometimes comes in the form or an array, just is temp fix!!
        if isinstance(self.current_UWB_bearing,numpy.ndarray):
            self.current_UWB_bearing=float(self.current_UWB_bearing[0])
            
        #Retrieve the rssi_noise            
        rssi_noise_list =  numpy.random.normal(self.RRT.getRSSITower(),1,1);
        rssi_noise =  round(float(rssi_noise_list[0]))
        #rssi_noise= round(self.RRT.getRSSITower())
        if rssi_noise>-43:
            rssi_noise = -43 
                    
        #if outbound == True:
         #   self.current_UWB_bearing =self.wrap_pi(outbound_angle-self.RRT.getHeading() )
        #FOR NOW JUST SUBSTITUTE RANGE FOR TRUE RANGE!!
        self.current_range = self.RRT.getUWBRange()
        
        closest_distance_other_bot, other_id= self.RRT.getClosestRAB()
        other_made_it = self.RRT.getMadeItID(other_id)
        priority= True
        if(other_id+1<own_id and other_made_it is not True):
            priority = False
        else:
            priority = True
            
       # print("own id + other_id+ priority",own_id,other_id,priority)
            
        goal_angle_other = self.RRT.getGoalAngleID(other_id)
       # print("priority bot", closest_distance_other_bot, own_id, other_id, priority )
        
        #GRADIENTBUG: get both distance and id of closest bot
        '''if (self.RRT.getClosestRAB()<100):
            self.bot_is_close = True
            print("bot is close!!!")
        else:
            self.bot_is_close = False'''
        
        twist, self.rssi_goal_angle_adjust, self.goal_angle, gb_state = self.GB3.stateMachine(self.RRT.getRealDistanceToWall(),self.RRT.getRangeRight(),self.RRT.getRangeLeft(),
                        self.RRT.getHeading(), self.current_range, -1*rssi_noise, odometry, self.RRT.getArgosTime()/10,False, self.WF,self.RRT,outbound,
                        closest_distance_other_bot/100.0,priority,goal_angle_other)
        

        gb_state_nr=0
        if gb_state is "FORWARD":
            gb_state_nr = 1
        elif gb_state is "WALL_FOLLOWING":
            gb_state_nr = 2
        elif gb_state is "ROTATE_TO_GOAL":
            gb_state_nr = 3
        elif gb_state is "MOVE_OUT_OF_WAY":
            gb_state_nr= 4
        
        #numpy.savetxt('state_distance'+str(own_id)+'.txt',[gb_state_nr,closest_distance_other_bot] ,delimiter=',')
        
        '''if closest_distance_other_bot/100.0<0.2:
            twist = Twist()'''
        
        # Call the controller from gradient_bug_v1 (gradient_bug repository)
        '''
        if outbound is False:
            twist, self.rssi_goal_angle_adjust = self.GB.stateMachine(self.RRT.getRealDistanceToWall(),self.RRT.getRangeRight(),self.RRT.getRangeLeft(),
                        self.RRT.getHeading(),self.current_UWB_bearing, self.current_range, rssi_noise, odometry, self.RRT.getArgosTime()/10,False, self.WF,self.RRT,outbound)
        else:
            twist, self.rssi_goal_angle_adjust = self.GB2.stateMachine(self.RRT.getRealDistanceToWall(),self.RRT.getRangeRight(),self.RRT.getRangeLeft(),
                        self.RRT.getHeading(),self.current_UWB_bearing, self.current_range, rssi_noise, odometry, self.RRT.getArgosTime()/10,False, self.WF,self.RRT,outbound, self.bot_is_close)
        '''
        return twist, self.goal_angle, gb_state_nr, closest_distance_other_bot


    # See if a value is within a margin from the wanted value
    def logicIsCloseTo(self, real_value = 0.0, checked_value =0.0, margin=0.05):

        if real_value> checked_value-margin and real_value< checked_value+margin:
            return True
        else:
            return False


    def wrap_pi(self, angles_rad):
        return numpy.mod(angles_rad+numpy.pi, 2.0 * numpy.pi) - numpy.pi
