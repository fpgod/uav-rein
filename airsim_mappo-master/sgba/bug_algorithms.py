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
from argos_bridge.srv import GetCmds
from argos_bridge.srv import GetCmdsResponse
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from gradient_bug.msg import goal_angle

from std_srvs.srv import Empty

import message_filters


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

from geometry_msgs.msg import Twist
from scipy.stats._continuous_distns import beta
import wall_following
import receive_rostopics
import re

class BugAlgorithms:
    cmdVelPub = None
    goalAnglePub=None
    bug_type="com_bug";
    bug_controller =  com_bug_controller.ComBugController()
    RRT = receive_rostopics.RecieveROSTopic()
    WF=wall_following.WallFollowing()
    reset_bug = False
    random_environment = False;
    odometry=[0,0];
    twist = Twist()
    noise_level = 0.4;
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
    def getController(self,argument):
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

    def __init__(self):
        # Subscribe to topics and init a publisher
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.goalAnglePub = rospy.Publisher('goal_angle', goal_angle, queue_size=10)
        self.madeItPub = rospy.Publisher('made_it', Bool, queue_size=10)

        #rospy.Subscriber('proximity', ProximityList, self.RRT.prox_callback,queue_size=100)
        #rospy.Subscriber('rangebearing', RangebearingList, self.RRT.rab_callback,queue_size=100)
        #rospy.Subscriber('position', PoseStamped, self.RRT.pose_callback,queue_size=100)
        rospy.Subscriber('position', PoseStamped, self.RRT.pose_callback,queue_size=10)
        rospy.Subscriber('/tower/position', PoseStamped, self.RRT.pose_callback_tower,queue_size=10)
        rospy.Subscriber('/switch_bug', String, self.switchBug,queue_size=10)
        rospy.Subscriber('/random_environment', Bool, self.random_environment,queue_size=10)
        rospy.Subscriber('/noise_level', Float64, self.noise_level_cb,queue_size=10)
        rospy.Subscriber('RSSI_to_tower', Float32, self.RRT.rssi_tower_callback,queue_size=10)


        
          
        rospy.Subscriber("/bot1/goal_angle",goal_angle,self.RRT.goal_angle_bot1_callback)
        rospy.Subscriber("/bot2/goal_angle",goal_angle,self.RRT.goal_angle_bot2_callback)
        rospy.Subscriber("/bot3/goal_angle",goal_angle,self.RRT.goal_angle_bot3_callback)
        rospy.Subscriber("/bot4/goal_angle",goal_angle,self.RRT.goal_angle_bot4_callback)
        rospy.Subscriber("/bot5/goal_angle",goal_angle,self.RRT.goal_angle_bot5_callback)
        rospy.Subscriber("/bot6/goal_angle",goal_angle,self.RRT.goal_angle_bot6_callback)
        rospy.Subscriber("/bot7/goal_angle",goal_angle,self.RRT.goal_angle_bot7_callback)
        rospy.Subscriber("/bot8/goal_angle",goal_angle,self.RRT.goal_angle_bot8_callback)
        rospy.Subscriber("/bot9/goal_angle",goal_angle,self.RRT.goal_angle_bot9_callback)

        rospy.Subscriber("/bot1/made_it",Bool,self.RRT.made_it_bot1_callback)
        rospy.Subscriber("/bot2/made_it",Bool,self.RRT.made_it_bot2_callback)
        rospy.Subscriber("/bot3/made_it",Bool,self.RRT.made_it_bot3_callback)
        rospy.Subscriber("/bot4/made_it",Bool,self.RRT.made_it_bot4_callback)
        rospy.Subscriber("/bot5/made_it",Bool,self.RRT.made_it_bot5_callback)
        rospy.Subscriber("/bot6/made_it",Bool,self.RRT.made_it_bot6_callback)
        rospy.Subscriber("/bot7/made_it",Bool,self.RRT.made_it_bot7_callback)
        rospy.Subscriber("/bot8/made_it",Bool,self.RRT.made_it_bot8_callback)
        rospy.Subscriber("/bot9/made_it",Bool,self.RRT.made_it_bot9_callback)

        
        '''
        goal_angle_sub = []
        topic_name = "/bot"+ str(it+1)+"/goal_angle"
           
            
        goal_angle_sub_temp = message_filters.Subscriber(topic_name,goal_angle)
            goal_angle_sub.append(goal_angle_sub_temp)
        ts = message_filters.TimeSynchronizer([goal_angle_sub[0],goal_angle_sub[1]], 10)
        ts.registerCallback(self.RRT.goal_angle_callback)
        '''
        
        #Wait for services to begin
        rospy.wait_for_service('/start_sim')
        s1 = rospy.Service('get_vel_cmd',GetCmds,self.runStateMachine)

        try:
            start_sim = rospy.ServiceProxy('/start_sim', StartSim)
            
	    # Start sim with indoor environment from file (from indoor environment generator package)
            start_sim(4,1,1)
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e

        self.bug_type = rospy.get_param('bug_algorithms/bug_type')
        self.outbound_angle = rospy.get_param('bug_algorithms/outbound_angle')

        self.bug_controller = self.getController(self.bug_type);
        self.bug_controller.__init__()

        if self.bug_controller == False:
            print "Wrong bug type!"
        
        self.send_stop = False


    # Ros loop were the rate of the controller is handled
    def rosLoop(self):

        rospy.spin()


    def switchBug(self,req):
        print("SWITCH BUG")
        self.bug_controller = self.getController(req.data);
        self.bug_type = req.data
        self.reset_bug = True
        self.opened_file = False
        print req.data
        try:
            start_sim = rospy.ServiceProxy('/start_sim', StartSim)
            if(self.random_environment):
                #start_sim(1,1,1)
                start_sim(4,1,1)
                self.random_environment = False
                print "python, send regenerate environment"
            else:
                start_sim(2,1,1)
                print "python, reset experiment with same environment"


        except rospy.ServiceException as e:
            print "Service call failed: %s"%e
        if self.bug_controller == False:
            print "Wrong bug type!"

    def runStateMachine(self, req):

        rospy.wait_for_service('/stop_sim')
        try:
            stop_sim = rospy.ServiceProxy('/stop_sim', Empty)
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e
            
        #Send values from service to the rostopic retrieval 
        self.RRT.prox_callback(req.proxList);
        self.RRT.rab_callback(req.RabList);
        self.RRT.pose_callback(req.PosQuat);
        
        
        
        
        # Get the current position of the bot
        pose_bot = PoseStamped()
        pose_bot = self.RRT.getPoseBot()
        
        # Get the distance to the tower
        distance_to_tower = math.sqrt(math.pow(pose_bot.pose.position.x,2)+math.pow(pose_bot.pose.position.y,2))
        angle_to_tower = math.atan2(pose_bot.pose.position.y, pose_bot.pose.position.x)
        
        number_id = int(filter(str.isdigit, req.botID.data))
        
       # print self.RRT.getArgosTime()/10 
        
        # If the bug algorithm is reset or the bug has been changed, initialize everythong
        if req.reset or self.reset_bug:
            self.WF.init()
            self.bug_controller.__init__()
            self.reset_bug = False
            self.odometry = PoseStamped()
            self.odometry_perfect = PoseStamped()
            self.send_stop = False
            self.outbound = True
            self.start_time = self.RRT.getArgosTime()
            
        if req.reset or self.opened_file is False:
            self.F = open("trajectory"+str(number_id)+".txt","w")
            self.F_state = open("state_distance"+str(number_id)+".txt","w")
            self.opened_file = True 
            
        #Get time and change outbound to back after 3 min
        if ( self.RRT.getArgosTime()-self.start_time)/10>300+number_id*10:
            self.outbound = False
            
            
        if self.RRT.getArgosTime()%1000 is 0:
            print(self.RRT.getArgosTime()-self.start_time)
            
        # If the bug is not near the tower (yet)
        #if self.RRT.getArgosTime()%100 is 0:
        #    print(distance_to_tower)
            
        if ((distance_to_tower>2.5 or self.outbound == True )):
            
            self.F.write("%5.2f, %5.2f\n" %(req.PosQuat.pose.position.x,req.PosQuat.pose.position.y));       
           # self.pos_save.append(req.PosQuat.pose.position.x)
           # self.pos_save.append(req.PosQuat.pose.position.y)
           # self.pos_save.append([])

            #Select the bug statemachine based on the launch file
            gb_state_nr = 0
            closest_distance_other_bot = 0
            if self.bug_type == 'alg_1':
                self.twist = self.bug_controller.stateMachine(self.RRT,self.get_odometry_from_commands(0.0),0.0,self.noise_level)
            elif self.bug_type == 'alg_2' :
                self.twist = self.bug_controller.stateMachine(self.RRT,self.get_odometry_from_commands(0.0),0.0,self.noise_level,0.0)
            elif self.bug_type == 'i_bug':
                self.twist = self.bug_controller.stateMachine(self.RRT,self.get_odometry_from_commands(0.0),self.noise_level)
            else:
                if self.RRT.getArgosTime()/10<number_id*10:
                    self.twist= Twist()
                else:
                    self.twist, self.angle_goal, gb_state_nr, closest_distance_other_bot = self.bug_controller.stateMachine(self.RRT,self.get_odometry_from_commands(self.noise_level), self.outbound, self.outbound_angle, number_id)
                
                
            self.F_state.write("%d, %5.2f\n" %(gb_state_nr, closest_distance_other_bot));       

            #Save values for the debugging (matlab)
                # Odometry with drift
           # numpy.savetxt('rel_x.txt',[self.odometry.pose.position.x],delimiter=',')
           # numpy.savetxt('rel_y.txt',[self.odometry.pose.position.y],delimiter=',')
                # Odometry perfect
           # self.get_odometry_from_commands_perfect()
           # numpy.savetxt('rel_x_per.txt',[self.odometry_perfect.pose.position.x],delimiter=',')
           # numpy.savetxt('rel_y_per.txt',[self.odometry_perfect.pose.position.y],delimiter=',') 
            
            msg_temp = goal_angle()
            msg_temp.goal_angle.data = self.angle_goal
            msg_temp.header.stamp = rospy.Time.now()
            self.goalAnglePub.publish(msg_temp)
            
            #Return the commands to the controller  
            msg_madeit = Bool()
            msg_madeit.data = False;
            self.madeItPub.publish(msg_madeit)         
            return GetCmdsResponse(self.twist)
        else:
            #Stop the bug and sent a stop signal for the simulator
            
            if(distance_to_tower<0.5):
                self.twist.linear.x = 0
                self.twist.angular.z = 0
            else:
                self.twist = self.go_to_tower(angle_to_tower)
            
            
            #self.F = open("trajectory"+str(number_id)+".txt","w")
           # self.F.close()
           
            msg_madeit = Bool()
            msg_madeit.data = True;
            self.madeItPub.publish(msg_madeit) 

            if( self.send_stop is False):
              #  numpy.savetxt("trajectory"+str(number_id)+".txt",self.pos_save,delimiter=',')
                print "bug has reached goal"
                stop_sim()
                self.send_stop = True
                self.outbound = True
                self.F.close()
            return GetCmdsResponse(self.twist)

    # Make a (noisy) odometry measurment based on the inputs on the system, based on a guassian, returns an odometry position.
    def get_odometry_from_commands(self,noise):
        current_time=float(self.RRT.getArgosTime())/10
        diff_time = current_time - self.previous_time
        if noise < 0.01:
            noisy_velocity_estimate = self.twist.linear.x*0.35
            noisy_heading = self.RRT.getHeading()
        else:
            noisy_velocity_estimate = numpy.random.normal(self.twist.linear.x*0.35,noise,1);
            noisy_heading =  numpy.random.normal(self.RRT.getHeading(),noise,1);
        self.odometry.pose.position.x = self.odometry.pose.position.x + diff_time*noisy_velocity_estimate*math.cos(noisy_heading)
        self.odometry.pose.position.y = self.odometry.pose.position.y + diff_time*noisy_velocity_estimate*math.sin(noisy_heading)
        self.previous_time = current_time
        return self.odometry
    
    # Returns a perfect odometry, for comparison purposes
    def get_odometry_from_commands_perfect(self):
        noisy_velocity_estimate = self.twist.linear.x*0.035
        self.odometry_perfect.pose.position.x = self.odometry_perfect.pose.position.x + noisy_velocity_estimate*math.cos(self.RRT.getHeading())
        self.odometry_perfect.pose.position.y = self.odometry_perfect.pose.position.y + noisy_velocity_estimate*math.sin(self.RRT.getHeading())
        return self.odometry
    
    # Retrieve command if need to make another environment
    def random_environment(self,req):
        self.random_environment = req.data;

    # Retrieve noise level from topic
    def noise_level_cb(self,req):
        self.noise_level = req.data;
    
    def go_to_tower(self,angle_tower):
        twist = Twist()
        if self.logicIsCloseTo(self.wrap_pi(3.14-(self.RRT.getHeading()-angle_tower)),0,0.1):
            twist.linear.x = 0.5
            twist.angular.z = 0
        else:
            twist.linear.x = 0
            twist.angular.z = 1.0
        return twist
        
    # See if a value is within a margin from the wanted value
    def logicIsCloseTo(self, real_value = 0.0, checked_value =0.0, margin=0.05):

        if real_value> checked_value-margin and real_value< checked_value+margin:
            return True
        else:
            return False


    def wrap_pi(self, angles_rad):
        return numpy.mod(angles_rad+numpy.pi, 2.0 * numpy.pi) - numpy.pi


if __name__ == '__main__':

    time.sleep(2)
    rospy.init_node("bug_algorithms")
    controller = BugAlgorithms()
    
    try:
        controller.rosLoop()
    except rospy.ROSInterruptException:
        pass
