#!/usr/bin/env python

"""
Demo controller for controlling an ARGoS simulated robot via argos_bridge.
The algorithm implemented here is a simple state machine designed to push
pucks to the walls.  If the proximity sensor detects an obstacle (other robot
or wall) then it moves away from it for a fixed period.
"""
import rospy
import math, random
import numpy
import time
from argos_bridge.msg import Puck
from argos_bridge.msg import PuckList
from argos_bridge.msg import Proximity
from argos_bridge.msg import ProximityList
from geometry_msgs.msg import Twist
from scipy.stats._continuous_distns import beta


class WallFollowing:
    state = "START_WALL_FOLLOWING"
    time = 0
    state_start_time = 0
    last_range_front = 0
    last_range_front_before_lost = 0
    last_range_side = 0
    last_range_side_before_lost = 0
    last_heading = 0;
    heading_when_started_turning = 0;
    last_heading_before_lost = 0;
    distance_go_around_corner = 0;
    last_time = 0
    has_alligned = False;
    direction_turn = 1
    distance_from_wall = 0.5
    distance_go_around_corner_90 = 2.5*0.35;
    location_precision = 0.5

    MAX_FORWARD_SPEED=1.0
    MAX_ROTATION_SPEED=2.5

    def init(self):

        self.state = "START_WALL_FOLLOWING"
        self.time = 0
        self.state_start_time = 0
        self.last_range_front = 0
        self.last_range_front_before_lost = 0
        self.last_range_side = 0
        self.last_range_side_before_lost = 0
        self.last_heading = 0;
        self.heading_when_started_turning = 0;
        self.last_heading_before_lost = 0;
        self.distance_go_around_corner = 0;
        self.last_time = 0
        self.has_alligned = False;
        self.direction_turn = 1
        self.distance_from_wall = 0.4
        self.distance_go_around_corner_90 = 2.3*0.35;
        self.location_precision = 0.5




    def wallFollowingController(self, range_side, range_front, closest_obstacle,
                                 current_heading, time_argos, direction):

        self.direction_turn = direction

        self.time = time_argos
        #
        # Handle state transitions
        #
        if self.state =="START_WALL_FOLLOWING":
            self.last_heading = current_heading;
            self.transition("ROTATE_TO_ALIGN_WALL")
        elif self.state=="ROTATE_TO_ALIGN_WALL":
            self.last_time = time_argos
            if self.logicIsCloseTo(range_side,range_front*math.cos(numpy.deg2rad(60)), 0.1):
                self.transition("WALL_FOLLOWING")
            # if front range is lost, assume you are on the brink of a corner
            if range_front > 2.0:
                self.last_range_front_before_lost = self.last_range_front
                self.last_range_side_before_lost = self.last_range_side - 0.2
                self.last_heading_before_lost = self.last_heading
                self.heading_when_started_turning = current_heading + 1.50
                self.has_alligned = False
                # Calculate how far the bot is from the edge of the corner, which is variable when this happens while alligning to a wall
                self.distance_go_around_corner = self.last_range_side_before_lost*math.sin(self.heading_when_started_turning-self.last_heading_before_lost-0.52)
                self.transition("ROTATE_AROUND_CORNER")
        elif self.state == "WALL_FOLLOWING":
            # IF a close obstacle is found while wall following, assume you are in a corner
            if closest_obstacle<self.distance_from_wall+0.1:
                self.last_heading = current_heading;
                self.heading_when_started_turning = current_heading
                self.transition("ROTATE_TO_ALIGN_WALL")
            # If Front range is lost, you are on the brink of a corner
            if range_front > 2.0 :
                self.last_range_front_before_lost = self.last_range_front
                self.last_range_side_before_lost =  self.last_range_side


                self.last_heading_before_lost = self.last_heading
                # Since with wall following, we arlready assume that the bot is alligned with the wall, therefore these assumptions
                self.heading_when_started_turning = current_heading + 1.50
                self.distance_go_around_corner = self.distance_go_around_corner_90

                self.has_alligned = True
                self.transition("ROTATE_AROUND_CORNER")
        elif self.state=="ROTATE_AROUND_CORNER":
            # If the wall is found by the wanted triangle by ranges, go to wall following
            if self.logicIsCloseTo(range_side,range_front*math.cos(numpy.deg2rad(60)), 0.1) and range_front < 1.5 and self.logicIsCloseTo( self.last_heading_before_lost,current_heading, 1.50):
                self.transition("WALL_FOLLOWING")
            # If a obstacle is seen, assume you are in a corner so align with wall
            if closest_obstacle<self.distance_from_wall + 0.1:
                self.last_range_front_before_lost = self.last_range_front
                self.last_range_side_before_lost =  self.last_range_side
                self.last_heading_before_lost = self.last_heading
                self.distance_go_around_corner = self.last_range_side_before_lost*math.sin(self.heading_when_started_turning-self.last_heading_before_lost-0.52)
                self.transition("ROTATE_TO_ALIGN_WALL")
        elif self.state=="STOP_MOVING":
            print "stop moving"
        else:
            die("Error: Invalid state")

       # print "State WallFollowing: " + self.state

        #
        # Handle state actions
        #
        twist = Twist()
        if self.state == "WALL_FOLLOWING":
            # Do simple wall following based on the front range sensor of the wedge and the the left sensor
            twist = self.twistWallFollowing(range_side, range_front)
        elif self.state== "ROTATE_TO_ALIGN_WALL":
            twist = self.twistTurnInCorner(direction)
        elif self.state == "ROTATE_AROUND_CORNER":
            twist = self.twistStop()
            #TODO: do this based on odometry!
            if(current_heading-self.heading_when_started_turning>-1.50) and self.has_alligned == False:
                twist = self.twistTurnInCorner(direction)
            elif (current_heading-self.heading_when_started_turning<=-1.50) and self.has_alligned == False:
                self.has_alligned = True
                self.state_start_time = self.time
            if self.distance_go_around_corner > self.distance_go_around_corner_90:
                self.distance_go_around_corner = self.distance_go_around_corner_90

            if (self.time - self.state_start_time)<self.distance_go_around_corner/0.35 * 10 and self.has_alligned == True:
                twist = self.twistForward()
               # print "go forward"
            elif (self.time - self.state_start_time)>self.distance_go_around_corner/0.35 * 10  and self.has_alligned == True:
                twist = self.twistTurnAroundCorner(self.last_range_side_before_lost+0.3,direction)
                if(self.last_range_side_before_lost>999):
                    self.last_range_side_before_lost = self.distance_from_wall

        elif self.state == "STOP_MOVING":
            twist = self.twistStop()
        else:
            die("Error: Invalid state")
        self.last_range_front = range_front;
        self.last_range_side = range_side;
        self.last_heading = current_heading;
        return twist, self.state


    # Transition state and restart the timer
    def transition(self, newState):
        self.state = newState
        self.state_start_time = self.time

    # Wall following logic
    def twistWallFollowing(self, range_side = 0.0, range_front=0.0):
        v = self.MAX_FORWARD_SPEED
        w = 0.0

        #Calculating height of triangle if the angle between sides is known
        # combination of:
        # 1- SAS for triangle area
        # 2- Half base times height method
        # 3- Cosinus Rule
        beta = numpy.deg2rad(60)

        height=(range_side*range_front*math.sin(beta))/math.sqrt(math.pow(range_side,2)+math.pow(range_front,2)-2*range_side*range_front*math.cos(beta))

        #This is to compare the range side with to keep it perpendicular to the wall
        range_equal_to =  range_front*math.cos(numpy.deg2rad(60))

        #Most important, first check if the robot if away from the wall!
        if self.logicIsCloseTo(height, self.distance_from_wall, 0.1):
            #If so, just try to align to the wall by changing the angle
            #print "Align with wall"
            if range_side > range_equal_to -0.05 and range_front!=0.0:
                w=-0.15*self.direction_turn
            elif range_side < range_equal_to + 0.05 and range_front!= 0.0:
                w=0.15*self.direction_turn
            else:
                w=0
        else:
            #if not, increase or decrease the distance by changing the heading
            # print "Keep distance from wall"
             if height > self.distance_from_wall :
                 w=0.15*self.direction_turn
             elif height < self.distance_from_wall and range_front<1.8:
                 w=-0.15*self.direction_turn
             elif height < self.distance_from_wall and range_front>1.8:
                 w=0.15*self.direction_turn

             else:
                 w=0

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist

    def twistTurnInCorner(self,direction_turn):
        v = 0
        w = -1*direction_turn*self.MAX_ROTATION_SPEED
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist

    def twistTurn(self, rate,speed):
        v = speed
        w = rate
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist


    def twistTurnAroundCorner(self,radius, direction_turn):
        v = self.MAX_FORWARD_SPEED
        w = direction_turn*v/radius
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist

    def twistForward(self):
        v = self.MAX_FORWARD_SPEED
        w = 0
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist

    def twistStop(self):
        v = 0
        w = 0
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist

    def twistRotateToGoal(self):
        v = 0
        w = self.MAX_FORWARD_SPEED * numpy.sign(self.RRT.getUWBBearing())
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist

    def logicIsCloseTo(self, real_value = 0.0, checked_value =0.0, margin=0.05):

        if real_value> checked_value-margin and real_value< checked_value+margin:
            return True
        else:
            return False

    def wrap_pi(self, angles_rad):
        return numpy.mod(angles_rad+numpy.pi, 2.0 * numpy.pi) - numpy.pi

    def getWantedDistanceToWall(self):
        return self.distance_from_wall;

    def getDirectionTurn(self):
        return self.direction_turn;

    def getMaximumForwardSpeed(self):
        return self.MAX_FORWARD_SPEED

    def getMaximumRotationSpeed(self):
        return self.MAX_ROTATION_SPEED

    def getDistanceAroundCorner90(self):
        return self.distance_go_around_corner_90

    def getLocationPrecision(self):
        return self.location_precision
