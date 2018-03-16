#!/usr/bin/env python

import rospy
# from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String, Bool
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
import tf
import math
from enum import Enum
import numpy as np

# threshold at which we consider the robot at a location
POS_EPS = .1
THETA_EPS = .6

# time to stop at a stop sign
STOP_TIME = 100

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = .5

# time taken to cross an intersection
CROSSING_TIME = 3

# state machine modes, not all implemented
class Mode(Enum):
    IDLE = 1
    POSE = 2
    STOP = 3
    CROSS = 4
    NAV = 5
    MANUAL = 6
    STOP_INTERMEDIATE = 7

class State(Enum):
    PRE_EXPLORE = 7
    EXPLORE = 1
    PICKUP = 2
    RESCUE = 3
    HOME = 4
    CELEBRATION = 5

# pre_explore_waypoints = [(0.3, 0.0, 1.57), (0.2, 0.7, 1.57)]
pre_explore_waypoints = [(3.2, 1, .7), (3.1, .4, .7), (2.8, .33, 0)]

class Supervisor:
    """ the state machine of the turtlebot """

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)

        # current pose
        self.x = 0
        self.y = 0
        self.theta = 0

        # pose goal
        self.x_g = 0
        self.y_g = 0
        self.theta_g = 0

        # camera offset
        self.x_off = 0
        self.y_off = 0
        self.th_off = 0

        # current mode
        self.mode = Mode.IDLE
        # self.state = State.PRE_EXPLORE
        self.state = State.EXPLORE
        self.last_mode_printed = None

    	self.stop_sign_start = rospy.get_rostime()

        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.animal_positions = [] # Animal positions is a list of tuples
        self.animal_index = 0
        self.NUM_ANIMALS = 3 # Boolean indicating whether we should rescure or not 
        self.pre_explore_index = -1
        self.ANIMAL_DIST_THRESH = 0.04 # it is the distance squared in centimeters

        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
       
        # All your fav animals

        rospy.Subscriber('/detector/bird', DetectedObject, self.animal_detected_callback)
        rospy.Subscriber('/detector/cat', DetectedObject, self.animal_detected_callback)
        rospy.Subscriber('/detector/dog', DetectedObject, self.animal_detected_callback)
        rospy.Subscriber('/detector/horse', DetectedObject, self.animal_detected_callback)
        rospy.Subscriber('/detector/sheep', DetectedObject, self.animal_detected_callback)
        rospy.Subscriber('/detector/cow', DetectedObject, self.animal_detected_callback)
        rospy.Subscriber('/detector/elephant', DetectedObject, self.animal_detected_callback)
        rospy.Subscriber('/detector/bear', DetectedObject, self.animal_detected_callback)
        rospy.Subscriber('/detector/zebra', DetectedObject, self.animal_detected_callback)
        rospy.Subscriber('/detector/giraffe', DetectedObject, self.animal_detected_callback)
        

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        rospy.Subscriber('/rescue_on', Bool, self.rescue_on_callback)
        self.rescue_pub = rospy.Publisher('/ready_to_rescue', Bool, queue_size = 10)

        self.trans_listener = tf.TransformListener()
        self.trans_broadcaster = tf.TransformBroadcaster()

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """

        self.x_g = msg.pose.position.x
        self.y_g = msg.pose.position.y
        rotation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(rotation)
        self.theta_g = euler[2]

        self.mode = Mode.NAV

    def stop_sign_detected_callback(self, msg):

        """ callback for when the detector has found a stop sign. Note that

        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign

        dist = msg.distance/100.0
        theta_sign = (msg.thetaleft + msg.thetaright)/2.0
        print "STOP SIGN "

        # if close enough and in nav mode, stop
        if dist > 0 and dist < STOP_MIN_DIST and self.mode == Mode.NAV:
            self.mode = Mode.STOP_INTERMEDIATE
            self.x_og_g = self.x_g
            self.y_og_g = self.y_g
            self.theta_og_g = self.theta_g
            delta_d = np.abs(dist*np.cos(theta_sign))
            theta_sign = theta_sign + self.theta

            if np.abs(self.theta) < np.radians(30): #facing forward
                self.x_g = self.x + delta_d
                self.y_g = self.y
                self.theta_g = 0

            elif np.abs(self.theta) > np.radians(150): #facing back
                self.x_g = self.x - delta_d
                self.y_g = self.y
                self.theta_g = -np.pi

            elif self.theta > 0: #facing left
                self.x_g = self.x
                self.y_g = self.y + delta_d
                self.theta_g = np.pi/2

            else: #facing right
                self.x_g = self.x
                self.y_g = self.y - delta_d
                self.theta_g = -np.pi/2
	    print 'x_g:', self.x_g, '\t y_g', self.y_g, '\t theta_g', self.theta_g


    def animal_detected_callback(self, msg):

        #theta_animal = (msg.thetaleft + msg.thetaright)/2.0

        animal_theta = self.theta
        animal_x = self.x + 0.2*np.cos(animal_theta)
        animal_y = self.y + 0.2*np.sin(animal_theta)
        
        if self.not_close_to_other_animals(animal_x, animal_y) and self.state == State.EXPLORE:

            self.animal_positions.append((animal_x, animal_y, animal_theta))
            
            self.trans_broadcaster.sendTransform((animal_x, animal_y, 0), 
                                    tf.transformations.quaternion_from_euler(0, 0, self.theta), 
                                    rospy.Time.now(), '/animal_frame', '/map')

            print "Recorded New Animal", msg.name
            print "Current position:", self.x, self.y, self.theta
            print "Animal Positions: ", self.animal_positions
            
        print "Recorded animal"

    def rescue_on_callback(self, msg):
        if self.state == State.PICKUP and self.rescue_bool:
            self.x_g = self.animal_positions[0][0]
            self.y_g = self.animal_positions[0][1]
            self.theta_g = self.animal_positions[0][2]
            self.state = State.RESCUE
            self.mode = Mode.NAV

    def go_to_pose(self):
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired pose to the naviagtor """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g

        self.nav_goal_publisher.publish(nav_g_msg)

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self,x,y,theta):
        """ checks if the robot is at a pose within some threshold """

        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)
    
    def not_close_to_other_animals(self, x, y):
        """ checks if the robot is at a pose within some threshold """
        for animal_pos in self.animal_positions:
            print (animal_pos[0] - x)**2 + (animal_pos[1] -y)**2
            if ((animal_pos[0] - x)**2 + (animal_pos[1] -y)**2) < self.ANIMAL_DIST_THRESH:
                return False

        return True

        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)
    def init_stop_sign(self):
        """ initiates a stop sign maneuver """

        self.stop_sign_start = rospy.get_rostime()
        self.mode = Mode.STOP

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return (self.mode == Mode.STOP and (rospy.get_rostime()-self.stop_sign_start)>rospy.Duration.from_sec(STOP_TIME))

    def init_crossing(self):
        """ initiates an intersection crossing maneuver """

        self.cross_start = rospy.get_rostime()
        self.mode = Mode.CROSS

    def has_crossed(self):
        """ checks if crossing maneuver is over """

        return (self.mode == Mode.CROSS and (rospy.get_rostime()-self.cross_start)>rospy.Duration.from_sec(CROSSING_TIME))

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        try:
            (translation,rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            self.x = translation[0]
            self.y = translation[1]
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.theta = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass


        # logs the current mode
        # rospy.loginfo("Current Mode: %s", self.mode)
        # rospy.loginfo("Current State: %s", self.state)
        if not(self.last_mode_printed == self.mode):
            rospy.loginfo("Current Mode: %s", self.mode)
            self.last_mode_printed = self.mode

        #### State Machine for Navigation ####
        
        if self.mode == Mode.IDLE:
            # send zero velocity
            self.stay_idle()

        elif self.mode == Mode.POSE:
            # moving towards a desired pose
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.go_to_pose()

        elif self.mode == Mode.STOP:
            # at a stop sign
            if self.has_stopped():
                self.init_crossing()
            else:
                self.stay_idle()

        elif self.mode == Mode.STOP_INTERMEDIATE:

            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.x_g = self.x_og_g
                self.y_g = self.y_og_g
                self.theta_g = self.theta_og_g
                self.mode = Mode.STOP
            elif ((self.x - self.x_g)**2 + (self.y - self.y_g)**2)< 0.04:
                self.go_to_pose()
            else:
                self.nav_to_pose()

        elif self.mode == Mode.CROSS:
            # crossing an intersection
            if self.has_crossed():
                self.mode = Mode.NAV
            else:
                self.nav_to_pose()

        elif self.mode == Mode.NAV:
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.nav_to_pose()

        else:
            raise Exception('This mode is not supported: %s'
                % str(self.mode))


        #### State Machine for Mission ####

        if self.state == State.PRE_EXPLORE:
            # if (self.x == 0 and self.y == 0 and self.theta == 0):
            #     print '---------------- WE ARE AT ZERO'
            # print 'current position:', self.x, ', ', self.y, ', ', self.theta
            if self.mode == Mode.IDLE:

                # Zero ourself in the beginning
                if self.pre_explore_index == -1:
                    print '---------------- PRE_EXPLORE: zeroing'
                    self.x = 0.0
                    self.y = 0.0
                    self.theta = 0.0

                self.pre_explore_index += 1

                # Done going through waypoints
                if self.pre_explore_index >= len(pre_explore_waypoints):
                    print '---------------- PRE_EXPLORE: done exploring' 
                    self.state = State.EXPLORE

                # Done going to current waypoint, go to next one
                else:
                    print '---------------- PRE_EXPLORE: index', self.pre_explore_index, 'waypoint', pre_explore_waypoints[self.pre_explore_index]
                    self.x_g, self.y_g, self.theta_g = pre_explore_waypoints[self.pre_explore_index]
                    self.mode = Mode.NAV

        elif self.state == State.EXPLORE:

            if len(self.animal_positions) == self.NUM_ANIMALS:
                self.state = State.PICKUP
                self.x_g = 0
                self.y_g = 0
                self.theta_g = 0
                self.mode = Mode.NAV

        elif self.state == State.PICKUP:

            if self.mode == Mode.IDLE:

                self.rescue_bool = True
                rospy.loginfo(self.rescue_bool)
                self.rescue_pub.publish(self.rescue_bool)


        elif self.state == State.RESCUE:

            print "RESCUING"
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.animal_index += 1 
                if self.animal_index >= len(self.animal_positions):
                    self.x_g = 0
                    self.y_g = 0
                    self.theta_g = 0                  
                    self.mode == Mode.HOME               
                else:
                    self.x_g = self.animal_positions[self.animal_index][0]
                    self.y_g = self.animal_positions[self.animal_index][1]
                    self.theta_g = self.animal_positions[self.animal_index][2]
                    self.mode = Mode.NAV

        elif self.state == State.HOME:

            if self.close_to(self.x_g,self.y_g,self.theta_g):
                
                self.state = State.CELEBRATION
                self.x_g = 0
                self.y_g = 0
                self.theta_g = 0
                self.mode = Mode.NAV

        elif self.state == State.CELEBRATION:
            oldTh = self.theta_g
            self.theta_g = np.pi + oldTh
            pose_g_msg = Pose2D()
            pose_g_msg.x = self.x_g
            pose_g_msg.y = self.y_g
            pose_g_msg.theta = self.theta_g
            self.pose_goal_publisher.publish(pose_g_msg)
            time.sleep(random.randint(1, 5))
            oldTh = self.theta_g
            self.theta_g = oldTh - np.pi
            pose_g_msg = Pose2D()
            pose_g_msg.x = self.x_g
            pose_g_msg.y = self.y_g
            pose_g_msg.theta = self.theta_g
            self.pose_goal_publisher.publish(pose_g_msg)
            time.sleep(random.randint(1, 5))

        else:
            raise Exception('This state is not supported: %s'
                % str(self.state))


    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
