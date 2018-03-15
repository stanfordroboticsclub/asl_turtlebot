#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String, Bool
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
import tf
import math
from enum import Enum
import numpy as np

# threshold at which we consider the robot at a location
POS_EPS = .1
THETA_EPS = .3

# time to stop at a stop sign
STOP_TIME = 3

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

class State(Enum):
    PRE_EXPLORE = 7
    EXPLORE = 1
    PICKUP = 2
    RESCUECAT = 3
    RESCUEDOG = 4
    HOME = 5
    CELEBRATION = 6

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

        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.cat_test = 0
        self.dog_test = 0
        self.cat_position = np.array([0, 0, 0])
        self.rescue_bool = False
        self.dog_positon = np.copy(self.cat_position)

        self.pre_explore_index = -1

        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
        rospy.Subscriber('/detector/cat', DetectedObject, self.cat_detected_callback)
        # rospy.Subscriber('/detector/dog', DetectedObject, self.dog_detected_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        rospy.Subscriber('/ready_to_rescue', Bool, self.ready_to_rescue_callback)
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
        dist = msg.distance

        # if close enough and in nav mode, stop
        if dist > 0 and dist < STOP_MIN_DIST and self.mode == Mode.NAV:
            self.init_stop_sign()

        print "STOP SIGN "


    def cat_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        if self.cat_test ==0:
            print msg
            self.cat_test = 1
            dist_cat = msg.distance
            theta_cat = (msg.thetaleft + msg.thetaright)/2.0

            x_off = self.x_off
            y_off = self.y_off
            th_off = self.th_off
            x = self.x
            y = self.y
            th = self.theta

            #### TODO ####
            # compute h, Hx
            ##############
            self.cat_position[2] = th + th_off + theta_cat
            self.cat_position[0] = x + x_off*np.cos(th) - y_off*np.sin(th) + dist_cat*np.cos(th + th_off + theta_cat)
            self.cat_position[1] = y + x_off*np.sin(th) + y_off*np.cos(th) + dist_cat*np.cos(th + th_off + theta_cat)
            # self.trans_broadcaster.sendTransform((self.cat_position[0], self.cat_position[1], 0), tf.transformations.quaternion_from_euler(0, 0, self.cat_position[2]), rospy.Time.now(), '/cat_frame', '/map')

            print "CAT HAS ARRIVED"

    def ready_to_rescue_callback(self, msg):
        if self.state == State.PICKUP and self.rescue_bool:
            self.x_g = self.cat_position[0]
            self.y_g = self.cat_position[1]
            self.theta_g = self.cat_position[2]
            self.state = State.RESCUECAT
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
                pass

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

            if self.cat_test == 1: #and self.dog_test == 1:
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


        elif self.state == State.RESCUECAT:

            if self.mode == Mode.IDLE:
                
                self.state = State.HOME
                self.x_g = 0
                self.y_g = 0
                self.theta_g = 0
                self.mode = Mode.NAV

        #elif self.state == State.RESCUEDOG:

        elif self.state == State.HOME:

            if self.mode == Mode.IDLE:
                
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
            self.mode = Mode.CELEBRATION

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