#!/usr/bin/env python


### Author: Jurica Papak

import sys
import rospy
import numpy as np
import tf2_ros
import tf
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from obstacle_detector.msg import Obstacles, CircleObstacle


class GuideController:

    NOT_TRACKING = 0
    TRACKING = 1
    ACTOR_IN_FRONT = 2

    def __init__(self):

        rospy.init_node('guide_controller') # initialize the node
        self.mainloop_rate = rospy.Rate(2) # set control rate

        self.GUIDE_DISTANCE = rospy.get_param("~guide_distance", 2.0)  # how far the robot will try to stay ahead of the actor
        self.ACTOR_TOL = rospy.get_param("~actor_tolerance", 0.2)  # tolerance of actor movement in a timestep
        self.SIGMOID_FACTOR = np.log(99)/self.GUIDE_DISTANCE # factor for shaping the sigmoid

        self.tf_buffer = tf2_ros.Buffer()   # transform buffer
        self.map_frame = rospy.get_param("~map_frame", "map")  # transform frame of the map (static)
        self.robot_frame = rospy.get_param("~robot_frame", "base_link")  # transform frame of the robot (dynamic)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=2)  # command velocity publisher
        self.actor_pub = rospy.Publisher('tracked_actor', CircleObstacle, queue_size=2) # actor location publisher
        self.robot_pub = rospy.Publisher('robot_position', Pose, queue_size=2) # robot location publisher

        self.actor = CircleObstacle() # last known actor location and velocity

        self.robot = Pose() # last known robot location and heading
        self.robot.position.x = None
        self.robot.position.y = None
        self.robot.orientation.z = None # [-pi, pi]

        self.guide_status = self.NOT_TRACKING # status of the guide controller

        listener = tf2_ros.TransformListener(self.tf_buffer) # setup transform buffer
        self.mainloop_rate.sleep() # give buffer time to fill up

        # initialize odometry subscriber for robot position initialization
        rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        while(self.robot.position.x == None):
            rospy.loginfo("GuideController: Waiting for odometry callback to set initial position..")
            self.mainloop_rate.sleep() # wait for the first odometry callback
        # initialize the rest of the subscribers
        rospy.Subscriber('cmd_vel_dwb', Twist, self.cmd_vel_callback, queue_size=1)
        rospy.Subscriber('obstacles', Obstacles, self.obstacle_callback, queue_size=1)
        rospy.loginfo("GuideController: Initialization successful.")

    def sigmoid(self, x):
        '''Sigmoid defined around keeping the actor at the GUIDE_DISTANCE'''
        return 2 / (1 + np.exp(self.SIGMOID_FACTOR*(x-self.GUIDE_DISTANCE)))

    def wrap_to_pi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def get_distance_vector(self):
        '''Returns the vector pointing from actor to robot'''
        return np.array([self.robot.position.x - self.actor.center.x, self.robot.position.y - self.actor.center.y])

    def actor_exceeds_angle(self):
        '''Returns true if actor is not behind robot (+-90 degrees)'''
        dist_vec = self.get_distance_vector()
        angle = self.robot.orientation.z - np.arctan2(dist_vec[1], dist_vec[0])
        wrapped_angle = self.wrap_to_pi(np.abs(angle))
        if np.abs(wrapped_angle) > np.pi/2:
            return True
        return False

    def correct_for_distance(self, original_vel, distance):
        factor = self.sigmoid(distance)
        if factor < 0.05:
            factor = 0
        original_vel.linear.x *= factor
        if np.abs(original_vel.linear.x) < 0.05:
            original_vel.linear.x = 0
        original_vel.angular.z *= factor

    def correct_for_angle(self, original_vel, angle, robot_heading):
        pass

    def cmd_vel_callback(self, cmd_vel):
        '''Augments the command velocity according to the actor-robot
            relative position and publishes it to the regular topic'''
        if self.guide_status == self.TRACKING:
            dist_vector = self.get_distance_vector()
            # distance correction
            distance = np.linalg.norm(dist_vector)
            self.correct_for_distance(cmd_vel, distance)
            # angle correction
            angle = np.arctan2(dist_vector[1], dist_vector[0])
            wrapped_angle = self.wrap_to_pi(np.abs(self.robot.orientation.z - angle))
            self.correct_for_angle(cmd_vel, wrapped_angle, self.robot.orientation.z)
        elif self.guide_status == self.NOT_TRACKING: # stop the robot
            cmd_vel = Twist()

        self.cmd_vel_pub.publish(cmd_vel)

    def obstacle_callback(self, obstacles):
        '''Handles initializing and updating the actor position'''
        if self.guide_status == self.TRACKING: # if tracking, update actor position
            tracked = False
            for circle in obstacles.circles:
                x, y = circle.center.x, circle.center.y
                if np.abs(x - self.actor.center.x) < self.ACTOR_TOL and \
                   np.abs(y - self.actor.center.y) < self.ACTOR_TOL:
                    self.actor.center.x, self.actor.center.y = x, y
                    self.actor.velocity.x, self.actor.velocity.y = circle.velocity.x, circle.velocity.y
                    tracked = True
                    break
            if tracked:
                self.guide_status = self.TRACKING
            else:
                self.guide_status = self.NOT_TRACKING
        elif self.guide_status == self.NOT_TRACKING: # if not tracking, attempt finding actor
            self.get_actor(obstacles)

    def odom_callback(self, odom):
        '''Updates the robot pose'''
        trans = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rospy.Time())

        self.robot.position.x = trans.transform.translation.x
        self.robot.position.y = trans.transform.translation.y

        quaternion = (trans.transform.rotation.x, trans.transform.rotation.y,
                      trans.transform.rotation.z, trans.transform.rotation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.robot.orientation.z = euler[2] # yaw angle used for robot heading


    def get_actor(self, obstacles):
        '''Tries to separate the actor from the obstacles'''
        best_candidate = None
        min_distance = 10000 # artbitrarily large number
        for circle in obstacles.circles:
            dist_vector = np.array([self.robot.position.x - circle.center.x, self.robot.position.y - circle.center.y])
            angle = np.arctan2(dist_vector[1], dist_vector[0])
            wrapped_angle = self.wrap_to_pi(np.abs(self.robot.orientation.z - angle))
            if np.abs(wrapped_angle) < 1:
                distance = np.linalg.norm(dist_vector)
                if distance < min_distance:
                    best_candidate = (circle.center.x, circle.center.y)
                    min_distance = distance

        if best_candidate == None:
            rospy.loginfo("GuideController: No suitable actor found!")
        else:
            self.actor.center.x = best_candidate[0]
            self.actor.center.y = best_candidate[1]
            self.guide_status = self.TRACKING
            print("GuideController: Actor at location (%1.2f, %1.2f) chosen." % (self.actor.center.x, self.actor.center.y))
            rospy.loginfo("GuideController: Actor at location (%1.2f, %1.2f) chosen." % (self.actor.center.x, self.actor.center.y))

    def run(self):
        while(1):
            if self.guide_status == self.TRACKING or self.guide_status == self.ACTOR_IN_FRONT:
                if self.actor_exceeds_angle():
                    self.guide_status = self.ACTOR_IN_FRONT
                    rospy.loginfo("GuideController: Actor in front of robot.")
                else:
                    self.guide_status = self.TRACKING
                self.robot_pub.publish(self.robot)
                self.actor_pub.publish(self.actor) # publish actor info
            elif self.guide_status == self.NOT_TRACKING:
                rospy.loginfo("GuideController: Searching for actor behind robot.")
            self.mainloop_rate.sleep()

if __name__ == '__main__':
    try:
        controller = GuideController()
        controller.run()
    except rospy.ROSInterruptException, RuntimeError:
        pass
