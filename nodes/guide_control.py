#!/usr/bin/env python


### Author: Jurica Papak

import sys
import rospy
import numpy as np
import tf
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from obstacle_detector.msg import Obstacles, CircleObstacle


class GuideController:

    ACTOR_TOL = 0.2 # tolerance of actor movement in a timestep
    GUIDE_DISTANCE = 1  # how far the robot will try to stay ahead of the actor
    SIGMOID_FACTOR = np.log(99)/GUIDE_DISTANCE # factor for shaping the sigmoid

    def __init__(self):
        # transform buffer
        self.tf_buffer = tf2_ros.Buffer()
        # command velocity publisher
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=2)
        # last known actor location
        self.actor_loc_x = 0
        self.actor_loc_y = 0
        # last known actor velocity
        self.actor_vel_x = 0
        self.actor_vel_y = 0
        # last known robot location and heading
        self.robot_loc_x = 0
        self.robot_loc_y = 0
        self.robot_angle = 0 # [-pi, pi]
        # status of the guide controller:
        # 0 - no actor available for guiding, finding actor
        # 1 - locked on actor, guiding
        self.guide_status = 0

        # initialize the node
        rospy.init_node('guide_control')

        # setup transform buffer and wait for it to fill up
        rate = rospy.Rate(1)
        listener = tf2_ros.TransformListener(self.tf_buffer)
        rate.sleep()

        # determine initial robot pose
        self.get_robot()

        # subscriber initialization
        rospy.Subscriber('cmd_vel_dwb', Twist, self.cmd_vel_callback, queue_size=1)
        rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('obstacles', Obstacles, self.obstacle_callback, queue_size=1)
        print ("[GUIDE_CONTROL]: Initialization successful.")

    # sigmoid defined around keeping the actor at the GUIDE_DISTANCE
    def sigmoid(self, x):
        return 2 / (1 + np.exp(self.SIGMOID_FACTOR*(x-self.GUIDE_DISTANCE)))

    def wrap_to_pi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def correct_for_distance(self, original_vel, distance):
        factor = self.sigmoid(distance)
        if factor < 0.05:
            factor = 0
        original_vel.linear.x *= factor
        original_vel.angular.z *= factor

    def correct_for_angle(self, original_vel, angle, robot_heading):
        pass

    # for augmenting cmd_vel according to the actor-robot relation
    def cmd_vel_callback(self, cmd_vel):
        dist_vector = np.array([self.robot_loc_x - self.actor_loc_x, self.robot_loc_y - self.actor_loc_y])
        # distance correction
        distance = np.linalg.norm(dist_vector)
        self.correct_for_distance(cmd_vel, distance)
        # angle correction
        angle = np.arctan2(dist_vector[1], dist_vector[0])
        wrapped_angle = self.wrap_to_pi(np.abs(robot_angle - angle))
        self.correct_for_angle(cmd_vel, wrapped_angle, robot_angle)

        self.cmd_vel_pub.publish(cmd_vel)

    # for updating the actor position
    def obstacle_callback(self, obstacles):
        tracked = False
        for circle in obstacles.circles:
            x, y = circle.center.x, circle.center.y
            if np.abs(x - self.actor_loc_x) < self.ACTOR_TOL and \
               np.abs(y - self.actor_loc_y) < self.ACTOR_TOL:
                self.actor_loc_x, self.actor_loc_y = x, y
                self.actor_vel_x, self.actor_vel_y = circle.velocity.x, circle.velocity.y
                tracked = True
        if tracked:
            self.guide_status = 1
        else:
            self.guide_status = 0

    # for updating the robot position
    def odom_callback(self, odom):
        trans = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time())

        self.robot_loc_x = trans.transform.translation.x
        self.robot_loc_y = trans.transform.translation.y

        quaternion = (trans.transform.rotation.x, trans.transform.rotation.y,
                      trans.transform.rotation.z, trans.transform.rotation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.robot_angle = euler[2] # yaw angle used for robot heading

    def get_robot(self):
        try:
            odom = rospy.wait_for_message('odom', Odometry, timeout=2)
            # use the odom_callback function for updating the globals
            self.odom_callback(odom)
        except rospy.ROSException:
            print ("[GUIDE_CONTROL]: Robot pose couldn't be retrieved in time.")

    # actor location inialization; uses closest actor behind the robot
    def get_actor(self):
        try:
            best_candidate = None
            min_distance = 10000
            obstacles = rospy.wait_for_message('obstacles', Obstacles, timeout=2)
            for circle in obstacles.circles:
                dist_vector = np.array([self.robot_loc_x - circle.center.x, self.robot_loc_y - circle.center.y])
                angle = np.arctan2(dist_vector[1], dist_vector[0])
                wrapped_angle = self.wrap_to_pi(np.abs(self.robot_angle - angle))
                if np.abs(wrapped_angle) < 1:
                    distance = np.linalg.norm(dist_vector)
                    if distance < min_distance:
                        best_candidate = (circle.center.x, circle.center.y)
                        min_distance = distance

            if best_candidate == None:
                print ("[GUIDE_CONTROL]: No suitable actor found! Trying again...")
                return False
            else:
                self.actor_loc_x = best_candidate[0]
                self.actor_loc_y = best_candidate[1]
                self.guide_status = 1 # tracking
                print ("[GUIDE_CONTROL]: Actor at location (%3f,%3f) chosen." % (self.actor_loc_x, self.actor_loc_y))
                return True

        except rospy.ROSException:
            print ("[GUIDE_CONTROL]: Message couldn't be recieved in time. Check /obstacles topic.")
            return False

if __name__ == '__main__':
    try:
        controller = GuideController()
        mainloop_rate = rospy.Rate(1)
        while(1):
            if controller.guide_status == 1:
                print("[GUIDE_CONTROL]: Actor tracked at (%f %f)" % (controller.actor_loc_x, controller.actor_loc_y))
            elif controller.guide_status == 0:
                controller.get_actor()
            mainloop_rate.sleep()

    except rospy.ROSInterruptException:
        pass
