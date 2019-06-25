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

    def __init__(self):
        # initialize the node and set control rate
        rospy.init_node('guide_controller')
        self.mainloop_rate = rospy.Rate(1)

        self.GUIDE_DISTANCE = rospy.get_param("~guide_distance", 2.0)  # how far the robot will try to stay ahead of the actor
        self.ACTOR_TOL = rospy.get_param("~actor_tolerance", 0.2)  # tolerance of actor movement in a timestep
        self.SIGMOID_FACTOR = np.log(99)/self.GUIDE_DISTANCE # factor for shaping the sigmoid

        self.tf_buffer = tf2_ros.Buffer()   # transform buffer
        self.map_frame = rospy.get_param("~map_frame", "map")  # transform frame of the map (static)
        self.robot_frame = rospy.get_param("~robot_frame", "base_link")  # transform frame of the robot (dynamic)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=2)  # command velocity publisher
        self.actor_pub = rospy.Publisher('tracked_actor', CircleObstacle, queue_size=2) # actor location publisher
        # last known actor location and velocity
        self.actor = CircleObstacle()
        # last known robot location and heading
        self.robot_loc_x = 0
        self.robot_loc_y = 0
        self.robot_yaw = 0 # [-pi, pi]
        # status of the guide controller:
        # 0 - no actor available for guiding, finding actor
        # 1 - locked on actor, guiding
        self.guide_status = 0

        # setup transform buffer and wait for it to fill up
        listener = tf2_ros.TransformListener(self.tf_buffer)
        self.mainloop_rate.sleep()

        # determine initial robot pose
        self.get_robot()

        # subscriber initialization
        rospy.Subscriber('cmd_vel_dwb', Twist, self.cmd_vel_callback, queue_size=1)
        rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('obstacles', Obstacles, self.obstacle_callback, queue_size=1)
        rospy.loginfo("GuideController: Initialization successful.")

    def sigmoid(self, x):
        '''Sigmoid defined around keeping the actor at the GUIDE_DISTANCE'''
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
        dist_vector = np.array([self.robot_loc_x - self.actor.center.x, self.robot_loc_y - self.actor.center.y])
        # distance correction
        distance = np.linalg.norm(dist_vector)
        self.correct_for_distance(cmd_vel, distance)
        # angle correction
        angle = np.arctan2(dist_vector[1], dist_vector[0])
        wrapped_angle = self.wrap_to_pi(np.abs(robot_yaw - angle))
        self.correct_for_angle(cmd_vel, wrapped_angle, robot_yaw)

        self.cmd_vel_pub.publish(cmd_vel)

    # for updating the actor position
    def obstacle_callback(self, obstacles):
        tracked = False
        for circle in obstacles.circles:
            x, y = circle.center.x, circle.center.y
            if np.abs(x - self.actor.center.x) < self.ACTOR_TOL and \
               np.abs(y - self.actor.center.y) < self.ACTOR_TOL:
                self.actor.center.x, self.actor.center.y = x, y
                self.actor.velocity.x, self.actor.velocity.y = circle.velocity.x, circle.velocity.y
                tracked = True
        if tracked:
            self.guide_status = 1
        else:
            self.guide_status = 0

    # for updating the robot position
    def odom_callback(self, odom):
        trans = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rospy.Time())

        self.robot_loc_x = trans.transform.translation.x
        self.robot_loc_y = trans.transform.translation.y

        quaternion = (trans.transform.rotation.x, trans.transform.rotation.y,
                      trans.transform.rotation.z, trans.transform.rotation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.robot_yaw = euler[2] # yaw angle used for robot heading

    def get_robot(self):
        try:
            odom = rospy.wait_for_message('odom', Odometry, timeout=2)
            # use the odom_callback function for updating the globals
            self.odom_callback(odom)
        except rospy.ROSException:
            rospy.logerr("GuideController: Robot pose couldn't be retrieved in time. Check /tf topic.")
            raise RuntimeError("GuideController failed initialization.")

    # actor location inialization; uses closest actor behind the robot
    def get_actor(self):
        try:
            best_candidate = None
            min_distance = 10000
            obstacles = rospy.wait_for_message('obstacles', Obstacles, timeout=2)
            for circle in obstacles.circles:
                dist_vector = np.array([self.robot_loc_x - circle.center.x, self.robot_loc_y - circle.center.y])
                angle = np.arctan2(dist_vector[1], dist_vector[0])
                wrapped_angle = self.wrap_to_pi(np.abs(self.robot_yaw - angle))
                if np.abs(wrapped_angle) < 1:
                    distance = np.linalg.norm(dist_vector)
                    if distance < min_distance:
                        best_candidate = (circle.center.x, circle.center.y)
                        min_distance = distance

            if best_candidate == None:
                rospy.loginfo("GuideController: No suitable actor found! Trying again...")
            else:
                self.actor.center.x = best_candidate[0]
                self.actor.center.y = best_candidate[1]
                self.guide_status = 1 # tracking
                rospy.loginfo("GuideController: Actor at location (%1.2f, %1.2f) chosen." % (self.actor.center.x, self.actor.center.y))

        except rospy.ROSException:
            rospy.loginfo("GuideController: Message couldn't be recieved in time. Check /obstacles topic.")

    def run(self):
        while(1):
            if self.guide_status == 1:
                rospy.loginfo("GuideController: Tracking actor at location (%1.2f, %1.2f)." % (self.actor.center.x, self.actor.center.y))
                self.actor_pub.publish(self.actor) # publish actor info
            elif self.guide_status == 0:
                rospy.loginfo("GuideController: No actor is being tracked, searching for new actor behind robot.")
                self.get_actor()
            self.mainloop_rate.sleep()

if __name__ == '__main__':
    try:
        controller = GuideController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
