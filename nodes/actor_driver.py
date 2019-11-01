#!/usr/bin/env python

import os, sys
import rospy
import tf
import numpy as np

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState

VELOCITY_ANG = 0.0
VELOCITY_LIN = 0.0

RATE = 20.0 # driver frequency

def cmd_vel_sub(data):
    global VELOCITY_ANG, VELOCITY_LIN
    VELOCITY_ANG = data.angular.z
    VELOCITY_LIN = data.linear.y

def main(model_name, start_x, start_y):
    global VELOCITY_ANG, VELOCITY_LIN, RATE

    print("[ACTOR DRIVER] Initalizing driver for %s" % (model_name))
    rospy.init_node('actor_driver')
    driver_rate = rospy.Rate(RATE)

    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rospy.Subscriber('/' + model_name + '_cmd_vel', Twist, cmd_vel_sub, queue_size=1)

    # initial pose and orientation
    pos_x = start_x
    pos_y = start_y
    yaw = 0

    while(1):
        # calculate new yaw angle
        yaw += VELOCITY_ANG * (float(1)/RATE)
        # increment the x and y positions based on current heading
        pos_x += np.cos(yaw) * VELOCITY_LIN * (float(1)/RATE)
        pos_y += np.sin(yaw) * VELOCITY_LIN * (float(1)/RATE)

        # calculate the quaternion required for the message and set the message
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        msg = ModelState()
        msg.model_name = model_name
        msg.pose.position.x = pos_x
        msg.pose.position.y = pos_y
        msg.pose.position.z = 1.05
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]

        pub.publish(msg)
        driver_rate.sleep()

if __name__ == '__main__':
    try:
        model_name = sys.argv[1]
        start_x = float(sys.argv[2])
        start_y = float(sys.argv[3])
        main(model_name, start_x, start_y)
    except rospy.ROSInterruptException:
        pass
