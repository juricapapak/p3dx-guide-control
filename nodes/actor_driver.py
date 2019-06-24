#!/usr/bin/env python

import os, sys
import rospy

from geometry_msgs.msg import Vector3
from gazebo_msgs.msg import ModelState

VELOCITY_X = 0.0
VELOCITY_Y = 0.0

RATE = 25.0 # frequency of 'driver'

def cmd_vel_sub(data):
    global VELOCITY_X, VELOCITY_Y
    VELOCITY_X = data.x
    VELOCITY_Y = data.y

def main(model_name, start_x, start_y):
    global VELOCITY_X, VELOCITY_Y, RATE

    print("[ACTOR DRIVER] Initalizing driver for %s" % (model_name))
    rospy.init_node('actor_driver')
    driver_rate = rospy.Rate(RATE)

    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rospy.Subscriber('/' + model_name + '_cmd_vel', Vector3, cmd_vel_sub, queue_size=1)

    pos_x = start_x
    pos_y = start_y

    while(1):
        pos_x += VELOCITY_X * (float(1)/RATE)
        pos_y += VELOCITY_Y * (float(1)/RATE)

        msg = ModelState()
        msg.model_name = model_name
        msg.pose.position.x = pos_x
        msg.pose.position.y = pos_y
        msg.pose.position.z = 0

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
