#!/usr/bin/env python
import os, sys
import select
import termios
import tty
import rospy

from geometry_msgs.msg import Twist

disp = """
Actor controller
---------------------------
Moving around:
        w
   a         d
        s

w/s - control movement forward/backward (increments of 0.1 [m/s])
a/d - control turning left/right (+-1 [rad/s] on press)

space key, x : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""
LIN_VELOCITY_STEP = 0.1
ANG_VELOCITY_STEP = 1
ACTOR_VEL_CAP = 1

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def constrain_speed(speed):
    global ACTOR_VEL_CAP
    if speed > ACTOR_VEL_CAP:
        return ACTOR_VEL_CAP
    elif speed < -ACTOR_VEL_CAP:
        return -ACTOR_VEL_CAP
    else:
        return speed

def main(model_name):
    global disp, e, LIN_VELOCITY_STEP, ANG_VELOCITY_STEP

    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('actor_controller')
    pub = rospy.Publisher('/' + model_name + '_cmd_vel', Twist, queue_size=10)

    status = 0
    actor_ang_vel = 0
    actor_lin_vel = 0

    try:
        print(disp)
        while(1):
            key = getKey(settings)
            # forward/backward
            if key == 'w' :
                actor_lin_vel = constrain_speed(actor_lin_vel + LIN_VELOCITY_STEP)
                status = status + 1
                print("Linear velocity: %f" % (actor_lin_vel))
            elif key == 's' :
                actor_lin_vel = constrain_speed(actor_lin_vel - LIN_VELOCITY_STEP)
                status = status + 1
                print("Linear velocity: %f" % (actor_lin_vel))
            # turning left/right
            elif key == 'a' :
                actor_ang_vel = ANG_VELOCITY_STEP
            elif key == 'd' :
                actor_ang_vel = -ANG_VELOCITY_STEP
            elif key == ' ' or key == 'x' :
                actor_ang_vel = 0
                actor_lin_vel = 0
                print("Actor stopped.")
            else:
                actor_ang_vel = 0
                if (key == '\x03'):
                    break

            if status == 20 :
                print(disp)
                status = 0

            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = actor_lin_vel
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = actor_ang_vel


            pub.publish(msg)

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    try:
        model_name = sys.argv[1]
        main(model_name)
    except rospy.ROSInterruptException:
        pass
