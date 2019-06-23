#!/usr/bin/env python
import os, sys
import select
import termios
import tty
import rospy

from geometry_msgs.msg import Vector3

disp = """
Actor controller
---------------------------
Moving around:
        w
   a         d
        s

w/s - control movement along y axis (increments of 0.1 [m/s])
a/d - control movement along x axis (increments of 0.1 [m/s])

space key, x : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""
VELOCITY_STEP = 0.1
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
    global disp, e, VELOCITY_STEP

    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('actor_controller')
    pub = rospy.Publisher('/' + model_name + '_cmd_vel', Vector3, queue_size=10)

    status = 0
    actor_x_vel = 0
    actor_y_vel = 0

    try:
        print(disp)
        while(1):
            key = getKey(settings)
            if key == 'w' :
                actor_y_vel = constrain_speed(actor_y_vel + VELOCITY_STEP)
                status = status + 1
                print("X velocity: %f || Y velocity: %f" % (actor_x_vel, actor_y_vel))
            elif key == 's' :
                actor_y_vel = constrain_speed(actor_y_vel - VELOCITY_STEP)
                status = status + 1
                print("X velocity: %f || Y velocity: %f" % (actor_x_vel, actor_y_vel))
            elif key == 'a' :
                actor_x_vel = constrain_speed(actor_x_vel - VELOCITY_STEP)
                status = status + 1
                print("X velocity: %f || Y velocity: %f" % (actor_x_vel, actor_y_vel))
            elif key == 'd' :
                actor_x_vel = constrain_speed(actor_x_vel + VELOCITY_STEP)
                status = status + 1
                print("X velocity: %f || Y velocity: %f" % (actor_x_vel, actor_y_vel))
            elif key == ' ' or key == 'x' :
                actor_x_vel = 0
                actor_y_vel = 0
                print("Actor stopped.")
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print(disp)
                status = 0

            msg = Vector3()
            msg.x = actor_x_vel
            msg.y = actor_y_vel
            msg.z = 0

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
