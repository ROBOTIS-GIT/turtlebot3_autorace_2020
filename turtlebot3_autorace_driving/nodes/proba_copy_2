#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from control_lane_2 import ControlLane
from turtlebot3_teleop_key_2 import teleop
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg1 = """
Control Your TurtleBot3!
---------------------------
Select control mode:

Teleop mode [t]
Autonomous mode [y]
"""
msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

q : select new control mode

CTRL-C to quit
"""

e = """
Communications Failed
"""

while not rospy.is_shutdown():
    mode=input(msg1)
    if mode == 't':
        print('you select the teleop mode')
        
        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)

        rospy.init_node('turtlebot3_teleop_key_2')
        node = teleop()
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        turtlebot3_model = rospy.get_param("model", "burger")

        status = 0
        target_linear_vel   = 0.0
        target_angular_vel  = 0.0
        control_linear_vel  = 0.0
        control_angular_vel = 0.0

        try:
            print(msg)
            while not rospy.is_shutdown():
                key = node.getKey(settings)
                if key == 'w' :
                    target_linear_vel = node.checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                    status = status + 1
                    print(node.vels(target_linear_vel,target_angular_vel))
                elif key == 'x' :
                    target_linear_vel = node.checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                    status = status + 1
                    print(node.vels(target_linear_vel,target_angular_vel))
                elif key == 'a' :
                    target_angular_vel = node.checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(node.vels(target_linear_vel,target_angular_vel))
                elif key == 'd' :
                    target_angular_vel = node.checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(node.vels(target_linear_vel,target_angular_vel))
                elif key == ' ' or key == 's' :
                    target_linear_vel   = 0.0
                    control_linear_vel  = 0.0
                    target_angular_vel  = 0.0
                    control_angular_vel = 0.0
                    print(node.vels(target_linear_vel, target_angular_vel))
                elif key == 'q' :
                    target_linear_vel   = 0.0
                    control_linear_vel  = 0.0
                    target_angular_vel  = 0.0
                    control_angular_vel = 0.0
                    print(node.vels(target_linear_vel, target_angular_vel))
                    break
                else:
                    if (key == '\x03'):
                        break

                if status == 20 :
                    print(msg)
                    status = 0

                twist = Twist()

                control_linear_vel = node.makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

                control_angular_vel = node.makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

                pub.publish(twist)

        except:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
            pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    elif mode == 'y':
        print('you select the autonomous mode')
        rospy.init_node('control_lane_2')
        node = ControlLane()
        node.main()
        
    else:
        print('that mode is not correct')

