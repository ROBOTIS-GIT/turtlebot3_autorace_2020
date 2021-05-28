#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Leon Jung, Gilbert, Ashe Kim
 
import rospy
import numpy as np
import math
import tf
from enum import Enum
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlebot3_autorace_msgs.msg import MovingParam

class ControlMoving():
    def __init__(self):
        self.sub_moving_state = rospy.Subscriber('/control/moving/state', MovingParam, self.get_param, queue_size = 1)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size=1)

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.pub_max_vel = rospy.Publisher('/control/max_vel', Float64, queue_size = 1)
        self.pub_moving_complete = rospy.Publisher('/control/moving/complete', UInt8, queue_size = 1)

        #moving type enum
        self.TypeOfMoving = Enum('TypeOfMoving', 'idle left right forward backward')
        self.TypeOfState = Enum('TypeOfState', 'idle start stop finish')
        
        self.current_pos_x = 0.0
        self.current_pos_y = 0.0
        
        #moving params
        self.moving_type = UInt8()
        self.moving_angluar = 0.0
        self.moving_linear = 0.0
        self.moving_msg = MovingParam()

        #moving flags
        self.is_step_start = False
        self.is_step_left = False
        self.is_step_right = False
        self.is_step_forward = False
        self.is_step_backward = False

        #moving internal valiables
        self.theta = 0.0
        self.current_theta = 0.0
        self.last_current_theta = 0.0
        self.lastError = 0.0

        self.test = math.radians(90)
     
        msg_pub_max_vel = Float64()
        msg_pub_max_vel.data = 0.05
        self.pub_max_vel.publish(msg_pub_max_vel)

        loop_rate = rospy.Rate(100) # 10hz
        while not rospy.is_shutdown():
            if self.is_step_left == True:
                self.turn_left(self.moving_msg)
            elif self.is_step_right == True:
                self.turn_right(self.moving_msg)
            elif self.is_step_forward == True:
                self.go_forward(self.moving_msg)
            elif self.is_step_backward == True:
                self.go_backward(self.moving_msg)
            else:
                pass
            loop_rate.sleep()
        rospy.on_shutdown(self.fnShutDown)


    def get_param(self, msg):
        self.moving_type = msg.moving_type
        self.moving_angluar = msg.moving_value_angular
        self.moving_linear = msg.moving_value_linear
        rospy.loginfo("get : moving_type : %d",self.moving_type)
        rospy.loginfo("get : moving_angluar : %f",self.moving_angluar)
        rospy.loginfo("get : moving_linear : %f", self.moving_linear)

        if self.is_step_left == True:
            rospy.loginfo("now left step stil working")
        elif self.is_step_right == True:
            rospy.loginfo("now right step stil working")
        elif self.is_step_forward == True:
            rospy.loginfo("now forward step stil working")
        elif self.is_step_backward == True:
            rospy.loginfo("now backward step stil working")

        if self.is_step_start == False:
            if msg.moving_type == self.TypeOfMoving.left.value:
                self.moving_msg = msg
                self.is_step_left = True
            elif msg.moving_type == self.TypeOfMoving.right.value:
                self.moving_msg = msg
                self.is_step_right = True
            elif msg.moving_type == self.TypeOfMoving.forward.value:
                self.moving_msg = msg
                self.is_step_forward = True
            elif msg.moving_type == self.TypeOfMoving.backward.value:
                self.moving_msg = msg
                self.is_step_backward = True
        

    def init_state(self):
        self.is_step_start = False
        self.is_step_left = False
        self.is_step_right = False
        self.is_step_forward = False
        self.is_step_backward = False


    def step_completed(self):
        self.is_step_start = False
        self.is_step_left = False
        self.is_step_right = False
        self.is_step_forward = False
        self.is_step_backward = False

        self.current_pos_x = 0.0
        self.current_pos_y = 0.0
        self.theta = 0.0
        self.current_theta = 0.0
        self.last_current_theta = 0.0
        self.lastError = 0.0
        
        self.fnStop()

        state = UInt8()
        state = self.TypeOfState.finish.value
        self.pub_moving_complete.publish(state)


    def turn_left(self, msg):
        rospy.loginfo("turn_left function is called")
        if self.is_step_start == False:
            self.lastError = 0.0
            self.desired_theta = self.current_theta + math.radians(msg.moving_value_angular)
            self.is_step_start = True
        
        error = self.fnTurn()

        if math.fabs(error) < 0.05:
            rospy.loginfo("turn_left function is finished")
            self.step_completed()

    def turn_right(self, msg):
        rospy.loginfo("turn_right function is called")
        if self.is_step_start == False:
            self.lastError = 0.0
            self.desired_theta = self.current_theta - math.radians(msg.moving_value_angular)
            self.is_step_start = True

        error = self.fnTurn()

        if math.fabs(error) < 0.05:
            rospy.loginfo("turn_right function is finished")
            self.step_completed()


    def go_forward(self, msg):
        rospy.loginfo("go_forawrd function is called")
        if self.is_step_start == False:
            self.lastErorr = 0.0
            self.start_pos_x = self.current_pos_x
            self.start_pos_y = self.current_pos_y
            self.is_step_start = True

        error = self.fnStraight(msg.moving_value_linear)

        if math.fabs(error) < 0.005:
            rospy.loginfo("go_forward function is finished")
            self.step_completed()


    def go_backward(self, msg):
        rospy.loginfo("go_backward function is called")
        if self.is_step_start == False:
            self.lastError = 0.0
            self.start_pos_x = self.current_pos_x
            self.start_pos_y = self.current_pos_y
            self.is_step_start = True
        
        error = self.fnBackStraight(msg.moving_value_linear)

        if math.fabs(error) < 0.005:
            rospy.loginfo("go_backward function is finished")
            self.step_completed()


    def fnTurn(self):
        err_theta = self.current_theta - self.desired_theta
        
        # rospy.loginfo("err_theta  desired_theta  current_theta : %f  %f  %f", err_theta, self.desired_theta, self.current_theta)
        Kp = 0.45
        Kd = 0.03

        angular_z = Kp * err_theta + Kd * (err_theta - self.lastError)
        self.lastError = err_theta

        twist = Twist()
        twist.linear.x = 0 #0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -(angular_z * 2)
        self.pub_cmd_vel.publish(twist)

        # rospy.loginfo("angular_z : %f", angular_z)

        return err_theta

    def fnStraight(self, desired_dist):   

        err_pos = math.sqrt((self.current_pos_x - self.start_pos_x) ** 2 + (self.current_pos_y - self.start_pos_y) ** 2) - desired_dist
    
        # rospy.loginfo("error_pos2 = %f", err_pos)
        Kp = 0.04
        Kd = 0.05

        angular_z = Kp * err_pos + Kd * (err_pos - self.lastError)
        self.lastError = err_pos

        twist = Twist()

        if err_pos < 0:
            twist.linear.x = 0.1 #0.07
        else:
            twist.linear.x = -0.1

        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

        return err_pos


    def fnBackStraight(self, desired_dist):   

        err_pos = math.sqrt((self.current_pos_x - self.start_pos_x) ** 2 + (self.current_pos_y - self.start_pos_y) ** 2) - desired_dist
    
        # rospy.loginfo("error_pos = %f", err_pos)
        Kp = 0.04
        Kd = 0.05

        angular_z = Kp * err_pos + Kd * (err_pos - self.lastError)
        self.lastError = err_pos

        twist = Twist()
      
        if err_pos < 0:
            twist.linear.x = -0.1 #0.07
        else:
            twist.linear.x = 0.1

        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

        return err_pos

    def fnStop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

    
    def cbOdom(self, odom_msg):
        quaternion = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
        self.current_theta = self.euler_from_quaternion(quaternion)

        if (self.current_theta - self.last_current_theta) < -math.pi:
            self.current_theta = 2. * math.pi + self.current_theta
            self.last_current_theta = math.pi
        elif (self.current_theta - self.last_current_theta) > math.pi:
            self.current_theta = -2. * math.pi + self.current_theta
            self.last_current_theta = -math.pi
        else:
            self.last_current_theta = self.current_theta

        self.current_pos_x = odom_msg.pose.pose.position.x
        self.current_pos_y = odom_msg.pose.pose.position.y

    def euler_from_quaternion(self, quaternion):
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        return theta

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist) 

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('control_moving')
    node = ControlMoving()
    node.main()
