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

# Author: Will Son, Ashe Kim

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from dynamic_reconfigure.server import Server

class ImagePublish():
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_topic",CompressedImage,self.callback, queue_size = 1)

    def callback(self,data):
        try:
            cv_image = self.bridge.cv2_to_compressed_imgmsg(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('sub_image')
    node = ImagePublish()
    node.main()
