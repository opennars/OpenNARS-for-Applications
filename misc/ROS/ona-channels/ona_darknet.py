"""
 * The MIT License
 *
 * Copyright 2020 The OpenNARS authors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * """

import cv2
import math
import rospy
import time
import string 
from std_msgs.msg import String, Float32
from ona_experiment1.msg import Statements
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from ona_roslib import *

def detectionToNarseseStatementsList(detection):
    narseseStatementsList = []
    (x, y) = detectionPosition(detection["boxes"][0], detection["boxes"][1], detection["boxes"][2], detection["boxes"][3])
    (xLabel, yLabel) = foveaRelativePosition(x, y)
    sz = discretizedSize(detection["boxes"][2], detection["boxes"][3])
    narseseStatementsList.append(object_narsese(sz, detection["label"], xLabel, yLabel, detection["confidence"]))
    return narseseStatementsList

class NARSUtility:

    def __init__(self):
        rospy.init_node('NARSUtility', anonymous=False)
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.darknet_callback)
        self.narsese_pub = rospy.Publisher('/ona_ros/nars/narsese', String, queue_size=1)
        self.rate = rospy.Rate(10)

    def darknet_callback(self, data):
        bboxes = data.bounding_boxes
        for bbox in bboxes:
            bbox = {'label': bbox.Class, 'confidence': bbox.probability, 'boxes': [bbox.xmin, bbox.ymin, bbox.xmax - bbox.xmin, bbox.ymax - bbox.ymin]}
            for stmt in detectionToNarseseStatementsList(bbox):
                self.narsese_pub.publish(stmt)

if __name__ == '__main__':
    try:
        utility = NARSUtility()
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass
