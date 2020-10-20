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
import rospy
from std_msgs.msg import String, Float32
from ona_experiment1.msg import Statements
from geometry_msgs.msg import PoseStamped
from ona_roslib import *

def detectionToNarseseStatementsList(position, orientation):
    narseseStatementsList = []
    (x, y) = (position[0], position[1])
    (xLabel, yLabel) = foveaRelativePosition(x, y)
    doorstate = "closed"
    if orientation[2] > 80: #larger than 80 degree z orientation (TODO see if it's the relevant one, potentially convert!
        doorstate = "open"
    narseseStatementsList.append(object_narsese(doorstate, "door", xLabel, yLabel))
    return narseseStatementsList

class TrackerUtility:

    def __init__(self):
        rospy.init_node('TrackerUtility', anonymous=False)
        rospy.Subscriber("/visp_auto_tracker/object_position", PoseStamped, self.tracker_callback)
        self.narsese_pub = rospy.Publisher('/ona_ros/nars/narsese', String, queue_size=1)
        self.rate = rospy.Rate(10)

    def tracker_callback(self, data):
        pose = data.pose
        position = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        orientation = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        for stmt in detectionToNarseseStatementsList(position, orientation):
            self.narsese_pub.publish(stmt)

if __name__ == '__main__':
    try:
        utility = TrackerUtility()
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass
