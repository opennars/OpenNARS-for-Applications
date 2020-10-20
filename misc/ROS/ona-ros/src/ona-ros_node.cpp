/* 
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
 */

#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "ona.hpp"

using namespace std;

extern ros::Publisher publisher;

int main(int argc, char **argv){
    ROSNAR_INIT();
    ros::init(argc, argv, "ROSNAR");
    ros::NodeHandle n;
 
    ros::Subscriber NarseseInputSubscriber = n.subscribe("/ona_ros/nars/narsese", 1000, narseseInputCallback);
    publisher = n.advertise<std_msgs::String>("/ona_ros/nars/execute", 1000);

    ros::Rate loop_rate(10); //Inference speed in ticks per second
    while (ros::ok()){
        NAR_Cycles(1);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
