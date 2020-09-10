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