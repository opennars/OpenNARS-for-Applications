#ifndef __ONA_H__
#define __ONA_H__

#include <iostream>
#include <ros/ros.h>
#include <cstdint>
#include <std_msgs/String.h>

extern "C" {
    #include <ona/NAR.h>
}

using namespace std;

extern ros::Publisher publisher;

void ROSNAR_INIT();

void publish(const char* name, Term args);

void narseseInputCallback(const std_msgs::String::ConstPtr& msg);

static void ROSNAR_op_left(Term args);
static void ROSNAR_op_right(Term args);
static void ROSNAR_op_up(Term args);
static void ROSNAR_op_down(Term args);
static void ROSNAR_op_say(Term args);
static void ROSNAR_op_pick(Term args);
static void ROSNAR_op_drop(Term args);
static void ROSNAR_op_go(Term args);
static void ROSNAR_op_activate(Term args);
static void ROSNAR_op_deactivate(Term args);

#endif