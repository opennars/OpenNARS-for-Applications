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

static Feedback ROSNAR_op_left(Term args);
static Feedback ROSNAR_op_right(Term args);
static Feedback ROSNAR_op_up(Term args);
static Feedback ROSNAR_op_down(Term args);
static Feedback ROSNAR_op_say(Term args);
static Feedback ROSNAR_op_pick(Term args);
static Feedback ROSNAR_op_drop(Term args);
static Feedback ROSNAR_op_go(Term args);
static Feedback ROSNAR_op_activate(Term args);
static Feedback ROSNAR_op_deactivate(Term args);

#endif
