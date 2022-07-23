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

#include "ona.hpp"
ros::Publisher publisher;

void publish(const char *name, Term args){
    std_msgs::String msg;
    stringstream ss;
    ss << name << " ";
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        Atom atom = args.atoms[i];
        if(atom != 0)
        {
            ss << Narsese_atomNames[atom-1] << " ";
        }
    }
    msg.data = ss.str();
    ROS_INFO("ROSNAR executed %s", msg.data.c_str());
    publisher.publish(msg);
}

void narseseInputCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("ROSNAR input: %s", msg->data.c_str());
    NAR_AddInputNarsese((char*) msg->data.c_str());
}

static Feedback ROSNAR_op1(Term args){
    publish("^op1", args);
    return (Feedback) {0};
}

static Feedback ROSNAR_op2(Term args){
    publish("^op2", args);
    return (Feedback) {0};
}

static Feedback ROSNAR_op3(Term args){
    publish("^op3", args);
    return (Feedback) {0};
}

static Feedback ROSNAR_op4(Term args){
    publish("^op4", args);
    return (Feedback) {0};
}

static Feedback ROSNAR_op5(Term args){
    publish("^op5", args);
    return (Feedback) {0};
}

static Feedback ROSNAR_op6(Term args){
    publish("^op6", args);
    return (Feedback) {0};
}

static Feedback ROSNAR_op7(Term args){
    publish("^op7", args);
    return (Feedback) {0};
}

static Feedback ROSNAR_op8(Term args){
    publish("^op8", args);
    return (Feedback) {0};
}

static Feedback ROSNAR_op9(Term args){
    publish("^op9", args);
    return (Feedback) {0};
}

static Feedback ROSNAR_op10(Term args){
    publish("^op10", args);
    return (Feedback) {0};
}

void ROSNAR_INIT(){
    NAR_INIT();
    PRINT_DERIVATIONS = false;
    int k=0; if(k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation((char*)"^op1", ROSNAR_op1); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation((char*)"^op2", ROSNAR_op2); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation((char*)"^op3", ROSNAR_op3); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation((char*)"^op4", ROSNAR_op4); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation((char*)"^op5", ROSNAR_op5); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation((char*)"^op6", ROSNAR_op6); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation((char*)"^op7", ROSNAR_op7); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation((char*)"^op8", ROSNAR_op8); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation((char*)"^op9", ROSNAR_op9); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation((char*)"^op10", ROSNAR_op10); if(++k >= OPERATIONS_MAX) { return; };
    std::cout<<"Shell_NARInit: Ran out of operators, add more there, or decrease OPERATIONS_MAX!"<<std::endl;
    exit(1);
}
