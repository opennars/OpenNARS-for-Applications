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

static void ROSNAR_op_left(Term args){
    publish("^left", args);
}

static void ROSNAR_op_right(Term args){
    publish("^right", args);
}

static void ROSNAR_op_up(Term args){
    publish("^up", args);
}

static void ROSNAR_op_down(Term args){
    publish("^down", args);
}

static void ROSNAR_op_say(Term args){
    publish("^say", args);
}

static void ROSNAR_op_pick(Term args){
    publish("^pick", args);
}

static void ROSNAR_op_drop(Term args){
    publish("^drop", args);
}

static void ROSNAR_op_go(Term args){
    publish("^go", args);
}

static void ROSNAR_op_activate(Term args){
    publish("^activate", args);
}

static void ROSNAR_op_deactivate(Term args){
    publish("^deactivate", args);
}

void ROSNAR_INIT(){
    NAR_INIT();
    PRINT_DERIVATIONS = false;
    int k=0; if(k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm((char*)"^left"), ROSNAR_op_left); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm((char*)"^right"), ROSNAR_op_right); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm((char*)"^up"), ROSNAR_op_up); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm((char*)"^down"), ROSNAR_op_down); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm((char*)"^say"), ROSNAR_op_say); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm((char*)"^pick"), ROSNAR_op_pick); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm((char*)"^drop"), ROSNAR_op_drop); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm((char*)"^go"), ROSNAR_op_go); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm((char*)"^activate"), ROSNAR_op_activate); if(++k >= OPERATIONS_MAX) { return; };
    NAR_AddOperation(Narsese_AtomicTerm((char*)"^deactivate"), ROSNAR_op_deactivate); if(++k >= OPERATIONS_MAX) { return; };
    std::cout<<"Shell_NARInit: Ran out of operators, add more there, or decrease OPERATIONS_MAX!"<<std::endl;
    exit(1);
}