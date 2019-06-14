#ifndef DECISION_H
#define DECISION_H

//References//
//----------//
#include <stdbool.h>
#include <stdio.h>
#include "Memory.h"
#include "ANSNA.h"

//Parameters//
//----------//
//confidence needed for executions
#define DECISION_THRESHOLD 0.8
//motor babbling chance
#define MOTOR_BABBLING_CHANCE 0.1

//Data structure//
//--------------//
typedef struct
{
    bool execute;
    int operationID;
    Operation op;
}Decision;

//Methods//
//-------//
//ANSNA decision making rule applying when goal is an operation
void Decision_Making(Event *goal, long currentTime);
Decision MotorBabbling();
#endif
