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
//truth expectation needed for executions
#define DECISION_THRESHOLD 0.501
#define ANTICIPATION_THRESHOLD 0.52
#define ANTICIPATION_CONFIDENCE 0.01
//motor babbling chance
#define MOTOR_BABBLING_CHANCE_INITIAL 0.2
extern double MOTOR_BABBLING_CHANCE;

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
bool Decision_Making(Event *goal, long currentTime);
//assumption of failure, also works for "do nothing operator"
void Decision_AssumptionOfFailure(int operationID, long currentTime);

#endif
