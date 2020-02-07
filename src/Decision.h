#ifndef DECISION_H
#define DECISION_H

//References//
//----------//
#include <stdbool.h>
#include <stdio.h>
#include "Memory.h"
#include "YAN.h"
#include "Config.h"

////////////////////
//  YAN Decision  //
////////////////////
//Realization of goals

//Parameters//
//----------//
extern double DECISION_THRESHOLD;
extern double ANTICIPATION_THRESHOLD;
extern double ANTICIPATION_CONFIDENCE;
extern double MOTOR_BABBLING_CHANCE;

//Data structure//
//--------------//
typedef struct
{
    double desire;
    bool execute;
    int operationID;
    Operation op;
    Term arguments;
}Decision;

//Methods//
//-------//
//execute decision
void Decision_Execute(Decision *decision);
//assumption of failure, also works for "do nothing operator"
void Decision_AssumptionOfFailure(int operationID, long currentTime);
//YAN decision making rule applying when goal is an operation
Decision Decision_Suggest(Event *goal, long currentTime);

#endif
