#ifndef DECISION_H
#define DECISION_H

//References//
//----------//
#include <stdbool.h>
#include <stdio.h>
#include "Memory.h"
#include "MSC.h"

////////////////////
//  MSC Decision  //
////////////////////
//Realization of goals

//Parameters//
//----------//
//truth expectation needed for executions
#define DECISION_THRESHOLD_INITIAL 0.501
extern double DECISION_THRESHOLD;
#define ANTICIPATION_THRESHOLD_INITIAL 0.501
extern double ANTICIPATION_THRESHOLD;
#define ANTICIPATION_CONFIDENCE_INITIAL 0.005
extern double ANTICIPATION_CONFIDENCE;
//motor babbling chance
#define MOTOR_BABBLING_CHANCE_INITIAL 0.2
extern double MOTOR_BABBLING_CHANCE;

//Data structure//
//--------------//
typedef struct
{
    double desire;
    bool execute;
    int operationID;
    Operation op;
}Decision;

//Methods//
//-------//
//execute decision
void Decision_Execute(Decision *decision);
//assumption of failure, also works for "do nothing operator"
void Decision_AssumptionOfFailure(int operationID, long currentTime);
//MSC decision making rule applying when goal is an operation
Decision Decision_Suggest(Event *goal, long currentTime);

#endif
