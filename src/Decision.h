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
#define DECISION_THRESHOLD 0.55
//motor babbling chance
#define MOTOR_BABBLING_CHANCE_INITIAL 0.2
extern double MOTOR_BABBLING_CHANCE;
//deadline = predictionTime*ANTICIPATION_FORWARD
#define ANTICIPATION_FORWARD 100.0 
//deadline += ANTICIPATION_WINDOW*variance
#define ANTICIPATION_WINDOW 10.0
//confidence of anticipation
#define ANTICIPATION_CONFIDENCE 0.1

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

#endif
