#ifndef DECISION_H
#define DECISION_H

//References//
//----------//
#include <stdbool.h>
#include <stdio.h>
#include "Memory.h"

//Parameters//
//----------//
//confidence needed for executions
#define DECISION_THRESHOLD 0.5
//motor babbling chance
#define MOTOR_BABBLING_CHANCE 0.001

//Data structure//
//--------------//
typedef struct
{
    bool matched;
    bool executed;
    Operation op;
}Decision;

//Methods//
//-------//
//it returns whether and which operation matched and whether it was executed
Decision Decision_PotentiallyExecute(Concept *c, Event *goal, long currentTime);
//"reflexes" to try different operations, especially important in the beginning
Decision Decision_MotorBabbling();
//Motor tagging, so that an operation gets attached to the precondition events
void Decision_MotorTagging(Concept *c, Operation op);

#endif
