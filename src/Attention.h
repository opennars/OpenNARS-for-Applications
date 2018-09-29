#ifndef ATTENTION_H
#define ATTENTION_H

////////////////////////////////////////////////////
//  ANSNA attention value and attention functions //
////////////////////////////////////////////////////

//References//
//-----------//
#include "Usage.h"
#include "Truth.h"

//Data structure//
//--------------//
typedef struct {
    //short term importance
    double priority;
    //how fast the item decays
    double durability;
} Attention;

//Parameters//
//----------//
//The max. value of a useful concept its priority value can not sink under,
//to allow useful concepts to survive in the memory priority queue even though
//they might not be useful in the current moment 
#define USEFULNESS_MAX_PRIORITY_BARRIER 0.1

//Methods//
//-------//
//Relative forget a task after it goes through attention buffer 
Attention Attention_forgetTask(Attention *taskAttention);
//Relatively forget a concept after it received a task
Attention Attention_forgetConcept(Attention *conceptAttention, Usage *conceptUsage, long currentTime);

#endif
