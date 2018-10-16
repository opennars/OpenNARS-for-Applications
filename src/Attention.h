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
    //when relative forgetting was applied the last time to the item
    long lastForgotten;
} Attention;

//Parameters//
//----------//
//The max. value of a useful concept its priority value can not sink under,
//to allow useful concepts to survive in the memory priority queue even though
//they might not be useful in the current moment 
#define USEFULNESS_MAX_PRIORITY_BARRIER 0.1
#define EVENT_DURABILITY 0.5
#define CONCEPT_DURABILITY 0.5

//Methods//
//-------//
//Relative forget a task after it goes through attention buffer 
Attention Attention_forgetEvent(Attention *eventAttention, long currentTime);
//Relatively forget a concept after it received a task
Attention Attention_forgetConcept(Attention *conceptAttention, Usage *conceptUsage, long currentTime);
//activate a concept with a even
Attention Attention_activateConcept(Attention *conceptAttention, Attention *eventAttention);
//attention of derived event
Attention Attention_deriveEvent(Attention *conceptAttention, Truth *truth, long currentTime);
//attention of input event
Attention Attention_inputEvent(Truth *truth, long currentTime);
//attention print
void Attention_Print(Attention *attention);

#endif
