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

#ifndef H_MEMORY
#define H_MEMORY

/////////////////
//  NAR Memory //
/////////////////
//The concept-based memory of NAR
//Concepts are created from events
//and are linked to each other with temporal implications
//and by their subterms via InvertedAtomIndex

//References//
//////////////
#include <math.h>
#include "Concept.h"
#include "OccurrenceTimeIndex.h"
#include "InvertedAtomIndex.h"
#include "PriorityQueue.h"
#include "Config.h"
#include "HashTable.h"
#include "Variable.h"

//Parameters//
//----------//
//Inferences per cycle (amount of events from cycling events)
extern double PROPAGATION_THRESHOLD;
extern bool PRINT_DERIVATIONS;
extern bool PRINT_INPUT;
extern double conceptPriorityThreshold;

//Data structure//
//--------------//
typedef struct
{
    Substitution subs;
    bool failed;
}Feedback; //operation feedback
typedef Feedback (*Action)(Term);
typedef struct
{
    Term term;
    Action action;
    Term arguments[OPERATIONS_BABBLE_ARGS_MAX];
    bool stdinOutput;
}Operation;
extern bool ontology_handling;
extern Event selectedBeliefs[BELIEF_EVENT_SELECTIONS]; //better to be global
extern double selectedBeliefsPriority[BELIEF_EVENT_SELECTIONS]; //better to be global
extern int beliefsSelectedCnt;
extern Event selectedGoals[GOAL_EVENT_SELECTIONS]; //better to be global
extern double selectedGoalsPriority[GOAL_EVENT_SELECTIONS]; //better to be global
extern int goalsSelectedCnt;
extern int concept_id;
//Concepts in main memory:
extern PriorityQueue concepts;
//cycling events cycling in main memory:
extern PriorityQueue cycling_belief_events;
extern PriorityQueue cycling_goal_events[CYCLING_GOAL_EVENTS_LAYERS];
//Hashtable of concepts used for fast retrieval of concepts via term:
extern HashTable HTconcepts;
//OccurrenceTimeIndex for accelerating temporal induction
extern OccurrenceTimeIndex occurrenceTimeIndex;
//Registered perations
extern Operation operations[OPERATIONS_MAX];
//Priority threshold for printing derivations
extern double PRINT_EVENTS_PRIORITY_THRESHOLD;

//Methods//
//-------//
//Init memory
void Memory_INIT();
//Find a concept
Concept *Memory_FindConceptByTerm(Term *term);
//Create a new concept
Concept* Memory_Conceptualize(Term *term, long currentTime);
//Add event to memory
void Memory_AddEvent(Event *event, long currentTime, double priority, bool input, bool derived, bool revised, int layer);
void Memory_AddInputEvent(Event *event, long currentTime);
//Add operation to memory
void Memory_AddOperation(int id, Operation op);
//check if implication is still valid (source concept might be forgotten)
bool Memory_ImplicationValid(Implication *imp);
//Print an event in memory:
void Memory_printAddedEvent(Stamp *stamp, Event *event, double priority, bool input, bool derived, bool revised, bool controlInfo, bool selected);
//Print an implication in memory:
void Memory_printAddedImplication(Stamp *stamp, Term *implication, Truth *truth, double occurrenceTimeOffset, double priority, bool input, bool revised, bool controlInfo);
//Get operation ID
int Memory_getOperationID(Term *term);

#endif
