#ifndef INFERENCE_H
#define INFERENCE_H

///////////////////
//  INFERENCE    //
///////////////////
//Support for NAL inference on SDR's
//But only a limited set:
//the commented derivations are all that need to happen in ANSNA.

//References//
//-----------//
#include <stdbool.h>
#include "SDR.h"
#include "Stamp.h"
#include "Task.h"
#include "TruthValue.h"

//Methods//
//-------//
//{Event task a., Event belief b.} |- Derived event task (&/,a,b).
Task Inference_BeliefEventIntersection(Task *a, Task *b);
//{Event task a., Event belief b.} |- Precondition and Postcondition belief <a =/> c>.
Task Inference_BeliefInduction(Task *subject, Task *predicate);
//{Precondition or Postcondition belief a., Precondition or Postcondition belief a.} |- 
// Precondition or Postcondition belief a.
Task Inference_BeliefRevision(Task *a, Task *b);
//{Event task a., Postcondition belief <a =/> b>.} |- Derived event task b.
Task Inference_BeliefEventDeduction(Task *component, Task *compound);
//{Event task b!, Postcondition belief <a =/> b>.} |- Derived event task a!
Task Inference_GoalEventDeduction(Task *component, Task *compound);
//{Event task b., Postcondition belief <a =/> b>.} |- Derived event task a.
Task Inference_BeliefEventAbduction(Task *component, Task *compound);
//{Event task a!, Precondition belief <a =/> b>.} |- Derived event task b!
Task Inference_GoalEventAbduction(Task *component, Task *compound);

#endif
