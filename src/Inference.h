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

//Parameters//
//----------//
#define EVIDENTAL_HORIZON 1.0


//Methods//
//-------//
//Get a stack-allocated truth valie
TruthValue getTruthValue(double frequency, double confidence);
//The output is conjunctively determined by the inputs
double and(double a, double b);
//The output is disjunctively determined by the input
double or(double a, double b);
//{Precondition or Postcondition belief a., Precondition or Postcondition belief a.} |- 
// Precondition or Postcondition belief a.
TruthValue revision(TruthValue v1, TruthValue v2);
//{Event task a., Postcondition belief <a =/> b>.} |- Derived event task b.
//{Event task b!, Precondition belief <a =/> b>.} |- Derived event task a!
TruthValue deduction(TruthValue v1, TruthValue v2);
//{Event task a., Event belief b.} |- Precondition and Postcondition belief <a =/> c>.
TruthValue induction(TruthValue v1, TruthValue v2);
//{Event task b., Precondition belief <a =/> b>.} |- Derived event task a.
//{Event task a!, Precondition belief <a =/> b>.} |- Derived event task b!
TruthValue abduction(TruthValue v1, TruthValue v2);
//{Event task a., Event belief b.} |- Derived event task (&/,a,b).
TruthValue intersection(TruthValue v1, TruthValue v2);


//{Event task a., Postcondition belief <a =/> b>.} |- Derived event task b.
Task Inference_BeliefEventDeduction(Task *component, Task *compound);

//{Event task a., Event belief b.} |- Precondition and Postcondition belief <a =/> c>.
Task Inference_eventInduction(Task *subject, Task *predicate);

#endif
