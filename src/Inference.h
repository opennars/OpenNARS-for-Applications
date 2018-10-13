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
#include <stdlib.h>
#include "Event.h"
#include "Implication.h"

//Parameters//
//----------//
#define ASSUMPTION_OF_FAILURE_CONFIDENCE 0.05

//Methods//
//-------//
//{Event a., Event b.} |- Event (&/,a,b).
Event Inference_BeliefIntersection(Event *a, Event *b);
//{Event a., Event b.} |- Implication <a =/> c>.
Implication Inference_BeliefInduction(Event *a, Event *b);
//{Event a., Event a.} |- Event a.
//{Event a!, Event a!} |- Event a!
Event Inference_EventRevision(Event *a, Event *b);
//{Implication <a =/> b>., <a =/> b>.} |- Implication <a =/> b>.
Implication Inference_ImplicationRevision(Implication *a, Implication *b);
//{Event a., Implication <a =/> b>.} |- Event b.
Event Inference_BeliefDeduction(Event *component, Implication *compound);
//{Event b!, Implication <a =/> b>.} |- Event a!
Event Inference_GoalDeduction(Event *component, Implication *compound);
//{Event b., Implication <a =/> b>.} |- Event a.
Event Inference_BeliefAbduction(Event *component, Implication *compound);
//{Event task a!, Implication <a =/> b>.} |- Event b!
Event Inference_GoalAbduction(Event *component, Implication *compound);
//When an implication is used for Inference_BeliefDeduction, it receives a little bit of negative evidence
Implication Inference_AssumptionOfFailure(Implication *compound);

#endif
