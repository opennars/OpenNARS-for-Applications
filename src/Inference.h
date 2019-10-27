#ifndef INFERENCE_H
#define INFERENCE_H

///////////////////
//  INFERENCE    //
///////////////////
//Support for NAL inference on Term's
//But only a limited set:
//the commented derivations are all that need to happen in YAN.

//References//
//-----------//
#include <stdbool.h>
#include <stdlib.h>
#include "Event.h"
#include "Implication.h"
#include "Globals.h"
#include <string.h>

//Methods//
//-------//
//{Event a.} |- Event a. updated to currentTime
Event Inference_EventUpdate(Event *ev, long currentTime);
//{Event a., Event b.} |- Event (&/,a,b).
Event Inference_BeliefIntersection(Event *a, Event *b);
//{Event a., Event b.} |- Implication <a =/> c>.
Implication Inference_BeliefInduction(Event *a, Event *b);
//{Implication <a =/> b>., <a =/> b>.} |- Implication <a =/> b>.
Implication Inference_ImplicationRevision(Implication *a, Implication *b);
//{Event b!, Implication <a =/> b>.} |- Event a!
Event Inference_GoalDeduction(Event *component, Implication *compound);
//{Event (&/,a,op())!, Event a.} |- Event op()!
Event Inference_OperationDeduction(Event *compound, Event *component, long currentTime);
//{Event a!, Event a!} |- Event a! (essentially revision or choice dependent on evidental overlap)
Event Inference_IncreasedActionPotential(Event *existing_potential, Event *incoming_spike, long currentTime);
//{Event a., Implication <a =/> b>.} |- Event b.
Event Inference_BeliefDeduction(Event *component, Implication *compound);
#endif
