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
#include "Globals.h"
#include <string.h>

//Methods//
//-------//
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
//{Event a!, Event a!} |- Event a! (revision and choice)
Event Inference_IncreasedActionPotential(Event *existing_potential, Event *incoming_spike, long currentTime);

#endif
