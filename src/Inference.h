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
#include "Narsese.h"
#include "Implication.h"
#include "Globals.h"
#include <string.h>

//Methods//
//-------//
//{Event a.} |- Event a. Truth_Projection (projecting to current time)
Event Inference_EventUpdate(Event *ev, long currentTime);
//{Event a., Event b.} |- Event (&/,a,b). Truth_Intersection (after projecting b to a)
Event Inference_BeliefIntersection(Event *a, Event *b);
//{Event a., Event b.} |- Implication <a =/> c>. Truth_Eternalize(Truth_Induction) (after projecting b to a)
Implication Inference_BeliefInduction(Event *a, Event *b);
//{Implication <a =/> b>., <a =/> b>.} |- Implication <a =/> b>. Truth_Revision
Implication Inference_ImplicationRevision(Implication *a, Implication *b);
//{Event b!, Implication <a =/> b>.} |- Event a! Truth_Deduction
Event Inference_GoalDeduction(Event *component, Implication *compound);
//{Event (&/,a,op())!, Event a.} |- Event op()! Truth_Deduction
Event Inference_OperationDeduction(Event *compound, Event *component, long currentTime);
//{Event a!, Event a!} |- Event a! Truth_Revision or Choice (dependent on evidental overlap)
Event Inference_IncreasedActionPotential(Event *existing_potential, Event *incoming_spike, long currentTime, bool *revised);
//{Event a., Implication <a =/> b>.} |- Event b.  Truth_Deduction
Event Inference_BeliefDeduction(Event *component, Implication *compound);

#endif
