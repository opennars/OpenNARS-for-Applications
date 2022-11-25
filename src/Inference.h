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

#ifndef H_INFERENCE
#define H_INFERENCE

///////////////////
//  INFERENCE    //
///////////////////
//This module handles all temporal (NAL-7) and procedural (NAL-8) inference.

//References//
//----------//
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include "Event.h"
#include "Narsese.h"
#include "Implication.h"
#include "Globals.h"
#include "Variable.h"
#include <string.h>

//Methods//
//-------//
//{Event a.} |- Event a. Truth_Projection (projecting to current time)
Event Inference_EventUpdate(Event *ev, long currentTime);
//{Event a., Event b.} |- Event (a &/ b). Truth_Intersection (after projecting a to b)
Event Inference_BeliefIntersection(Event *a, Event *b, bool *success);
//{Event a., Event b.} |- Implication <a =/> b>. Truth_Eternalize(Truth_Induction) (after projecting a to b)
Implication Inference_BeliefInduction(Event *a, Event *b, bool *success);
//{Implication <a =/> b>., <a =/> b>.} |- Implication <a =/> b>. Truth_Revision
Implication Inference_ImplicationRevision(Implication *a, Implication *b);
//{Event b!, Implication <a =/> b>.} |- Event a! Truth_Deduction
Event Inference_GoalDeduction(Event *component, Implication *compound, long currentTime);
//{Event (a &/ b)!, Event a.} |- Event b! Truth_Deduction
Event Inference_GoalSequenceDeduction(Event *compound, Event *component, long currentTime);
//{Event a!, Event a!} |- Event a! Truth_Revision or Choice (dependent on evidental overlap)
Event Inference_RevisionAndChoice(Event *existing_potential, Event *incoming_spike, long currentTime, bool *revised);
//{Event a., Implication <a =/> b>.} |- Event b.  Truth_Deduction
Event Inference_BeliefDeduction(Event *component, Implication *compound);
Event Inference_EventRevision(Event *a, Event *b);

#endif
