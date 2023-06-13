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

#ifndef EVENT_H
#define EVENT_H

///////////////////
//   Event       //
///////////////////
//An event named by a term
//It can be a belief event corresponding to a certain
//invariance observed by the system
//Input patterns usually directly come from sensory channels
//while derived events can be compounds built by the system
//Events can also be goals, which makes
//the system want to observe a belief event with same term asap.

//References//
//----------//
#include "Term.h"
#include "Stamp.h"

//Data structure//
//--------------//
extern long base;
extern Stamp importstamp;
#define EVENT_TYPE_GOAL 1
#define EVENT_TYPE_BELIEF 2
#define EVENT_TYPE_DELETED 0
typedef struct {
    Term term;
    char type; //either JUDGMENT or GOAL
    Truth truth;
    Stamp stamp;
    long occurrenceTime;
    double occurrenceTimeOffset; //necessary if the event is an =/>
    bool processed;
    long creationTime;
    bool input;
} Event;

//Methods//
//-------//
//Init/Reset module
void Event_INIT();
//construct an input event
Event Event_InputEvent(Term term, char type, Truth truth, double occurrenceTimeOffset, long currentTime);
//Whether two events are the same
bool Event_Equal(Event *event, Event *existing);
//Whether the left event with same term and stamp overlap is less confident than the second
bool Event_EqualTermEqualStampLessConfidentThan(Event *event, Event *existing);
//Eternalized event
Event Event_Eternalized(Event *event);

#endif
