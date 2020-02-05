#ifndef EVENT_H
#define EVENT_H

///////////////////
//  Term Event   //
///////////////////

//References//
//-----------//
#include "Term.h"
#include "Stamp.h"

//Data structure//
//--------------//
#define EVENT_TYPE_GOAL 1
#define EVENT_TYPE_BELIEF 2
#define EVENT_TYPE_DELETED 0
typedef struct {
    Term term;
    //Term_HASH_TYPE term_hash;
    char type; //either JUDGMENT or GOAL
    Truth truth;
    Stamp stamp;
    long occurrenceTime;
    bool processed;
    bool propagated;
    long creationTime;
} Event;

//Methods//
//-------//
//Init/Reset module
void Event_INIT();
//Assign a new name to an event
void Event_SetTerm(Event *event, Term term);
//construct an input event
Event Event_InputEvent(Term term, char type, Truth truth, long currentTime);
//print event
void Event_Print(Event *event);
//Whether two events are the same
bool Event_Equal(Event *event, Event *existing);

#endif
