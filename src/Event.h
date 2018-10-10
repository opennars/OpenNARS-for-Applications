#ifndef EVENT_H
#define EVENT_H

///////////////////
//  SDR Event    //
///////////////////
//also see https://github.com/patham9/ANSNA/wiki/Input

//References//
//-----------//
#include "SDR.h"
#include "Stamp.h"
#include "Truth.h"
#include "Attention.h"

//Data structure//
//--------------//
#define EVENT_TYPE_GOAL 1
#define EVENT_TYPE_BELIEF 2
#define EVENT_TYPE_DELETED 0
typedef struct {
    Attention attention;
    SDR sdr;
    SDR_HASH_TYPE sdr_hash;
    char type; //either JUDGMENT or GOAL
    Truth truth;
    Stamp stamp;
    long occurrenceTime;
} Event;

//Methods//
//-------//
//Assign a new name to an event
void Event_SetSDR(Event *event, SDR sdr);

#endif
