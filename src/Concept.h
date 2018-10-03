#ifndef CONCEPT_H
#define CONCEPT_H

///////////////////
//  SDR Concept  //
///////////////////
//A concept named by a SDR

//References//
//-----------//
#include "SDR.h"
#include "Event.h"
#include "Implication.h"
#include "Attention.h"
#include "FIFO.h"

//Parameters//
//----------//
#define PRECONDITION_BELIEFS_MAX 512
#define POSTCONDITION_BELIEFS_MAX 512

//Data structure//
//--------------//
typedef struct {
    Attention attention;
    Usage usage;
    /** name of the concept like in OpenNARS */
    SDR sdr;
    SDR_HASH_TYPE sdr_hash;
    FIFO event_beliefs;
    FIFO event_goals;
    //TODO replace with Table, same as belief tables in OpenNARS:
    Implication precondition_beliefs[PRECONDITION_BELIEFS_MAX];
    int precondition_beliefs_amount;
    Implication postcondition_beliefs[POSTCONDITION_BELIEFS_MAX];
    int postcondition_beliefs_amount;
} Concept;

//Methods//
//-------//
//Assign a new name to a concept
void Concept_SetSDR(Concept *concept, SDR sdr);

#endif
