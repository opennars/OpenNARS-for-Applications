#ifndef CONCEPT_H
#define CONCEPT_H

///////////////////
//  SDR Concept  //
///////////////////
//A concept named by a SDR

//References//
//-----------//
#include "FIFO.h"
#include "Table.h"

//Data structure//
//--------------//
typedef struct {
    Attention attention;
    Usage usage;
    /** name of the concept like in OpenNARS */
    long id; //ID assigned to the concept on conceptualization, cleaner than using its address
    SDR sdr;
    SDR_HASH_TYPE sdr_hash;
    FIFO event_beliefs;
    FIFO event_goals;
    //TODO replace with Table, same as belief tables in OpenNARS:
    Table precondition_beliefs;
    Table postcondition_beliefs;
} Concept;

//Methods//
//-------//
//Assign a new name to a concept
void Concept_SetSDR(Concept *concept, SDR sdr);
//print a concept
void Concept_Print(Concept *concept);

#endif
