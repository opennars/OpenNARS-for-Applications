#ifndef CONCEPT_H
#define CONCEPT_H

#include "SDR.h"
#include "Task.h"

///////////////////
//  SDR Concept  //
///////////////////
//A concept named by a SDR

//Parameters//
//----------//
//#define CONCEPT_TERMS 512
#define CONCEPT_TASKS 512

//Data structure//
//--------------//
typedef struct {
    /** name of the concept like in OpenNARS */
    SDR *name;
    //null pointer indicates free space
    //SDR *terms[CONCEPT_TERMS];


    //null pointer indicates free space
    Task *tasks[CONCEPT_TASKS];
} Concept;

//Methods//
//-------//
void concept_init(Concept *concept, SDR *name);

#endif
