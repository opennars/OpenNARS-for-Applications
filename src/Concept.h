#ifndef CONCEPT_H
#define CONCEPT_H

#include "SDR.h"

///////////////////
//  SDR Concept  //
///////////////////

//Description//
//-----------//
//A concept named by a SDR

//Parameters//
//----------//
#define CONCEPT_TERMS 512
#define CONCEPT_COUNT 64

//Data structure//
//--------------//
typedef struct {
    /** name of the concept like in OpenNARS */
    SDR *name;
    
    //null pointer indicates free space
    SDR *terms[CONCEPT_TERMS];
} Concept;

//Methods//
//-------//
void concept_init(Concept *concept, SDR *name);

#endif
