#ifndef STAMP_H
#define STAMP_H

//////////////////////
//  Stamp           //
//////////////////////
//keeps track of evidental bases

//References//
//----------//
#include <stdbool.h>
#include <stdio.h>

//Parameters//
//----------//
#define STAMP_SIZE 20
#define STAMP_FREE 0

//Data structure//
//--------------//
//Stamp as implemented by all NARS implementations 
typedef struct {
    //EvidentalBase of stamp
    long evidentalBase[STAMP_SIZE];
} Stamp;

//Methods//
//-------//
//zip stamp1 and stamp2 into a stamp
Stamp Stamp_make(Stamp *stamp1, Stamp *stamp2);
//true iff there is evidental base overlap between a and b
bool Stamp_checkOverlap(Stamp *a, Stamp *b);
//print stamp
void Stamp_print(Stamp *stamp);

#endif
