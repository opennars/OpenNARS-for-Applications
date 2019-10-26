#ifndef SDR_H
#define SDR_H

///////////////////
//  SDR_TERM     //
///////////////////

//Parameters//
//----------//
#define MAX_SEQUENCE_LEN 2

//Description//
//-----------//
//An SDR is an blocksay of a specific number of 128 bit blocks
//(that way no Hash ops are necessary, it's faster for this SDR size)

//References//
//-----------//
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "Truth.h"

//Data structure//
//--------------//
typedef struct
{
    char terms[MAX_SEQUENCE_LEN];
}SDR;

//Methods//
//-------//
// print indices of true bits
void Term_Print(SDR *sdr);
//Tuple on the other hand:
SDR Term_Tuple(SDR *a, SDR *b);
//Whether two SDR's are equal completely
bool Term_Equal(SDR *a, SDR *b);

#endif
