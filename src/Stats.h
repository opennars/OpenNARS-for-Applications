#ifndef H_STATS
#define H_STATS

///////////////////
// Runtime stats //
///////////////////

//References//
//----------//
#include <stdio.h>
//#include "Memory.h"

//Global vars//
//-----------//
extern long Stats_countConceptsMatchedTotal;
extern long Stats_countConceptsMatchedMax;

//Methods//
//-------//
void Stats_Print(long currentTime);

#endif
