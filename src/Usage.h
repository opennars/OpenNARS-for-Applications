#ifndef USEFULNESSVALUE_H
#define USEFULNESSVALUE_H

/////////////////////////////
//  ANSNA usefulness value //
/////////////////////////////

//Data structure//
//--------------//
typedef struct {
    /** use_count, how often it was used in total */
    int useCount;
    /** age, how many cycles ago it was last used */
    int lastUsed;
} Usage;

//Methods//
//-------//
//how useful it is in respect to the current moment
double Usage_usefulness(Usage *usage, long currentTime);
//use the item
Usage Usage_use(Usage *usage, long currentTime);
#endif
