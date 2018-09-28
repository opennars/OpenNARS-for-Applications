#ifndef STAMP_H
#define STAMP_H

//References//
//----------//
#include <stdbool.h>

#define STAMP_SIZE 20

#define STAMP_FREE -1

/** Stamp as implemented by all NARS implementations */
typedef struct {
    /** evidentalBase of stamp */
    long evidentalBase[STAMP_SIZE];
} Stamp;

void Stamp_RESET(Stamp *stamp);
//zip stamp1 and stamp2 into a stamp
Stamp Stamp_make(Stamp *stamp1, Stamp *stamp2);
//true iff there is evidental base overlap between a and b
bool Stamp_checkOverlap(Stamp *a, Stamp *b);

#endif
