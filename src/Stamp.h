#ifndef STAMP_H
#define STAMP_H

//References//
//----------//
#include <stdbool.h>

#define STAMP_SIZE 20

#define STAMP_FREE -1

/** Stamp as implemented by all NARS implementations */
typedef struct {
	/** ids of stamp */
	long ids[STAMP_SIZE];
} Stamp;

void stamp_init(Stamp *stamp);

bool stamp_checkOverlap(Stamp *a, Stamp *b);

#endif