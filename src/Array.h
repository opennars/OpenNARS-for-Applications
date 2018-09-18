#ifndef ARRAY_H
#define ARRAY_H

#include <stdbool.h>

typedef struct {
	/** raw pointer, don't use this from the outside! */
    void *ptr;

    /** size of one element for indexing */
    int elementSize;

    /** number of allocated elements - not to be confused with used elements */
    int allocatedElements;


    int usedElements;
} Array;

#define ARRAY_ALLOC 32

bool array_init(Array *array, int elementSize);

void array_free(Array *array);

void *array_at(Array *array, int idx);

// used internally
bool array_expand(Array *array, int additionalNumberOfElements);

int array_length(Array *array);

bool array_append(Array *array, void *ptr);

#endif
