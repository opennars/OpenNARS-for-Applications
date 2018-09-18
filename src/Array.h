#ifndef ARRAY_H
#define ARRAY_H

///////////////////
//  ARRAY        //
///////////////////
//An array that supports to be resized

//References//
//----------//
#include <stdbool.h>

//Parameters//
//----------//
#define ARRAY_ALLOC 32

//Data structure//
//--------------//
typedef struct {
	/** raw pointer, don't use this from the outside! */
    void *ptr;
    /** size of one element for indexing */
    int elementSize;
    /** number of allocated elements - not to be confused with used elements */
    int allocatedElements;
	/** number of used elements */
    int usedElements;
} Array;

//Methods//
//-------//
//Init an array of certain size
bool array_init(Array *array, int elementSize);
//Free the array
void array_free(Array *array);
//Return item at specifc index
void *array_at(Array *array, int idx);
//Return length of array
int array_length(Array *array);
//Appent item to array
bool array_append(Array *array, void *ptr);

#endif
