#include "Array.h"

#include <stdlib.h>
#include <string.h>

bool array_init(Array *array, int elementSize)
{
	array->ptr = calloc(ARRAY_ALLOC, elementSize);
	if (!(array->ptr))
	{
		return 0;
	}
	array->elementSize = elementSize;
	array->allocatedElements = ARRAY_ALLOC;
	array->usedElements = 0;
	return 1;
}

void array_free(Array *array)
{
	free(array->ptr);
	array->ptr = (void*)0;
	array->allocatedElements = 0;
	array->usedElements = 0;
}

void *array_at(Array *array, int idx)
{
	return (void*)(((unsigned char*)array->ptr)[array->elementSize * idx]);
}

// used internally
bool array_expand(Array *array, int additionalNumberOfElements)
{
	void *reallocatedPtr = realloc(array->ptr, (array->allocatedElements + additionalNumberOfElements) * array->elementSize);
	if (!reallocatedPtr)
	{
		return 0;
	}
	array->ptr = reallocatedPtr;
	array->allocatedElements += additionalNumberOfElements;
	return 1;
}

int array_length(Array *array)
{
	return array->usedElements;
}

bool array_append(Array *array, void *ptr)
{
	if (array->usedElements + 1 > array->allocatedElements)
	{
		if (!array_expand(array, ARRAY_ALLOC))
		{
			return 0;
		}
	}
	void *destPtr = (void*)&(((unsigned char*)array->ptr)[array->usedElements * array->elementSize]);
	memcpy(destPtr, ptr, array->elementSize);
	array->usedElements++;
	return 1;
}

bool array_removeAt(Array *array, int idx) {
	// TODO< check/assert >

	int length = array_length(array);

	for (int i=idx;i<length-1;i++) {
		memcpy(
			array_at(array, i),
			array_at(array, i+1),
			array->elementSize
		);
	}

	array->usedElements--;

	return 1;
}