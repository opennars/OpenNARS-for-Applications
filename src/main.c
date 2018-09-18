#include <stdio.h>
#include <stdlib.h>
#include "SDR.h"
#include "Memory.h"
#include "ScalarEncoder.h"

int main() 
{
    SDR_INIT();
    SDR *mySDR = getTerm(1);
    printSDRWhereTrue(mySDR);
    //not ready yet:
    SDR sdr2 = Permute(*mySDR, true);
    printSDRWhereTrue(&sdr2);
    
    // memory
    Memory memory;
    memory_init(&memory);
    
    // first test for concept
    // TODO< calloc concept dynamically >
    Concept conceptA;
    SDR *conceptAName = getTerm(2);
    concept_init(&conceptA, conceptAName);
    memory_appendConcept(&memory, &conceptA);

    printf("conceptA.ptr=%d\n", &conceptA);

    Concept *closest = memory_getClosestConceptByName(&memory, conceptAName);

    printf("closest.ptr=%d\n", closest);    



    /* numeric encoder test */
    int w = 40;
    SDR sdrForNumber = encoder_scalar(w, 0, 64, 30);

    printf("SDR for number 30:\n");
    printSDRWhereTrue(&sdrForNumber);




    return 0;
}

