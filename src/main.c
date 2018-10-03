#include <stdio.h>
#include <stdlib.h>
#include "SDR.h"
#include "Memory.h"
#include "Encode.h"

void SDR_Test()
{
    SDR_INIT();
    SDR mySDR = Encode_Term("term1");
    SDR_PrintWhereTrue(&mySDR);
    //not ready yet:
    SDR sdr2 = SDR_PermuteByRotation(&mySDR, true);
    SDR_PrintWhereTrue(&sdr2);
    int perm[SDR_SIZE];
    int perm_inv[SDR_SIZE];
    SDR_GeneratePermutation(perm, perm_inv);
    for(int i=0;i<SDR_SIZE; i++)
    {
        //printf("%d\n",perm[i]);
    }
    SDR sdr3 = SDR_Permute(&mySDR, perm);
    SDR_PrintWhereTrue(&sdr3);
}

void Stamp_Test()
{
    Stamp stamp1 = (Stamp) { .evidentalBase = {1,2} };
    Stamp_print(&stamp1);
    Stamp stamp2 = (Stamp) { .evidentalBase = {2,3,4} };
    Stamp_print(&stamp2);
    Stamp stamp3 = Stamp_make(&stamp1, &stamp2);
    printf("zipped:");
    Stamp_print(&stamp3);
    printf("stamps overlapped? %d\n",Stamp_checkOverlap(&stamp1,&stamp2));
}

int main() 
{
    SDR_Test();
    Stamp_Test();
    
    /*
    // memory
    Memory memory;
    memory_RESET(&memory);
    
    // first test for concept
    // TODO< calloc concept dynamically >
    Concept conceptA;
    SDR conceptAName = Encode_Term(2);
    Concept_RESET(&conceptA, conceptAName);
    memory_appendConcept(&memory, &conceptA);

    printf("conceptA.ptr=%d\n", &conceptA);

    Concept *closest = memory_getClosestConceptByName(&memory, &conceptAName);

    printf("closest.ptr=%d\n", closest);    

    //numeric encoder test
    int w = 40;
    SDR sdrForNumber = Encode_Scalar(w, 0, 64, 30);

    printf("SDR for number 30:\n");
    SDR_PrintWhereTrue(&sdrForNumber);
    */



    return 0;
}

