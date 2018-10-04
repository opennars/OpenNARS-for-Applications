#include <stdio.h>
#include <stdlib.h>
#include "SDR.h"
#include "Memory.h"
#include "Encode.h"

void assert(bool b)
{
    if(!b)
    {
        printf("Test error\n");
        exit(1);
    }
}

void SDR_Test()
{
    printf(">>SDR Test Start\n");
    SDR mySDR = Encode_Term("term1");
    assert(SDR_CountTrue(&mySDR) == TERM_ONES);
    SDR sdr2 = SDR_PermuteByRotation(&mySDR, true);
    SDR_PrintWhereTrue(&sdr2);
    assert(SDR_CountTrue(&sdr2) == TERM_ONES);
    int perm[SDR_SIZE];
    int perm_inv[SDR_SIZE];
    SDR_GeneratePermutation(perm, perm_inv);
    SDR sdr3 = SDR_Permute(&mySDR, perm);
    assert(SDR_CountTrue(&sdr3) == TERM_ONES);
    SDR_PrintWhereTrue(&sdr3);
    SDR sdrTest = {0};
    SDR_WriteBit(&sdrTest, 255, 1);
    SDR_Swap(&sdrTest, 256, 255);
    assert(SDR_ReadBit(&sdrTest, 254) == 0);
    assert(SDR_ReadBit(&sdrTest, 255) == 0);
    assert(SDR_ReadBit(&sdrTest, 256) == 1);
    SDR_PrintWhereTrue(&sdrTest);
    printf("<<SDR Test successful\n");
}

void Stamp_Test()
{
    printf(">>Stamp test start\n");
    Stamp stamp1 = (Stamp) { .evidentalBase = {1,2} };
    Stamp_print(&stamp1);
    Stamp stamp2 = (Stamp) { .evidentalBase = {2,3,4} };
    Stamp_print(&stamp2);
    Stamp stamp3 = Stamp_make(&stamp1, &stamp2);
    printf("zipped:");
    Stamp_print(&stamp3);
    assert(Stamp_checkOverlap(&stamp1,&stamp2) == true);
    printf("<<Stamp test successful\n");
}

int main() 
{
    SDR_INIT();
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

