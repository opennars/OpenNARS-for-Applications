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

void FIFO_Test()
{
    printf(">>FIFO test start\n");
    FIFO fifo;
    //First, evaluate whether the fifo works, not leading to overflow
    for(int i=FIFO_SIZE*2; i>=1; i--) //"rolling over" once by adding a k*FIFO_Size items
    {
        Event event1 = (Event) { .sdr = Encode_Term("test"), 
                                 .type = EVENT_TYPE_BELIEF, 
                                 .truth = {.frequency = 1.0, .confidence = 0.9},
                                 .stamp = (Stamp) { .evidentalBase = {i} }, 
                                 .occurrenceTime = i*10 };
        FIFO_Add(&event1, &fifo);
    }
    for(int i=0; i<FIFO_SIZE; i++)
    {
        assert(FIFO_SIZE-i == fifo.array[i].stamp.evidentalBase[0]);
    }
    //now see whether a new item is revised with the correct one:
    int i=10; //revise with item 10, which has occurrence time 10
    int newbase = FIFO_SIZE*2+1;
    Event event2 = (Event) { .sdr = Encode_Term("test"), 
                             .type = EVENT_TYPE_BELIEF, 
                             .truth = {.frequency = 1.0, .confidence = 0.9},
                             .stamp = (Stamp) { .evidentalBase = {newbase} }, 
                             .occurrenceTime = i*10+3 };
    Event ret = FIFO_AddAndRevise(&event2, &fifo);
    assert(ret.occurrenceTime > i*10 && ret.occurrenceTime < i*10+3);
    assert(ret.stamp.evidentalBase[0] == i && ret.stamp.evidentalBase[1] == newbase);
    assert(fifo.array[FIFO_SIZE-i].type == EVENT_TYPE_DELETED); //as it was replaced
    Event addedRet = fifo.array[fifo.currentIndex == 0 ? FIFO_SIZE-1 : fifo.currentIndex - 1];
    assert(addedRet.stamp.evidentalBase[0] == i && addedRet.stamp.evidentalBase[1] == newbase); //it is at the first position of the FIFO now
    printf("%f %f \n", ret.truth.frequency, ret.truth.confidence);
    assert(ret.truth.confidence > 0.9);
    printf("<<FIFO Test successful\n");
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
    FIFO_Test();
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

