#include <stdio.h>
#include <stdlib.h>
#include "SDR.h"
#include "Memory.h"
#include "Encode.h"

void assert(bool b, char* message)
{
    if(!b)
    {
        printf(message);
        printf("\nTest error\n");
        exit(1);
    }
}

void SDR_Test()
{
    printf(">>SDR Test Start\n");
    SDR mySDR = Encode_Term("term1");
    assert(SDR_CountTrue(&mySDR) == TERM_ONES, "SDR term should have TERM_ONES ones");
    SDR sdr2 = SDR_PermuteByRotation(&mySDR, true);
    SDR mySDR_Recons1 = SDR_PermuteByRotation(&sdr2, false);
    assert(SDR_Equal(&mySDR_Recons1, &mySDR), "Inverse rotation should lead to original result");
    int perm[SDR_SIZE];
    int perm_inv[SDR_SIZE];
    SDR_GeneratePermutation(perm, perm_inv);
    SDR sdr3 = SDR_Permute(&mySDR, perm);
    SDR mySDR_recons2 = SDR_Permute(&sdr3, perm_inv);
    SDR_PrintWhereTrue(&mySDR);
    SDR_PrintWhereTrue(&mySDR_recons2);
    assert(SDR_Equal(&mySDR_recons2, &mySDR), "Inverse permutation should lead to original result");
    SDR sdrTest = {0};
    SDR_WriteBit(&sdrTest, 255, 1);
    SDR_Swap(&sdrTest, &sdrTest, 256, 255);
    assert(SDR_ReadBit(&sdrTest, 254) == 0, "bit was not set, should be 0");
    assert(SDR_ReadBit(&sdrTest, 255) == 0, "bit was swapped to 0, should be 0");
    assert(SDR_ReadBit(&sdrTest, 256) == 1, "bit was swapped to 1, should be 1");
    printf("testing tuples now:\n");
    SDR_PrintWhereTrue(&mySDR);
    SDR mySDR2 = Encode_Term("term2");
    SDR_PrintWhereTrue(&mySDR2);
    printf("recons:");
    SDR tuple = SDR_Tuple(&mySDR, &mySDR2);
    SDR SDR1Recons = SDR_TupleGetFirstElement(&tuple, &mySDR2);
    SDR_PrintWhereTrue(&SDR1Recons);
    SDR SDR2Recons = SDR_TupleGetFirstElement(&tuple, &mySDR);
    SDR_PrintWhereTrue(&SDR2Recons);
    Truth selfTest = SDR_Similarity(&mySDR, &mySDR);
    printf("sdr1 sdr1 similarity: %f %f\n", selfTest.frequency, selfTest.confidence);
    assert(selfTest.frequency == 1, "No negative evidence is allowed to be found when matching to itself");
    Truth t1 = SDR_Similarity(&mySDR, &SDR1Recons);
    printf("sdr1 sdr1recons similarity: %f %f\n", t1.frequency, t1.confidence);
    Truth t2 = SDR_Similarity(&mySDR2, &SDR2Recons);
    printf("sdr2 sdr2recons similarity: %f %f\n", t2.frequency, t2.confidence);
    Truth t3 = SDR_Similarity(&mySDR, &SDR2Recons);
    printf("sdr1 sdr2recons similarity: %f %f\n", t3.frequency, t3.confidence);
    Truth t4 = SDR_Similarity(&mySDR2, &SDR1Recons);
    printf("sdr2 sdr1recons similarity: %f %f\n", t4.frequency, t4.confidence);
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
        assert(FIFO_SIZE-i == fifo.array[i].stamp.evidentalBase[0], "Item at FIFO position has to be right");
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
    assert(ret.occurrenceTime > i*10 && ret.occurrenceTime < i*10+3, "occurrence time has to be within");
    assert(ret.stamp.evidentalBase[0] == i && ret.stamp.evidentalBase[1] == newbase, "it has to be the new event");
    assert(fifo.array[FIFO_SIZE-i].type == EVENT_TYPE_DELETED, "FIFO should have deleted the entry"); //as it was replaced
    Event addedRet = fifo.array[fifo.currentIndex == 0 ? FIFO_SIZE-1 : fifo.currentIndex - 1]; //it is at the "first" position of the FIFO now
    assert(addedRet.stamp.evidentalBase[0] == i && addedRet.stamp.evidentalBase[1] == newbase, "it has to be the new event");
    printf("%f %f \n", ret.truth.frequency, ret.truth.confidence);
    assert(ret.truth.confidence > 0.9, "confidence of revision result should be higher than premise's");
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
    assert(Stamp_checkOverlap(&stamp1,&stamp2) == true, "Stamp should overlap");
    printf("<<Stamp test successful\n");
}

int main() 
{
    srand(1337);
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

