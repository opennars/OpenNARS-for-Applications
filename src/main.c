#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "SDR.h"
#include "Memory.h"
#include "Encode.h"
#include "ANSNA.h"

void SDR_Test()
{
    printf(">>SDR Test Start\n");
    printf("testing encoding and permutation:\n");
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
    assert(SDR_Equal(&mySDR_recons2, &mySDR), "Inverse permutation should lead to original result");
    printf("testing tuples now:\n");
    SDR mySDR2 = Encode_Term("term2");
    printf("recons:");
    SDR tuple = SDR_Tuple(&mySDR, &mySDR2);
    SDR SDR1Recons = SDR_TupleGetFirstElement(&tuple, &mySDR2);
    SDR SDR2Recons = SDR_TupleGetSecondElement(&tuple, &mySDR);
    Truth selfTest = SDR_Similarity(&mySDR, &mySDR);
    printf("sdr1 sdr1 similarity: %f %f\n", selfTest.frequency, selfTest.confidence);
    assert(selfTest.frequency == 1, "No negative evidence is allowed to be found when matching to itself");
    Truth t1 = SDR_Similarity(&mySDR, &SDR1Recons);
    assert(t1.frequency == 1, "Reconstructed tuple element1 should be almost the same as the original");
    Truth t2 = SDR_Similarity(&mySDR2, &SDR2Recons);
    assert(t2.frequency == 1, "Reconstructed tuple element2 should be almost the same as the original");
    Truth t3 = SDR_Similarity(&mySDR, &SDR2Recons);
    assert(t3.frequency < 0.5, "These elements should mostly differ");
    Truth t4 = SDR_Similarity(&mySDR2, &SDR1Recons);
    assert(t4.frequency < 0.5, "These elements should mostly differ too");
    printf("<<SDR Test successful\n");
}

void FIFO_Test()
{
    printf(">>FIFO test start\n");
    FIFO fifo = {0};
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
    int i=3; //revise with item 10, which has occurrence time 10
    int newbase = FIFO_SIZE*2+1;
    Event event2 = (Event) { .sdr = Encode_Term("test"), 
                             .type = EVENT_TYPE_BELIEF, 
                             .truth = {.frequency = 1.0, .confidence = 0.9},
                             .stamp = (Stamp) { .evidentalBase = {newbase} }, 
                             .occurrenceTime = i*10+3 };
    Event ret = FIFO_AddAndRevise(&event2, &fifo);
    assert(ret.occurrenceTime > i*10 && ret.occurrenceTime < i*10+3, "occurrence time has to be within");
    assert(ret.stamp.evidentalBase[0] == i && ret.stamp.evidentalBase[1] == newbase, "it has to be the new event");
    assert(fifo.array[FIFO_SIZE-i].stamp.evidentalBase[0] == i && fifo.array[FIFO_SIZE-i].stamp.evidentalBase[1] == newbase, "FIFO should have deleted the entry");
    printf("%f %f \n", ret.truth.frequency, ret.truth.confidence);
    assert(ret.truth.confidence > 0.9, "confidence of revision result should be higher than premise's");
    FIFO fifo2 = {0};
    for(int i=0; i<FIFO_SIZE*2; i++)
    {
        FIFO_Add(&event2, &fifo2);
    }
    assert(fifo2.itemsAmount == FIFO_SIZE, "FIFO size differs");
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

void PriorityQueue_Test()
{
    printf(">>PriorityQueue test start\n");
    PriorityQueue queue;
    int n_items = 10;
    Item items[n_items];
    for(int i=0; i<n_items; i++)
    {
        items[i].address = (void*) ((long) i+1);
        items[i].priority = 0;
    }
    PriorityQueue_RESET(&queue, items, n_items);
    for(int i=0, evictions=0; i<n_items*2; i++)
    {
        PriorityQueue_Push_Feedback feedback = PriorityQueue_Push(&queue, 1.0/((double) (n_items*2-i)));
        if(feedback.added)
        {
            printf("item was added %f %ld\n", feedback.addedItem.priority, (long)feedback.addedItem.address);
        }
        if(feedback.evicted)
        {
            printf("evicted item %f %ld\n", feedback.evictedItem.priority, (long)feedback.evictedItem.address);
            assert(evictions>0 || feedback.evictedItem.priority == 1.0/((double) (n_items*2)), "the evicted item has to be the lowest priority one");
            assert(queue.itemsAmount < n_items+1, "eviction should only happen when full!");
            evictions++;
        }
    }
    printf("<<PriorityQueue test successful\n");
}

void Table_Test()
{
    printf(">>Table test start\n");
    Table table = {0};
    for(int i=TABLE_SIZE*2; i>=1; i--)
    {
        Implication imp = (Implication) { .sdr = Encode_Term("test"), 
                                          .truth = (Truth) { .frequency = 1.0, .confidence = 1.0/((double)(i+1)) },
                                          .stamp = (Stamp) { .evidentalBase = {i} },
                                          .occurrenceTimeOffset = 10 };
        Table_Add(&table, &imp);
    }
    for(int i=0; i<TABLE_SIZE; i++)
    {
        assert(i+1 == table.array[i].stamp.evidentalBase[0], "Item at table position has to be right");
    }
    Implication imp = (Implication) { .sdr = Encode_Term("test"), 
                                      .truth = (Truth) { .frequency = 1.0, .confidence = 0.9},
                                      .stamp = (Stamp) { .evidentalBase = {TABLE_SIZE*2+1} },
                                      .occurrenceTimeOffset = 10 };
    assert(table.array[0].truth.confidence==0.5, "The highest confidence one should be the first.");
    Table_AddAndRevise(&table, &imp, "");
    assert(table.array[0].truth.confidence>0.5, "The revision result should be more confident than the table element that existed.");
    printf("<<Table test successful\n");
}

void Memory_Test()
{
    ANSNA_INIT();
    printf(">>Memory test start\n");
    Event e = Event_InputEvent(Encode_Term("a"), 
                               EVENT_TYPE_BELIEF, 
                               (Truth) {.frequency = 1, .confidence = 0.9}, 
                               1337);
    e.attention.priority = 0.9;
    Memory_addEvent(&e);
    assert(belief_events.array[0].truth.confidence == 0.9, "event has to be there"); //identify
    int returnIndex;
    assert(!Memory_getClosestConcept(&e.sdr, e.sdr_hash, &returnIndex), "a concept doesn't exist yet!");
    Concept *c = Memory_Conceptualize(&e.sdr, e.attention);
    bool conceptWasCreated = false;
    for(int i=0; i<CONCEPTS_MAX; i++)
    {
        if(c == concepts.items[i].address)
        {
            conceptWasCreated = true;
        }
    }
    assert(conceptWasCreated, "Concept should have been created!");
    assert(Memory_FindConceptBySDR(&e.sdr, e.sdr_hash, &returnIndex), "Concept should be found!");
    assert(c == concepts.items[returnIndex].address, "e should match to c!");
    assert(Memory_getClosestConcept(&e.sdr, e.sdr_hash, &returnIndex), "Concept should be found!");
    assert(c == concepts.items[returnIndex].address, "e should match to c!");

    Event e2 = Event_InputEvent(Encode_Term("b"), 
                               EVENT_TYPE_BELIEF, 
                               (Truth) {.frequency = 1, .confidence = 0.9}, 
                               1337);
    e2.attention.priority = 0.3;
    Memory_addEvent(&e2);
    Concept *c2 = Memory_Conceptualize(&e2.sdr, e2.attention);
    assert(Memory_FindConceptBySDR(&e2.sdr, e2.sdr_hash, &returnIndex), "Concept should be found!");
    assert(c2 == concepts.items[returnIndex].address, "e2 should match to c2!");
    assert(Memory_getClosestConcept(&e2.sdr, e2.sdr_hash, &returnIndex), "Concept should be found!");
    assert(c2 == concepts.items[returnIndex].address, "e2 should closest-match to c2!");
    assert(Memory_FindConceptBySDR(&e.sdr, e.sdr_hash, &returnIndex), "Concept should be found!");
    assert(c == concepts.items[returnIndex].address, "e should match to c!");
    assert(Memory_getClosestConcept(&e.sdr, e.sdr_hash, &returnIndex), "Concept should be found!");
    assert(c == concepts.items[returnIndex].address, "e should closest-match to c!");
    printf("<<Memory test successful\n");
}


void ANSNA_Alphabet_Test()
{
    ANSNA_INIT();
    printf(">>ANSNA Alphabet test start\n");
    ANSNA_AddInput(Encode_Term("a"), EVENT_TYPE_BELIEF, ANSNA_DEFAULT_TRUTH, "a");
    for(int i=0; i<50; i++)
    {
        int k=i%10;
        if(i % 3 == 0)
        {
            char c[2] = {'a'+k,0};
            ANSNA_AddInput(Encode_Term(c), EVENT_TYPE_BELIEF, ANSNA_DEFAULT_TRUTH, c);
        }
        ANSNA_Cycles(1);
        printf("TICK\n");
    }
    printf("<<ANSNA Alphabet test successful\n");
}

bool ANSNA_Procedure_Test_Op_executed = false;
void ANSNA_Procedure_Test_Op()
{
    printf("op executed by ANSNA\n");
    ANSNA_Procedure_Test_Op_executed = true;
}
void ANSNA_Procedure_Test()
{
    ANSNA_INIT();
    printf(">>ANSNA Procedure test start\n");
    ANSNA_AddOperation(Encode_Term("op"), ANSNA_Procedure_Test_Op); 
    ANSNA_AddInputBelief(Encode_Term("a"), "a");
    ANSNA_Cycles(10);
    printf("---------------\n"); //++
    ANSNA_AddInputBelief(Encode_Term("op"), "op");
    ANSNA_Cycles(10);
    printf("---------------\n"); //++
    ANSNA_AddInputBelief(Encode_Term("result"), "result");
    ANSNA_Cycles(10);
    printf("---------------\n"); //++
    ANSNA_AddInputBelief(Encode_Term("a"), "a");
    ANSNA_Cycles(10);
    printf("---------------\n"); //++
    ANSNA_AddInputGoal(Encode_Term("result"), "result");
    ANSNA_Cycles(10);
    printf("---------------\n"); //++
    assert(ANSNA_Procedure_Test_Op_executed, "ANSNA should have executed op!");
    printf("<<ANSNA Procedure test successful\n");
}

bool ANSNA_Follow_Test_Left_executed = false;
void ANSNA_Follow_Test_Left()
{
    printf("left executed by ANSNA\n");
    ANSNA_Follow_Test_Left_executed = true;
}
bool ANSNA_Follow_Test_Right_executed = false;
void ANSNA_Follow_Test_Right()
{
    printf("right executed by ANSNA\n");
    ANSNA_Follow_Test_Right_executed = true;
}
void ANSNA_Follow_Test()
{
    OUTPUT = 0;
    ANSNA_INIT();
    printf(">>ANSNA Follow test start\n");
    ANSNA_AddOperation(Encode_Term("op_left"), ANSNA_Follow_Test_Left); 
    ANSNA_AddOperation(Encode_Term("op_right"), ANSNA_Follow_Test_Right); 
    int simsteps = 1000000;
    int LEFT = 0;
    int RIGHT = 1;
    int BALL = RIGHT;
    int score = 0;
    int goods = 0;
    int bads = 0;
    for(int i=0;i<simsteps; i++)
    {
        printf(BALL == LEFT ? "LEFT\n" : "RIGHT\n");
        ANSNA_AddInputBelief(BALL == LEFT ? Encode_Term("ball_left") : Encode_Term("ball_right"), BALL == LEFT ? "ball_left" : "ball_right");
        ANSNA_AddInputGoal(Encode_Term("good_ansna"), "good_ansna");
        if(ANSNA_Follow_Test_Right_executed)
        {
            if(BALL == RIGHT)
            {
                ANSNA_AddInputBelief(Encode_Term("good_ansna"), "good_ansna");
                printf("(ball=%d) good\n",BALL);
                score++;
                goods++;
            }
            else
            {
                //ANSNA_AddInput(Encode_Term("good_ansna"), EVENT_TYPE_BELIEF, (Truth) {.frequency = 0, .confidence = 0.9}, "good_ansna");
                printf("(ball=%d) bad\n",BALL);
                score--;
                bads++;
            }
            ANSNA_Follow_Test_Right_executed = false;
        }
        if(ANSNA_Follow_Test_Left_executed)
        {        
            if(BALL == LEFT)
            {
                ANSNA_AddInputBelief(Encode_Term("good_ansna"), "good_ansna");
                printf("(ball=%d) good\n",BALL);
                score++;
                goods++;
            }
            else
            {
                //ANSNA_AddInput(Encode_Term("good_ansna"), EVENT_TYPE_BELIEF, (Truth) {.frequency = 0, .confidence = 0.9}, "good_ansna");
                printf("(ball=%d) bad\n",BALL);
                score--;
                bads++;
            }
            ANSNA_Follow_Test_Left_executed = false;
        }
        BALL = rand() % 2;
        printf("Score %i step%d=\n", score,i);
        assert(score > -100, "too bad");
        if(score >= 500)
            break;
    }
    printf("<<ANSNA Follow test successful goods=%d bads=%d\n",goods,bads);
}

bool ANSNA_Pong_Left_executed = false;
void ANSNA_Pong_Left()
{
    ANSNA_Pong_Left_executed = true;
}
bool ANSNA_Pong_Right_executed = false;
void ANSNA_Pong_Right()
{
    ANSNA_Pong_Right_executed = true;
}

void ANSNA_Pong()
{
    OUTPUT = 0;
    ANSNA_INIT();
    printf(">>ANSNA Pong start\n");
    ANSNA_AddOperation(Encode_Term("op_left"), ANSNA_Pong_Left); 
    ANSNA_AddOperation(Encode_Term("op_right"), ANSNA_Pong_Right); 
    int szX = 40;
    int szY = 20;
    int ballX = szX/2;
    int ballY = szY/5;
    int batX = 20;
    int batVX = 0;
    int batWidth = 4; //"radius", batWidth from middle to the left and right
    int vX = 1;
    int vY = 1;
    ANSNA_AddInputBelief(Encode_Term("good_boy"),"good_boy");
    ANSNA_AddInputBelief(Encode_Term("ball_right"), "ball_right");
    ANSNA_AddInputBelief(Encode_Term("ball_left"), "ball_left");
    while(1)
    {
        printf("\033[1;1H\033[2J"); //POSIX clear screen
        if(batX < ballX)
        {
            ANSNA_Util_PrintExistingEventNarsese(ANSNA_AddInputBelief(Encode_Term("ball_right"), "ball_right"));
        }
        if(ballX < batX)
        {
            ANSNA_Util_PrintExistingEventNarsese(ANSNA_AddInputBelief(Encode_Term("ball_left"), "ball_left"));
        }
        ANSNA_Util_PrintExistingEventNarsese(ANSNA_AddInputGoal(Encode_Term("good_ansna"), "good_ansna"));
        printf("\n");
        if(ballX <= 0)
        {
            vX = 1;
        }
        if(ballX >= szX-1)
        {
            vX = -1;
        }
        if(ballY <= 0)
        {
            vY = 1;
        }
        if(ballY >= szY-1)
        {
            vY = -1;
        }
        ballX += vX;
        ballY += vY;
        for(int i=0; i<batX-batWidth+1; i++)
        {
            printf(" ");
        }
        for(int i=0; i<batWidth*2-1 ;i++)
        {
            printf("@");
        }
        printf("\n");
        for(int i=0; i<ballY; i++)
        {
            for(int k=0; k<szX; k++)
            {
                printf(" ");
            }
            printf("|\n");
        }
        for(int i=0; i<ballX; i++)
        {
            printf(" ");
        }
        printf("#");
        for(int i=ballX+1; i<szX; i++)
        {
            printf(" ");
        }
        printf("|\n");
        for(int i=ballY+1; i<szY; i++)
        {
            for(int k=0; k<szX; k++)
            {
                printf(" ");
            }
            printf("|\n");
        }
        if(ballY == 0)
        {
            if(abs(ballX-batX) <= batWidth)
            {
                ANSNA_Util_PrintExistingEventNarsese(ANSNA_AddInputBelief(Encode_Term("good_ansna"), "good_ansna"));
                printf("good\n");
            }
            else
            {
                //ANSNA_AddInput(Encode_Term("good_ansna"), EVENT_TYPE_BELIEF, (Truth) {.frequency = 0, .confidence = 0.9}, "good_ansna");
                printf("bad\n");
            }
            ballY = szY/2+rand()%(szY/2);
            ballX = rand()%szX;
        }
        if(ANSNA_Pong_Left_executed)
        {
            ANSNA_Pong_Left_executed = false;
            printf("Exec: op_left\n");
            batVX = -2;
        }
        if(ANSNA_Pong_Right_executed)
        {
            ANSNA_Pong_Right_executed = false;
            printf("Exec: op_right\n");
            batVX = 2;
        }
        batX=MAX(0,MIN(szX-1,batX+batVX*batWidth/2));
        nanosleep((struct timespec[]){{0, 100000000L}}, NULL); //POSIX sleep
    }
}

int main(int argc, char *argv[]) 
{
    //printf("sizeof concept %d\n",(int) sizeof(Concept));
    //exit(0);
    if(argc == 2) //pong
    {
        ANSNA_Pong();
    }
    srand(1337);
    ANSNA_INIT();
    SDR_Test();
    Stamp_Test();
    FIFO_Test();
    PriorityQueue_Test();
    //Table_Test();
    //ANSNA_Alphabet_Test();
    //ANSNA_Procedure_Test();
    ANSNA_Follow_Test();
    Memory_Test();
    printf("All tests ran successfully, if you wish to run examples now, just pass the corresponding parameter:\n");
    printf("ANSNA pong (starts Pong example)\n");
    return 0;
}

