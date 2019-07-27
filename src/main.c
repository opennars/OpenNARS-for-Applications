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
    puts(">>SDR Test Start");
    puts("testing encoding and permutation:");
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
    puts("testing tuples now:");
    SDR mySDR2 = Encode_Term("term2");
    fputs("recons:", stdout);
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
    puts("<<SDR Test successful");
}

void FIFO_Test()
{
    puts(">>FIFO test start");
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
        assert(FIFO_SIZE-i == fifo.array[0][i].stamp.evidentalBase[0], "Item at FIFO position has to be right");
    }
    //now see whether a new item is revised with the correct one:
    int i=3; //revise with item 10, which has occurrence time 10
    int newbase = FIFO_SIZE*2+1;
    Event event2 = (Event) { .sdr = Encode_Term("test"), 
                             .type = EVENT_TYPE_BELIEF, 
                             .truth = {.frequency = 1.0, .confidence = 0.9},
                             .stamp = (Stamp) { .evidentalBase = {newbase} }, 
                             .occurrenceTime = i*10+3 };
    FIFO fifo2 = {0};
    for(int i=0; i<FIFO_SIZE*2; i++)
    {
        FIFO_Add(&event2, &fifo2);
    }
    assert(fifo2.itemsAmount == FIFO_SIZE, "FIFO size differs");
    puts("<<FIFO Test successful");
}

void Stamp_Test()
{
    puts(">>Stamp test start");
    Stamp stamp1 = (Stamp) { .evidentalBase = {1,2} };
    Stamp_print(&stamp1);
    Stamp stamp2 = (Stamp) { .evidentalBase = {2,3,4} };
    Stamp_print(&stamp2);
    Stamp stamp3 = Stamp_make(&stamp1, &stamp2);
    fputs("zipped:", stdout);
    Stamp_print(&stamp3);
    assert(Stamp_checkOverlap(&stamp1,&stamp2) == true, "Stamp should overlap");
    puts("<<Stamp test successful");
}

void PriorityQueue_Test()
{
    puts(">>PriorityQueue test start");
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
    puts("<<PriorityQueue test successful");
}

void Table_Test()
{
    puts(">>Table test start");
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
    puts("<<Table test successful");
}

void Memory_Test()
{
    ANSNA_INIT();
    puts(">>Memory test start");
    Event e = Event_InputEvent(Encode_Term("a"), 
                               EVENT_TYPE_BELIEF, 
                               (Truth) {.frequency = 1, .confidence = 0.9}, 
                               1337);
    Memory_addEvent(&e);
    assert(belief_events.array[0][0].truth.confidence == 0.9, "event has to be there"); //identify
    int returnIndex;
    assert(!Memory_getClosestConcept(&e.sdr, e.sdr_hash, &returnIndex), "a concept doesn't exist yet!");
    Concept *c = Memory_Conceptualize(&e.sdr);
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
    Memory_addEvent(&e2);
    Concept *c2 = Memory_Conceptualize(&e2.sdr);
    Concept_Print(c2);
    assert(Memory_FindConceptBySDR(&e2.sdr, e2.sdr_hash, &returnIndex), "Concept should be found!");
    assert(c2 == concepts.items[returnIndex].address, "e2 should match to c2!");
    assert(Memory_getClosestConcept(&e2.sdr, e2.sdr_hash, &returnIndex), "Concept should be found!");
    assert(c2 == concepts.items[returnIndex].address, "e2 should closest-match to c2!");
    assert(Memory_FindConceptBySDR(&e.sdr, e.sdr_hash, &returnIndex), "Concept should be found!");
    assert(c == concepts.items[returnIndex].address, "e should match to c!");
    assert(Memory_getClosestConcept(&e.sdr, e.sdr_hash, &returnIndex), "Concept should be found!");
    assert(c == concepts.items[returnIndex].address, "e should closest-match to c!");
    puts("<<Memory test successful");
}

void ANSNA_Alphabet_Test()
{
    ANSNA_INIT();
    puts(">>ANSNA Alphabet test start");
    ANSNA_AddInput(Encode_Term("a"), EVENT_TYPE_BELIEF, ANSNA_DEFAULT_TRUTH);
    for(int i=0; i<50; i++)
    {
        int k=i%10;
        if(i % 3 == 0)
        {
            char c[2] = {'a'+k,0};
            ANSNA_AddInput(Encode_Term(c), EVENT_TYPE_BELIEF, ANSNA_DEFAULT_TRUTH);
        }
        ANSNA_Cycles(1);
        puts("TICK");
    }
    puts("<<ANSNA Alphabet test successful");
}

bool ANSNA_Procedure_Test_Op_executed = false;
void ANSNA_Procedure_Test_Op()
{
    puts("op executed by ANSNA");
    ANSNA_Procedure_Test_Op_executed = true;
}
void ANSNA_Procedure_Test()
{
    ANSNA_INIT();
    puts(">>ANSNA Procedure test start");
    ANSNA_AddOperation(Encode_Term("op"), ANSNA_Procedure_Test_Op); 
    ANSNA_AddInputBelief(Encode_Term("a"));
    ANSNA_Cycles(10);
    puts("---------------");
    ANSNA_AddInputBelief(Encode_Term("op"));
    ANSNA_Cycles(10);
    puts("---------------");
    ANSNA_AddInputBelief(Encode_Term("result"));
    ANSNA_Cycles(10);
    puts("---------------");
    ANSNA_AddInputBelief(Encode_Term("a"));
    ANSNA_Cycles(10);
    puts("---------------");
    ANSNA_AddInputGoal(Encode_Term("result"));
    ANSNA_Cycles(10);
    puts("---------------");
    assert(ANSNA_Procedure_Test_Op_executed, "ANSNA should have executed op!");
    puts("<<ANSNA Procedure test successful");
}

bool ANSNA_Follow_Test_Left_executed = false;
void ANSNA_Follow_Test_Left()
{
    puts("left executed by ANSNA");
    ANSNA_Follow_Test_Left_executed = true;
}
bool ANSNA_Follow_Test_Right_executed = false;
void ANSNA_Follow_Test_Right()
{
    puts("right executed by ANSNA");
    ANSNA_Follow_Test_Right_executed = true;
}
void ANSNA_Follow_Test()
{
    OUTPUT = 0;
    ANSNA_INIT();
    puts(">>ANSNA Follow test start");
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
        puts(BALL == LEFT ? "LEFT" : "RIGHT");
        ANSNA_AddInputBelief(BALL == LEFT ? Encode_Term("ball_left") : Encode_Term("ball_right"));
        ANSNA_AddInputGoal(Encode_Term("good_ansna"));
        if(ANSNA_Follow_Test_Right_executed)
        {
            if(BALL == RIGHT)
            {
                ANSNA_AddInputBelief(Encode_Term("good_ansna"));
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
                ANSNA_AddInputBelief(Encode_Term("good_ansna"));
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
        assert(score > -100, "too bad score");
        assert(bads < 500, "too many wrong trials");
        if(score >= 500)
            break;
        ANSNA_Cycles(1000);
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
void ANSNA_Pong(bool useNumericEncoding)
{
    OUTPUT = 0;
    ANSNA_INIT();
    puts(">>ANSNA Pong start");
    ANSNA_AddOperation(Encode_Term("op_left"), ANSNA_Pong_Left); 
    ANSNA_AddOperation(Encode_Term("op_right"), ANSNA_Pong_Right); 
    int szX = 50;
    int szY = 20;
    int ballX = szX/2;
    int ballY = szY/5;
    int batX = 20;
    int batVX = 0;
    int batWidth = 4; //"radius", batWidth from middle to the left and right
    int vX = 1;
    int vY = 1;
    int hits = 0;
    int misses = 0;
    while(1)
    {
        fputs("\033[1;1H\033[2J", stdout); //POSIX clear screen
        for(int i=0; i<batX-batWidth+1; i++)
        {
            fputs(" ", stdout);
        }
        for(int i=0; i<batWidth*2-1 ;i++)
        {
            fputs("@", stdout);
        }
        puts("");
        for(int i=0; i<ballY; i++)
        {
            for(int k=0; k<szX; k++)
            {
                fputs(" ", stdout);
            }
            puts("|");
        }
        for(int i=0; i<ballX; i++)
        {
            fputs(" ", stdout);
        }
        fputs("#", stdout);
        for(int i=ballX+1; i<szX; i++)
        {
            fputs(" ", stdout);
        }
        puts("|");
        for(int i=ballY+1; i<szY; i++)
        {
            for(int k=0; k<szX; k++)
            {
                fputs(" ", stdout);
            }
            puts("|");
        }
        if(useNumericEncoding)
        {
            SDR sdrX = Encode_Scalar(0, 2*szX, szX+(ballX-batX));
            //SDR_PrintWhereTrue(&sdrX);
            ANSNA_AddInputBelief(sdrX);
        }
        else
        {
            if(batX < ballX)
            {
                ANSNA_AddInputBelief(Encode_Term("ball_right"));
            }
            if(ballX < batX)
            {
                ANSNA_AddInputBelief(Encode_Term("ball_left"));
            }
        }
        ANSNA_AddInputGoal(Encode_Term("good_ansna"));
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
        if(ballY == 0)
        {
            if(abs(ballX-batX) <= batWidth)
            {
                ANSNA_AddInputBelief(Encode_Term("good_ansna"));
                puts("good");
                hits++;
            }
            else
            {
                //ANSNA_AddInput(Encode_Term("good_ansna"), EVENT_TYPE_BELIEF, (Truth) {.frequency = 0, .confidence = 0.9}, "good_ansna");
                puts("bad");
                misses++;
            }
        }
        if(ballY == 0 || ballX == 0 || ballX >= szX-1)
        {
            ballY = szY/2+rand()%(szY/2);
            ballX = rand()%szX;
            vX = rand()%2 == 0 ? 1 : -1;
        }
        if(ANSNA_Pong_Left_executed)
        {
            ANSNA_Pong_Left_executed = false;
            puts("Exec: op_left");
            batVX = -2;
        }
        if(ANSNA_Pong_Right_executed)
        {
            ANSNA_Pong_Right_executed = false;
            puts("Exec: op_right");
            batVX = 2;
        }
        batX=MAX(0,MIN(szX-1,batX+batVX*batWidth/2));
        printf("Hits=%d misses=%d ratio=%f\n", hits, misses, (float) (((float) hits) / ((float) misses)));
        nanosleep((struct timespec[]){{0, 100000000L}}, NULL); //POSIX sleep
        //ANSNA_Cycles(10);
    }
}

bool ANSNA_Lightswitch_GotoSwitch_executed = false;
void ANSNA_Lightswitch_GotoSwitch()
{
    ANSNA_Lightswitch_GotoSwitch_executed = true;
    puts("ANSNA invoked goto switch");
}
bool ANSNA_Lightswitch_ActivateSwitch_executed = false;
void ANSNA_Lightswitch_ActivateSwitch()
{
    ANSNA_Lightswitch_ActivateSwitch_executed = true;
    puts("ANSNA invoked activate switch");
}
void ANSNA_Multistep_Test()
{
    MOTOR_BABBLING_CHANCE = 0;
    puts(">>ANSNA Multistep test start");
    OUTPUT = 0;
    ANSNA_INIT();
    ANSNA_AddOperation(Encode_Term("op_goto_switch"), ANSNA_Lightswitch_GotoSwitch); 
    ANSNA_AddOperation(Encode_Term("op_activate_switch"), ANSNA_Lightswitch_ActivateSwitch); 
    for(int i=0; i<5; i++)
    {
        ANSNA_AddInputBelief(Encode_Term("start_at"));
        ANSNA_AddInputBelief(Encode_Term("op_goto_switch"));
        ANSNA_Cycles(10);
        ANSNA_AddInputBelief(Encode_Term("switch_at"));
        ANSNA_AddInputBelief(Encode_Term("op_activate_switch"));
        ANSNA_AddInputBelief(Encode_Term("switch_active"));
        ANSNA_Cycles(5);
        ANSNA_AddInputBelief(Encode_Term("light_active"));
        ANSNA_Cycles(100);
    }
    ANSNA_Cycles(1000);
    ANSNA_AddInputBelief(Encode_Term("start_at"));
    ANSNA_AddInputGoal(Encode_Term("light_active"));
    ANSNA_Cycles(100);
    assert(ANSNA_Lightswitch_GotoSwitch_executed && !ANSNA_Lightswitch_ActivateSwitch_executed, "ANSNA needs to go to the switch first");
    ANSNA_Lightswitch_GotoSwitch_executed = false;
    puts("ANSNA arrived at the switch");
    ANSNA_AddInputBelief(Encode_Term("switch_at"));
    ANSNA_AddInputGoal(Encode_Term("light_active"));
    assert(!ANSNA_Lightswitch_GotoSwitch_executed && ANSNA_Lightswitch_ActivateSwitch_executed, "ANSNA needs to activate the switch");
    ANSNA_Lightswitch_ActivateSwitch_executed = false;
    puts("<<ANSNA Multistep test successful");
}
void ANSNA_Multistep2_Test()
{
    MOTOR_BABBLING_CHANCE = 0;
    puts(">>ANSNA Multistep2 test start");
    OUTPUT = 0;
    ANSNA_INIT();
    ANSNA_AddOperation(Encode_Term("op_goto_switch"), ANSNA_Lightswitch_GotoSwitch); 
    ANSNA_AddOperation(Encode_Term("op_activate_switch"), ANSNA_Lightswitch_ActivateSwitch); 
    for(int i=0; i<50; i++)
    {
        ANSNA_AddInputBelief(Encode_Term("start_at"));
        ANSNA_AddInputBelief(Encode_Term("op_goto_switch"));
        ANSNA_Cycles(10);
        ANSNA_AddInputBelief(Encode_Term("switch_at"));
        ANSNA_Cycles(100);
    }
    ANSNA_Cycles(1000);
    for(int i=0; i<50; i++)
    {
        ANSNA_AddInputBelief(Encode_Term("switch_at"));
        ANSNA_AddInputBelief(Encode_Term("op_activate_switch"));
        ANSNA_AddInputBelief(Encode_Term("switch_active"));
        ANSNA_Cycles(5);
        ANSNA_AddInputBelief(Encode_Term("light_active"));
        ANSNA_Cycles(100);
    }
    ANSNA_Cycles(1000);
    ANSNA_AddInputBelief(Encode_Term("start_at"));
    ANSNA_AddInputGoal(Encode_Term("light_active"));
    ANSNA_Cycles(100);
    assert(ANSNA_Lightswitch_GotoSwitch_executed && !ANSNA_Lightswitch_ActivateSwitch_executed, "ANSNA needs to go to the switch first (2)");
    ANSNA_Lightswitch_GotoSwitch_executed = false;
    puts("ANSNA arrived at the switch");
    ANSNA_AddInputBelief(Encode_Term("switch_at"));
    ANSNA_AddInputGoal(Encode_Term("light_active"));
    assert(!ANSNA_Lightswitch_GotoSwitch_executed && ANSNA_Lightswitch_ActivateSwitch_executed, "ANSNA needs to activate the switch (2)");
    ANSNA_Lightswitch_ActivateSwitch_executed = false;
    puts("<<ANSNA Multistep2 test successful");
}

int main(int argc, char *argv[]) 
{
    //printf("sizeof concept %d\n",(int) sizeof(Concept));
    //exit(0);
    if(argc == 2) //pong
    {
        if(!strcmp(argv[1],"pong"))
        {
            ANSNA_Pong(false);
        }
        if(!strcmp(argv[1],"numeric-pong"))
        {
            ANSNA_Pong(true);
        }
    }
    srand(1337);
    ANSNA_INIT();
    SDR_Test();
    Stamp_Test();
    FIFO_Test();
    PriorityQueue_Test();
    Table_Test();
    ANSNA_Alphabet_Test();
    ANSNA_Procedure_Test();
    Memory_Test();
    ANSNA_Follow_Test();
    ANSNA_Multistep_Test();
    ANSNA_Multistep2_Test();
    puts("\nAll tests ran successfully, if you wish to run examples now, just pass the corresponding parameter:");
    puts("ANSNA pong (starts Pong example)");
    puts("ANSNA numeric-pong (starts Pong example with numeric input)");
    return 0;
}

