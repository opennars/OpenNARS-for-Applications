#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "Term.h"
#include "Memory.h"
#include "Encode.h"
#include "MSC.h"

void FIFO_Test()
{
    puts(">>FIFO test start");
    FIFO fifo = {0};
    //First, evaluate whether the fifo works, not leading to overflow
    for(int i=FIFO_SIZE*2; i>=1; i--) //"rolling over" once by adding a k*FIFO_Size items
    {
        Event event1 = { .term = Encode_Term("test"), 
                         .type = EVENT_TYPE_BELIEF, 
                         .truth = { .frequency = 1.0, .confidence = 0.9 },
                         .stamp = { .evidentalBase = { i } }, 
                         .occurrenceTime = FIFO_SIZE*2 - i*10 };
        FIFO_Add(&event1, &fifo);
    }
    for(int i=0; i<FIFO_SIZE; i++)
    {
        assert(FIFO_SIZE-i == fifo.array[0][i].stamp.evidentalBase[0], "Item at FIFO position has to be right");
    }
    //now see whether a new item is revised with the correct one:
    int i=3; //revise with item 10, which has occurrence time 10
    int newbase = FIFO_SIZE*2+1;
    Event event2 = { .term = Encode_Term("test"), 
                     .type = EVENT_TYPE_BELIEF, 
                     .truth = { .frequency = 1.0, .confidence = 0.9 },
                     .stamp = { .evidentalBase = { newbase } }, 
                     .occurrenceTime = i*10+3 };
    FIFO fifo2 = {0};
    for(int i=0; i<FIFO_SIZE*2; i++)
    {
        Term zero = (Term) {0};
        FIFO_Add(&event2, &fifo2);
        if(i < FIFO_SIZE && i < MAX_SEQUENCE_LEN)
        {
            char buf[100]; 
            Event *ev = FIFO_GetKthNewestSequence(&fifo2, 0, i);
            sprintf(buf,"This event Term is not allowed to be zero, sequence length=%d\n",i+1);
            assert(!Term_Equal(&zero, &ev->term),buf);
        }
    }
    assert(fifo2.itemsAmount == FIFO_SIZE, "FIFO size differs");
    puts("<<FIFO Test successful");
}

void Stamp_Test()
{
    puts(">>Stamp test start");
    Stamp stamp1 = { .evidentalBase = {1,2} };
    Stamp_print(&stamp1);
    Stamp stamp2 = { .evidentalBase = {2,3,4} };
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
            assert(evictions>0 || feedback.evictedItem.priority == ((double)(1.0/((double) (n_items*2)))), "the evicted item has to be the lowest priority one");
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
        Implication imp = { .term = Encode_Term("test"), 
                            .truth = { .frequency = 1.0, .confidence = 1.0/((double)(i+1)) },
                            .stamp = { .evidentalBase = { i } },
                            .occurrenceTimeOffset = 10 };
        Table_Add(&table, &imp);
    }
    for(int i=0; i<TABLE_SIZE; i++)
    {
        assert(i+1 == table.array[i].stamp.evidentalBase[0], "Item at table position has to be right");
    }
    Implication imp = { .term = Encode_Term("test"), 
                        .truth = { .frequency = 1.0, .confidence = 0.9},
                        .stamp = { .evidentalBase = { TABLE_SIZE*2+1 } },
                        .occurrenceTimeOffset = 10 };
    assert(table.array[0].truth.confidence==0.5, "The highest confidence one should be the first.");
    Table_AddAndRevise(&table, &imp, "");
    assert(table.array[0].truth.confidence>0.5, "The revision result should be more confident than the table element that existed.");
    puts("<<Table test successful");
}

void Memory_Test()
{
    MSC_INIT();
    puts(">>Memory test start");
    Event e = Event_InputEvent(Encode_Term("a"), 
                               EVENT_TYPE_BELIEF, 
                               (Truth) { .frequency = 1, .confidence = 0.9 }, 
                               1337);
    Memory_addEvent(&e);
    assert(belief_events.array[0][0].truth.confidence == (double) 0.9, "event has to be there"); //identify
    int returnIndex;
    assert(!Memory_FindConceptByTerm(&e.term, /*e.term_hash, */ &returnIndex), "a concept doesn't exist yet!");
    Memory_Conceptualize(&e.term);
    int concept_i;
    assert(Memory_FindConceptByTerm(&e.term, /*Term_Hash(&e.term),*/ &concept_i), "Concept should have been created!");
    Concept *c = concepts.items[concept_i].address;
    assert(Memory_FindConceptByTerm(&e.term, /*e.term_hash,*/ &returnIndex), "Concept should be found!");
    assert(c == concepts.items[returnIndex].address, "e should match to c!");
    assert(Memory_FindConceptByTerm(&e.term, /*e.term_hash,*/ &returnIndex), "Concept should be found!");
    assert(c == concepts.items[returnIndex].address, "e should match to c!");
    Event e2 = Event_InputEvent(Encode_Term("b"), 
                               EVENT_TYPE_BELIEF, 
                               (Truth) { .frequency = 1, .confidence = 0.9 }, 
                               1337);
    Memory_addEvent(&e2);
    Memory_Conceptualize(&e2.term);
    assert(Memory_FindConceptByTerm(&e2.term, /*Term_Hash(&e2.term),*/ &concept_i), "Concept should have been created!");
    Concept *c2 = concepts.items[concept_i].address;
    Concept_Print(c2);
    assert(Memory_FindConceptByTerm(&e2.term, /*e2.term_hash,*/ &returnIndex), "Concept should be found!");
    assert(c2 == concepts.items[returnIndex].address, "e2 should closest-match to c2!");
    assert(Memory_FindConceptByTerm(&e.term, /*e.term_hash,*/ &returnIndex), "Concept should be found!");
    assert(c == concepts.items[returnIndex].address, "e should closest-match to c!");
    puts("<<Memory test successful");
}

void MSC_Alphabet_Test()
{
    MSC_INIT();
    puts(">>MSC Alphabet test start");
    MSC_AddInput(Encode_Term("a"), EVENT_TYPE_BELIEF, MSC_DEFAULT_TRUTH,0);
    for(int i=0; i<50; i++)
    {
        int k=i%10;
        if(i % 3 == 0)
        {
            char c[2] = {'a'+k,0};
            MSC_AddInput(Encode_Term(c), EVENT_TYPE_BELIEF, MSC_DEFAULT_TRUTH,0);
        }
        MSC_Cycles(1);
        puts("TICK");
    }
    puts("<<MSC Alphabet test successful");
}

bool MSC_Procedure_Test_Op_executed = false;
void MSC_Procedure_Test_Op()
{
    puts("op executed by MSC");
    MSC_Procedure_Test_Op_executed = true;
}
void MSC_Procedure_Test()
{
    MSC_INIT();
    puts(">>MSC Procedure test start");
    MSC_AddOperation(Encode_Term("op"), MSC_Procedure_Test_Op); 
    MSC_AddInputBelief(Encode_Term("a"), 0);
    MSC_Cycles(1);
    puts("---------------");
    MSC_AddInputBelief(Encode_Term("op"), 1);
    MSC_Cycles(1);
    puts("---------------");
    MSC_AddInputBelief(Encode_Term("result"), 0);
    MSC_Cycles(1);
    puts("---------------");
    MSC_AddInputBelief(Encode_Term("a"), 0);
    MSC_Cycles(1);
    puts("---------------");
    MSC_AddInputGoal(Encode_Term("result"));
    MSC_Cycles(1);
    puts("---------------");
    assert(MSC_Procedure_Test_Op_executed, "MSC should have executed op!");
    puts("<<MSC Procedure test successful");
}

bool MSC_Follow_Test_Left_executed = false;
void MSC_Follow_Test_Left()
{
    puts("left executed by MSC");
    MSC_Follow_Test_Left_executed = true;
}
bool MSC_Follow_Test_Right_executed = false;
void MSC_Follow_Test_Right()
{
    puts("right executed by MSC");
    MSC_Follow_Test_Right_executed = true;
}
void MSC_Follow_Test()
{
    OUTPUT = 0;
    MSC_INIT();
    puts(">>MSC Follow test start");
    MSC_AddOperation(Encode_Term("op_left"), MSC_Follow_Test_Left); 
    MSC_AddOperation(Encode_Term("op_right"), MSC_Follow_Test_Right); 
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
        MSC_AddInputBelief(BALL == LEFT ? Encode_Term("ball_left") : Encode_Term("ball_right"), 0);
        MSC_AddInputGoal(Encode_Term("good_msc"));
        if(MSC_Follow_Test_Right_executed)
        {
            if(BALL == RIGHT)
            {
                MSC_AddInputBelief(Encode_Term("good_msc"), 0);
                printf("(ball=%d) good\n",BALL);
                score++;
                goods++;
            }
            else
            {
                //MSC_AddInput(Encode_Term("good_msc"), EVENT_TYPE_BELIEF, (Truth) {.frequency = 0, .confidence = 0.9}, "good_msc");
                printf("(ball=%d) bad\n",BALL);
                score--;
                bads++;
            }
            MSC_Follow_Test_Right_executed = false;
        }
        if(MSC_Follow_Test_Left_executed)
        {        
            if(BALL == LEFT)
            {
                MSC_AddInputBelief(Encode_Term("good_msc"), 0);
                printf("(ball=%d) good\n",BALL);
                score++;
                goods++;
            }
            else
            {
                //MSC_AddInput(Encode_Term("good_msc"), EVENT_TYPE_BELIEF, (Truth) {.frequency = 0, .confidence = 0.9}, "good_msc");
                printf("(ball=%d) bad\n",BALL);
                score--;
                bads++;
            }
            MSC_Follow_Test_Left_executed = false;
        }
        BALL = rand() % 2;
        printf("Score %i step%d=\n", score,i);
        assert(score > -100, "too bad score");
        assert(bads < 500, "too many wrong trials");
        if(score >= 500)
            break;
        MSC_Cycles(10);
    }
    printf("<<MSC Follow test successful goods=%d bads=%d\n",goods,bads);
}

bool MSC_Pong_Left_executed = false;
void MSC_Pong_Left()
{
    MSC_Pong_Left_executed = true;
}
bool MSC_Pong_Right_executed = false;
void MSC_Pong_Right()
{
    MSC_Pong_Right_executed = true;
}
bool MSC_Pong_Stop_executed = false;
void MSC_Pong_Stop()
{
    MSC_Pong_Stop_executed = true;
}
void MSC_Pong2()
{
    OUTPUT = 0;
    MSC_INIT();
    puts(">>MSC Pong start");
    MSC_AddOperation(Encode_Term("op_left"), MSC_Pong_Left); 
    MSC_AddOperation(Encode_Term("op_right"), MSC_Pong_Right); 
    MSC_AddOperation(Encode_Term("op_stop"), MSC_Pong_Stop); 
    int szX = 50;
    int szY = 20;
    int ballX = szX/2;
    int ballY = szY/5;
    int batX = 20;
    int batVX = 0;
    int batWidth = 6; //"radius", batWidth from middle to the left and right
    int vX = 1;
    int vY = 1;
    int hits = 0;
    int misses = 0;
    int t=0;
    while(1)
    {
        t++;
        //if(t%10000 == 0)
        //    getchar();
        fputs("\033[1;1H\033[2J", stdout); //POSIX clear screen
        for(int i=0; i<batX-batWidth+1; i++)
        {
            fputs(" ", stdout);
        }
        for(int i=0; i<batWidth*2-1+MIN(0,batX) ;i++)
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
        if(batX <= ballX - batWidth)
        {
            MSC_AddInputBelief(Encode_Term("ball_right"), 0);
        }
        else
        if(ballX + batWidth < batX)
        {
            MSC_AddInputBelief(Encode_Term("ball_left"), 0);
        }
        else
        {
            MSC_AddInputBelief(Encode_Term("ball_equal"), 0);
        }
        MSC_AddInputGoal(Encode_Term("good_msc"));
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
        if(t%2 == -1)
        {
            ballX += vX;
        }
        ballY += vY;
        if(ballY == 0)
        {
            if(abs(ballX-batX) <= batWidth)
            {
                MSC_AddInputBelief(Encode_Term("good_msc"), 0);
                puts("good");
                hits++;
            }
            else
            {
                //MSC_AddInput(Encode_Term("good_msc"), EVENT_TYPE_BELIEF, (Truth) {.frequency = 0, .confidence = 0.9});
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
        if(MSC_Pong_Left_executed)
        {
            MSC_Pong_Left_executed = false;
            puts("Exec: op_left");
            batVX = -3;
        }
        if(MSC_Pong_Right_executed)
        {
            MSC_Pong_Right_executed = false;
            puts("Exec: op_right");
            batVX = 3;
        }
        if(MSC_Pong_Stop_executed)
        {
            MSC_Pong_Stop_executed = false;
            puts("Exec: op_stop");
            batVX = 0;
        }
        batX=MAX(-batWidth*2,MIN(szX-1+batWidth,batX+batVX*batWidth/2));
        printf("Hits=%d misses=%d ratio=%f time=%ld\n", hits, misses, (float) (((float) hits) / ((float) misses)), currentTime);
        nanosleep((struct timespec[]){{0, 20000000L}}, NULL); //POSIX sleep
        //MSC_Cycles(10);
    }
}
//int t=0;
void MSC_Pong()
{
    OUTPUT = 0;
    MSC_INIT();
    puts(">>MSC Pong start");
    MSC_AddOperation(Encode_Term("op_left"), MSC_Pong_Left); 
    MSC_AddOperation(Encode_Term("op_right"), MSC_Pong_Right); 
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
        //t++;
        //if(t%10000 == 0)
        //    getchar();
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
        if(batX < ballX)
        {
            MSC_AddInputBelief(Encode_Term("ball_right"), 0);
        }
        if(ballX < batX)
        {
            MSC_AddInputBelief(Encode_Term("ball_left"), 0);
        }
        MSC_AddInputGoal(Encode_Term("good_msc"));
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
                MSC_AddInputBelief(Encode_Term("good_msc"), 0);
                puts("good");
                hits++;
            }
            else
            {
                //MSC_AddInput(Encode_Term("good_msc"), EVENT_TYPE_BELIEF, (Truth) {.frequency = 0, .confidence = 0.9}, "good_msc");
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
        if(MSC_Pong_Left_executed)
        {
            MSC_Pong_Left_executed = false;
            puts("Exec: op_left");
            batVX = -2;
        }
        if(MSC_Pong_Right_executed)
        {
            MSC_Pong_Right_executed = false;
            puts("Exec: op_right");
            batVX = 2;
        }
        batX=MAX(0,MIN(szX-1,batX+batVX*batWidth/2));
        printf("Hits=%d misses=%d ratio=%f time=%ld\n", hits, misses, (float) (((float) hits) / ((float) misses)), currentTime);
        nanosleep((struct timespec[]){{0, 20000000L}}, NULL); //POSIX sleep
        //MSC_Cycles(10);
    }
}

bool MSC_Lightswitch_GotoSwitch_executed = false;
void MSC_Lightswitch_GotoSwitch()
{
    MSC_Lightswitch_GotoSwitch_executed = true;
    puts("MSC invoked goto switch");
}
bool MSC_Lightswitch_ActivateSwitch_executed = false;
void MSC_Lightswitch_ActivateSwitch()
{
    MSC_Lightswitch_ActivateSwitch_executed = true;
    puts("MSC invoked activate switch");
}
void MSC_Multistep_Test()
{
    MOTOR_BABBLING_CHANCE = 0;
    puts(">>MSC Multistep test start");
    OUTPUT = 0;
    MSC_INIT();
    MSC_AddOperation(Encode_Term("op_goto_switch"), MSC_Lightswitch_GotoSwitch); 
    MSC_AddOperation(Encode_Term("op_activate_switch"), MSC_Lightswitch_ActivateSwitch); 
    for(int i=0; i<5; i++)
    {
        MSC_AddInputBelief(Encode_Term("start_at"), 0);
        MSC_AddInputBelief(Encode_Term("op_goto_switch"), 1);
        MSC_Cycles(1);
        MSC_AddInputBelief(Encode_Term("switch_at"), 0);
        MSC_AddInputBelief(Encode_Term("op_activate_switch"), 2);
        MSC_AddInputBelief(Encode_Term("switch_active"), 0);
        MSC_Cycles(1);
        MSC_AddInputBelief(Encode_Term("light_active"), 0);
        MSC_Cycles(10);
    }
    MSC_Cycles(10);
    MSC_AddInputBelief(Encode_Term("start_at"), 0);
    MSC_AddInputGoal(Encode_Term("light_active"));
    MSC_Cycles(10);
    assert(MSC_Lightswitch_GotoSwitch_executed && !MSC_Lightswitch_ActivateSwitch_executed, "MSC needs to go to the switch first");
    MSC_Lightswitch_GotoSwitch_executed = false;
    puts("MSC arrived at the switch");
    MSC_AddInputBelief(Encode_Term("switch_at"), 0);
    MSC_AddInputGoal(Encode_Term("light_active"));
    assert(!MSC_Lightswitch_GotoSwitch_executed && MSC_Lightswitch_ActivateSwitch_executed, "MSC needs to activate the switch");
    MSC_Lightswitch_ActivateSwitch_executed = false;
    puts("<<MSC Multistep test successful");
}
void MSC_Multistep2_Test()
{
    MOTOR_BABBLING_CHANCE = 0;
    puts(">>MSC Multistep2 test start");
    OUTPUT = 0;
    MSC_INIT();
    MSC_AddOperation(Encode_Term("op_goto_switch"), MSC_Lightswitch_GotoSwitch); 
    MSC_AddOperation(Encode_Term("op_activate_switch"), MSC_Lightswitch_ActivateSwitch); 
    for(int i=0; i<5; i++)
    {
        MSC_AddInputBelief(Encode_Term("start_at"), 0);
        MSC_AddInputBelief(Encode_Term("op_goto_switch"), 1);
        MSC_Cycles(1);
        MSC_AddInputBelief(Encode_Term("switch_at"), 0);
        MSC_Cycles(10);
    }
    MSC_Cycles(1000);
    for(int i=0; i<5; i++)
    {
        MSC_AddInputBelief(Encode_Term("switch_at"), 0);
        MSC_AddInputBelief(Encode_Term("op_activate_switch"), 2);
        MSC_AddInputBelief(Encode_Term("switch_active"), 0);
        MSC_Cycles(1);
        MSC_AddInputBelief(Encode_Term("light_active"), 0);
        MSC_Cycles(10);
    }
    MSC_Cycles(10);
    MSC_AddInputBelief(Encode_Term("start_at"), 0);
    MSC_AddInputGoal(Encode_Term("light_active"));
    MSC_Cycles(10);
    assert(MSC_Lightswitch_GotoSwitch_executed && !MSC_Lightswitch_ActivateSwitch_executed, "MSC needs to go to the switch first (2)");
    MSC_Lightswitch_GotoSwitch_executed = false;
    puts("MSC arrived at the switch");
    MSC_AddInputBelief(Encode_Term("switch_at"), 0);
    MSC_AddInputGoal(Encode_Term("light_active"));
    assert(!MSC_Lightswitch_GotoSwitch_executed && MSC_Lightswitch_ActivateSwitch_executed, "MSC needs to activate the switch (2)");
    MSC_Lightswitch_ActivateSwitch_executed = false;
    puts("<<MSC Multistep2 test successful");
}

static bool goto_s0 = false;
static bool goto_s1 = false;
static bool goto_s2 = false;
static bool goto_s3 = false;
static bool goto_l0 = false;
static bool goto_l1 = false;
static bool activate = false;
static bool deactivate = false;
void MSC_TestChamber_goto_s0()
{
    goto_s0 = true;
    puts("MSC goto s0");
}
void MSC_TestChamber_goto_s1()
{
    goto_s1 = true;
    puts("MSC goto s1");
}
void MSC_TestChamber_goto_s2()
{
    goto_s2 = true;
    puts("MSC goto s2");
}
void MSC_TestChamber_goto_s3()
{
    goto_s3 = true;
    puts("MSC goto s3");
}
void MSC_TestChamber_goto_l0()
{
    goto_l0 = true;
    puts("MSC goto l0");
}
void MSC_TestChamber_goto_l1()
{
    goto_l1 = true;
    puts("MSC goto l1");
}
void MSC_TestChamber_activate()
{
    activate = true;
    puts("MSC activate");
}
void MSC_TestChamber_deactivate()
{
    deactivate = true;
    puts("MSC deactivate");
}
void MSC_TestChamber()
{
    TRUTH_PROJECTION_DECAY = 0.9; //precise timing isn't so important in this domain, so projection decay can be higher
    ANTICIPATION_CONFIDENCE = 0.3; //neg. evidence accumulation can be stronger
    OUTPUT = 0;
    MSC_INIT();
    MOTOR_BABBLING_CHANCE = 0;
    MSC_AddOperation(Encode_Term("op_goto_s0"), MSC_TestChamber_goto_s0); 
    MSC_AddOperation(Encode_Term("op_goto_s1"), MSC_TestChamber_goto_s1); 
    MSC_AddOperation(Encode_Term("op_goto_s2"), MSC_TestChamber_goto_s2); 
    MSC_AddOperation(Encode_Term("op_goto_s3"), MSC_TestChamber_goto_s3); 
    MSC_AddOperation(Encode_Term("op_goto_l0"), MSC_TestChamber_goto_l0); 
    MSC_AddOperation(Encode_Term("op_goto_l1"), MSC_TestChamber_goto_l1); 
    MSC_AddOperation(Encode_Term("op_activate"), MSC_TestChamber_activate); 
    MSC_AddOperation(Encode_Term("op_deactivate"), MSC_TestChamber_deactivate); 
    int size = 7;
    char world[7][13] = { "_________    ",
                          "| l0  s2| s1 ",
                          "| (0)-0 |-0  ",
                          "|_______|    ",
                          "          s0 ",
                          " l1  s3   .< ",
                          " (0)-0       " };
    //positions:
    const int pos_s0 = 0;
    const int pos_s1 = 1;
    const int pos_s2 = 2;
    const int pos_s3 = 3;
    const int pos_l0 = 4;
    const int pos_l1 = 5;
    //states:
    int pos = pos_s0; //agent position
    bool s1 = false;
    bool s2 = false;
    bool s3 = false;
    bool l0 = false;
    bool l1 = false;
    bool door = false; //door closed
    puts("at_s0");
    MSC_AddInputBelief(Encode_Term("at_s0"), 0);
    char lastchar = 'a';
    while(1)
    {
        //movement
        bool goto_executed = goto_l0 || goto_l1 || goto_s0 || goto_s1 || goto_s2 || goto_s3;
        if(goto_s0)
        {
            pos = pos_s0;
        }
        else
        if(goto_s1)
        {
            pos = pos_s1;
        }
        else
        if(goto_s3)
        {
            pos = pos_s3;
        }
        else
        if(goto_l1)
        {
            pos = pos_l1;
        }
        else
        if(goto_l0 && door)
        {
            pos = pos_l0;
        }
        else
        if(goto_s2 && door)
        {
            pos = pos_s2;
        }
        if(goto_executed)
        {
            activate = deactivate = goto_l0 = goto_l1 = goto_s0 = goto_s1 = goto_s2 = goto_s3 = false;
        }
        //manipulation
        if(pos == pos_s1 && deactivate)
        {
            s1 = false;
            puts("s1_is_0.");
            MSC_AddInputBelief(Encode_Term("s1_is_0"), 0);
            //s1 also closes the door:
            door = false;
            puts("door_is_closed.");
            MSC_AddInputBelief(Encode_Term("door_is_closed"), 0);
        }
        else
        if(pos == pos_s2 && deactivate)
        {
            s2 = false;
            puts("s2_is_0.");
            MSC_AddInputBelief(Encode_Term("s2_is_0"), 0);
            //s2 also deactivates l0:
            l0 = false;
            puts("l0_is_0.");
            MSC_AddInputBelief(Encode_Term("l0_is_0"), 0);
        }
        else
        if(pos == pos_s3 && deactivate)
        {
            s3 = false;
            puts("s3_is_0.");
            MSC_AddInputBelief(Encode_Term("s3_is_0"), 0);
            //s3 also deactivates l1
            l1 = false;
            puts("l1_is_0.");
            MSC_AddInputBelief(Encode_Term("l1_is_0"), 0);
        }
        else
        if(pos == pos_s1 && activate)
        {
            s1 = true;
            puts("s1_is_1.");
            MSC_AddInputBelief(Encode_Term("s1_is_1"), 0);
            //s1 also opens the door:
            door = true;
            puts("door_is_open.");
            MSC_AddInputBelief(Encode_Term("door_is_open"), 0);
        }
        else
        if(pos == pos_s2 && activate)
        {
            s2 = true;
            puts("s2_is_1.");
            MSC_AddInputBelief(Encode_Term("s2_is_1"), 0);
            //s2 also activates l0:
            l0 = true;
            puts("l0_is_1.");
            MSC_AddInputBelief(Encode_Term("l0_is_1"), 0);
        }
        else
        if(pos == pos_s3 && activate)
        {
            s3 = true;
            puts("s3_is_1.");
            MSC_AddInputBelief(Encode_Term("s3_is_1"), 0);
            //s3 also activates l1
            l1 = true;
            puts("l1_is_1.");
            MSC_AddInputBelief(Encode_Term("l1_is_1"), 0);
        }
        activate = deactivate = goto_l0 = goto_l1 = goto_s0 = goto_s1 = goto_s2 = goto_s3 = false;
        //inform MSC about current location
        if(pos == pos_s0)
        {
            puts("at_s0.");
            MSC_AddInputBelief(Encode_Term("at_s0"), 0);
        }
        if(pos == pos_s1)
        {
            puts("at_s1.");
            MSC_AddInputBelief(Encode_Term("at_s1"), 0);
        }
        if(pos == pos_s2)
        {
            puts("at_s2.");
            MSC_AddInputBelief(Encode_Term("at_s2"), 0);
        }
        if(pos == pos_s3)
        {
            puts("at_s3.");
            MSC_AddInputBelief(Encode_Term("at_s3"), 0);
        }
        if(pos == pos_l0)
        {
            puts("at_l0.");
            MSC_AddInputBelief(Encode_Term("at_l0"), 0);
        }
        if(pos == pos_l1)
        {
            puts("at_l1.");
            MSC_AddInputBelief(Encode_Term("at_l1"), 0);
        }
        //change char array to draw:
        world[6][6] = world[6][0] = world[5][11] = world[2][11] = world[2][7] = world[2][1] = ' ';
        if(pos == pos_s3)
        {
             world[6][6] = '<';
        }
        if(pos == pos_l1)
        {
             world[6][0] = '<';
        }
        if(pos == pos_s0)
        {
             world[5][11] = '<';
        }
        if(pos == pos_s1)
        {
             world[2][11] = '<';
        }
        if(pos == pos_s2)
        {
             world[2][7] = '<';
        }
        if(pos == pos_l0)
        {
             world[2][1] = '<';
        }
        if(l0)
        {
            world[2][3] = '1';
        }
        else
        {
            world[2][3] = '0';
        }
        if(s2)
        {
            world[2][6] = '1';
        }
        else
        {
            world[2][6] = '0';
        }
        if(s1)
        {
            world[2][10] = '1';
        }
        else
        {
            world[2][10] = '0';
        }
        if(l1)
        {
            world[6][2] = '1';
        }
        else
        {
            world[6][2] = '0';
        }
        if(s3)
        {
            world[6][5] = '1';
        }
        else
        {
            world[6][5] = '0';
        }
        if(door)
        {
            world[2][8] = ' ';
        }
        else
        {
            world[2][8] = '|';
        }
        fputs("\033[1;1H\033[2J", stdout); //POSIX clear screen
        puts("\n---------------\nNew iteration\nCommands:");
        puts("a ... goto s0");
        puts("b ... goto s1");
        puts("c ... goto s2");
        puts("d ... goto s3");
        puts("e ... goto l0");
        puts("f ... goto l1");
        puts("g ... activate");
        puts("h ... deactivate");
        puts("i ... door should be open!");
        puts("j ... door should be closed!");
        puts("k ... s1 should be 1!");
        puts("l ... s1 should be 0!");
        puts("m ... s2 should be 1!");
        puts("n ... s2 should be 0!");
        puts("o ... s3 should be 1!");
        puts("p ... s3 should be 0!");
        puts("q ... l0 should be 1!");
        puts("r ... l0 should be 0!");
        puts("s ... l1 should be 1!");
        puts("t ... l1 should be 0!");
        puts("u ... you should be at s0!");
        puts("v ... you should be at s1!");
        puts("w ... you should be at s2!");
        puts("x ... you should be at s3!");
        puts("y ... you should be at l0!");
        puts("z ... you should be at l1!");
        puts("other ... Next timestep (same command)\n");
        for(int i=0; i<size; i++)
        {
            for(int j=0; j<13; j++)
            {
                putchar(world[i][j]);
            }
            puts("");
        }
        puts("\nCommand:");
        char c = getchar();
        if(!(c >= 'a' && c<='z'))
        {
            c = lastchar;
        }
        lastchar = c;
        if(c == 'a')
        {
            goto_s0 = true;
            puts("op_goto_s0.");
            MSC_AddInputBelief(Encode_Term("op_goto_s0"), 1);
        }
        if(c == 'b')
        {
            goto_s1 = true;
            puts("op_goto_s1.");
            MSC_AddInputBelief(Encode_Term("op_goto_s1"), 2);
        }
        if(c == 'c')
        {
            goto_s2 = true;
            puts("op_goto_s2.");
            MSC_AddInputBelief(Encode_Term("op_goto_s2"), 3);
        }
        if(c == 'd')
        {
            goto_s3 = true;
            puts("op_goto_s3.");
            MSC_AddInputBelief(Encode_Term("op_goto_s3"), 4);
        }
        if(c == 'e')
        {
            goto_l0 = true;
            puts("op_goto_l0.");
            MSC_AddInputBelief(Encode_Term("op_goto_l0"), 5);
        }
        if(c == 'f')
        {
            goto_l1 = true;
            puts("op_goto_l1.");
            MSC_AddInputBelief(Encode_Term("op_goto_l1"), 6);
        }
        if(c == 'g')
        {
            activate = true;
            puts("op_activate.");
            MSC_AddInputBelief(Encode_Term("op_activate"), 7);
        }
        if(c == 'h')
        {
            deactivate = true;
            puts("op_deactivate.");
            MSC_AddInputBelief(Encode_Term("op_deactivate"), 8);
        }
        if(c == 'i')
        {
            puts("door_is_open!");
            MSC_AddInputGoal(Encode_Term("door_is_open"));
            //door should be open
        }
        if(c == 'j')
        {
            puts("door_is_closed!");
            MSC_AddInputGoal(Encode_Term("door_is_closed"));
            //door should be closed
        }
        if(c == 'k')
        {
            puts("s1_is_1!");
            MSC_AddInputGoal(Encode_Term("s1_is_1"));
            //s1 should be 1
        }
        if(c == 'l')
        {
            puts("s1_is_0!");
            MSC_AddInputGoal(Encode_Term("s1_is_0"));
            //s1 should be 0
        }
        if(c == 'm')
        {
            puts("s2_is_1!");
            MSC_AddInputGoal(Encode_Term("s2_is_1"));
            //s2 should be 1
        }
        if(c == 'n')
        {
            puts("s2_is_0!");
            MSC_AddInputGoal(Encode_Term("s2_is_0"));
            //s2 should be 0
        }
        if(c == 'o')
        {
            puts("s3_is_1!");
            MSC_AddInputGoal(Encode_Term("s3_is_1"));
            //s3 should be 1
        }
        if(c == 'p')
        {
            puts("s3_is_0!");
            MSC_AddInputGoal(Encode_Term("s3_is_0"));
            //s3 should be 0
        }
        if(c == 'q')
        {
            puts("l0_is_1!");
            MSC_AddInputGoal(Encode_Term("l0_is_1"));
            //l0 should be 1
        }
        if(c == 'r')
        {
            puts("l0_is_1!");
            MSC_AddInputGoal(Encode_Term("l0_is_0"));
            //l0 should be 0
        }
        if(c == 's')
        {
            puts("l1_is_1!");
            MSC_AddInputGoal(Encode_Term("l1_is_1"));
            //l1 should be 1
        }
        if(c == 't')
        {
            puts("l1_is_0!");
            MSC_AddInputGoal(Encode_Term("l1_is_0"));
            //l1 should be 0
        }
        if(c == 'u')
        {
            puts("at_s0!");
            MSC_AddInputGoal(Encode_Term("at_s0"));
            //you should be at s0!
        }
        if(c == 'v')
        {
            puts("at_s1!");
            MSC_AddInputGoal(Encode_Term("at_s1"));
            //you should be at s1!
        }
        if(c == 'w')
        {
            puts("at_s2!");
            MSC_AddInputGoal(Encode_Term("at_s2"));
            //you should be at s2!
        }
        if(c == 'x')
        {
            puts("at_s3!");
            MSC_AddInputGoal(Encode_Term("at_s3"));
            //you should be at s3!
        }
        if(c == 'y')
        {
            puts("at_l0!");
            MSC_AddInputGoal(Encode_Term("at_l0"));
            //you should be at l0!
        }
        if(c == 'z')
        {
            puts("at_l1!");
            MSC_AddInputGoal(Encode_Term("at_l1"));
            //you should be at l1!
        }
    }
}

bool op_1_executed = false;
void op_1()
{
    op_1_executed = true;
}
bool op_2_executed = false;
void op_2()
{
    op_2_executed = true;
}
bool op_3_executed = false;
void op_3()
{
    op_3_executed = true;
}
void Sequence_Test()
{
    OUTPUT=0;
    MSC_INIT();
    MOTOR_BABBLING_CHANCE = 0;
    puts(">>Sequence test start");
    MSC_AddOperation(Encode_Term("op_1"), op_1); 
    MSC_AddOperation(Encode_Term("op_2"), op_2); 
    MSC_AddOperation(Encode_Term("op_3"), op_3); 
    for(int i=0;i<5;i++)
    {
        MSC_AddInputBelief(Encode_Term("a"), 0); //0 2 4 5
        MSC_AddInputBelief(Encode_Term("b"), 0);
        MSC_AddInputBelief(Encode_Term("op_1"), 1);
        MSC_AddInputBelief(Encode_Term("g"), 0);
        MSC_Cycles(100);
    }
    for(int i=0;i<100;i++)
    {
        MSC_AddInputBelief(Encode_Term("a"), 0);
        MSC_AddInputBelief(Encode_Term("op_1"), 1);
        MSC_Cycles(100);
    }
    for(int i=0;i<100;i++)
    {
        MSC_AddInputBelief(Encode_Term("b"), 0);
        MSC_AddInputBelief(Encode_Term("op_1"), 1);
        MSC_Cycles(100);
    }
    for(int i=0;i<2;i++)
    {
        MSC_AddInputBelief(Encode_Term("b"), 0);
        MSC_AddInputBelief(Encode_Term("op_2"), 2);
        MSC_AddInputBelief(Encode_Term("g"), 0);
        MSC_Cycles(100);
    }
    for(int i=0;i<2;i++)
    {
        MSC_AddInputBelief(Encode_Term("a"), 0);
        MSC_AddInputBelief(Encode_Term("op_3"), 3);
        MSC_AddInputBelief(Encode_Term("g"), 0);
        MSC_Cycles(100);
    }
    MSC_AddInputBelief(Encode_Term("a"), 0);
    MSC_AddInputBelief(Encode_Term("b"), 0);
    MSC_AddInputGoal(Encode_Term("g"));
    assert(op_1_executed && !op_2_executed && !op_3_executed, "Expected op1 execution");
    op_1_executed = op_2_executed = op_3_executed = false;
    //TODO use "preconditons as operator argument" which then should be equal to (&/,a,b) here
    MSC_Cycles(100);
    MSC_AddInputBelief(Encode_Term("b"), 0);
    MSC_AddInputGoal(Encode_Term("g"));
    assert(!op_1_executed && op_2_executed && !op_3_executed, "Expected op2 execution"); //b here
    op_1_executed = op_2_executed = op_3_executed = false;
    MSC_Cycles(100);
    MSC_AddInputBelief(Encode_Term("a"), 0);
    MSC_AddInputGoal(Encode_Term("g"));
    assert(!op_1_executed && !op_2_executed && op_3_executed, "Expected op3 execution"); //a here
    op_1_executed = op_2_executed = op_3_executed = false;
    MOTOR_BABBLING_CHANCE = MOTOR_BABBLING_CHANCE_INITIAL;
    puts(">>Sequence Test successul");
}

int main(int argc, char *argv[])
{
    //printf("sizeof concept %d\n",(int) sizeof(Concept));
    //exit(0);
    if(argc == 2) //pong
    {
        if(!strcmp(argv[1],"pong"))
        {
            MSC_Pong();
        }
        if(!strcmp(argv[1],"pong2"))
        {
            MSC_Pong2();
        }
        if(!strcmp(argv[1],"testchamber"))
        {
            MSC_TestChamber();
        }
    }
    srand(1337);
    MSC_INIT();
    OUTPUT = 0;
    //Term_Test();
    Stamp_Test();
    FIFO_Test();
    PriorityQueue_Test();
    Table_Test();
    MSC_Alphabet_Test();
    MSC_Procedure_Test();
    Memory_Test();
    MSC_Follow_Test();
    MSC_Multistep_Test();
    MSC_Multistep2_Test();
    Sequence_Test();
    puts("\nAll tests ran successfully, if you wish to run examples now, just pass the corresponding parameter:");
    puts("MSC pong (starts Pong example)");
    puts("MSC pong2 (starts Pong2 example)");
    puts("MSC testchamber (starts Test Chamber multistep procedure learning example)");
    return 0;
}

