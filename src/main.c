/* 
 * The MIT License
 *
 * Copyright 2020 The OpenNARS authors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "Term.h"
#include "Memory.h"
#include "Narsese.h"
#include "YAN.h"
#include "NAL.h"
#include "Shell.h"
#include "Stack.h"

void FIFO_Test()
{
    puts(">>FIFO test start");
    FIFO fifo = {0};
    //First, evaluate whether the fifo works, not leading to overflow
    for(int i=FIFO_SIZE*2; i>=1; i--) //"rolling over" once by adding a k*FIFO_Size items
    {
        Event event1 = { .term = Narsese_AtomicTerm("test"), 
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
    Event event2 = { .term = Narsese_AtomicTerm("test"), 
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
    Concept sourceConcept = {0};
    Table table = {0};
    for(int i=TABLE_SIZE*2; i>=1; i--)
    {
        Implication imp = { .term = Narsese_AtomicTerm("test"), 
                            .truth = { .frequency = 1.0, .confidence = 1.0/((double)(i+1)) },
                            .stamp = { .evidentalBase = { i } },
                            .occurrenceTimeOffset = 10,
                            .sourceConcept = &sourceConcept };
        Table_Add(&table, &imp);
    }
    for(int i=0; i<TABLE_SIZE; i++)
    {
        assert(i+1 == table.array[i].stamp.evidentalBase[0], "Item at table position has to be right");
    }
    Implication imp = { .term = Narsese_AtomicTerm("test"), 
                        .truth = { .frequency = 1.0, .confidence = 0.9},
                        .stamp = { .evidentalBase = { TABLE_SIZE*2+1 } },
                        .occurrenceTimeOffset = 10,
                        .sourceConcept = &sourceConcept };
    assert(table.array[0].truth.confidence==0.5, "The highest confidence one should be the first.");
    Table_AddAndRevise(&table, &imp);
    assert(table.array[0].truth.confidence>0.5, "The revision result should be more confident than the table element that existed.");
    puts("<<Table test successful");
}

void Memory_Test()
{
    YAN_INIT();
    puts(">>Memory test start");
    Event e = Event_InputEvent(Narsese_AtomicTerm("a"), 
                               EVENT_TYPE_BELIEF, 
                               (Truth) { .frequency = 1, .confidence = 0.9 }, 
                               1337);
    Memory_addInputEvent(&e, 0);
    assert(belief_events.array[0][0].truth.confidence == (double) 0.9, "event has to be there"); //identify
    Memory_Conceptualize(&e.term, 1);
    Concept *c1 = Memory_FindConceptByTerm(&e.term);
    assert(c1 != NULL, "Concept should have been created!");
    Event e2 = Event_InputEvent(Narsese_AtomicTerm("b"), 
                               EVENT_TYPE_BELIEF, 
                               (Truth) { .frequency = 1, .confidence = 0.9 }, 
                               1337);
    Memory_addInputEvent(&e2, 0);
    Memory_Conceptualize(&e2.term, 1);
    Concept *c2 = Memory_FindConceptByTerm(&e2.term);
    assert(c2 != NULL, "Concept should have been created!");
    Concept_Print(c2);
    puts("<<Memory test successful");
}

void YAN_Alphabet_Test()
{
    YAN_INIT();
    puts(">>YAN Alphabet test start");
    YAN_AddInput(Narsese_AtomicTerm("a"), EVENT_TYPE_BELIEF, YAN_DEFAULT_TRUTH, false);
    for(int i=0; i<50; i++)
    {
        int k=i%10;
        if(i % 3 == 0)
        {
            char c[2] = {'a'+k,0};
            YAN_AddInput(Narsese_AtomicTerm(c), EVENT_TYPE_BELIEF, YAN_DEFAULT_TRUTH, false);
        }
        YAN_Cycles(1);
        puts("TICK");
    }
    puts("<<YAN Alphabet test successful");
}

bool YAN_Procedure_Test_Op_executed = false;
void YAN_Procedure_Test_Op()
{
    puts("op executed by YAN");
    YAN_Procedure_Test_Op_executed = true;
}
void YAN_Procedure_Test()
{
    YAN_INIT();
    puts(">>YAN Procedure test start");
    YAN_AddOperation(Narsese_AtomicTerm("^op"), YAN_Procedure_Test_Op); 
    YAN_AddInputBelief(Narsese_AtomicTerm("a"));
    YAN_Cycles(1);
    puts("---------------");
    YAN_AddInputBelief(Narsese_AtomicTerm("^op"));
    YAN_Cycles(1);
    puts("---------------");
    YAN_AddInputBelief(Narsese_AtomicTerm("result"));
    YAN_Cycles(1);
    puts("---------------");
    YAN_AddInputBelief(Narsese_AtomicTerm("a"));
    YAN_Cycles(1);
    puts("---------------");
    YAN_AddInputGoal(Narsese_AtomicTerm("result"));
    YAN_Cycles(1);
    puts("---------------");
    assert(YAN_Procedure_Test_Op_executed, "YAN should have executed op!");
    puts("<<YAN Procedure test successful");
}

bool YAN_Follow_Test_Left_executed = false;
void YAN_Follow_Test_Left()
{
    puts("left executed by YAN");
    YAN_Follow_Test_Left_executed = true;
}
bool YAN_Follow_Test_Right_executed = false;
void YAN_Follow_Test_Right()
{
    puts("right executed by YAN");
    YAN_Follow_Test_Right_executed = true;
}
void YAN_Follow_Test()
{
    YAN_INIT();
    puts(">>YAN Follow test start");
    YAN_AddOperation(Narsese_AtomicTerm("^left"), YAN_Follow_Test_Left); 
    YAN_AddOperation(Narsese_AtomicTerm("^right"), YAN_Follow_Test_Right); 
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
        YAN_AddInputBelief(BALL == LEFT ? Narsese_AtomicTerm("ball_left") : Narsese_AtomicTerm("ball_right"));
        YAN_AddInputGoal(Narsese_AtomicTerm("good_yan"));
        if(YAN_Follow_Test_Right_executed)
        {
            if(BALL == RIGHT)
            {
                YAN_AddInputBelief(Narsese_AtomicTerm("good_yan"));
                printf("(ball=%d) good\n",BALL);
                score++;
                goods++;
            }
            else
            {
                printf("(ball=%d) bad\n",BALL);
                score--;
                bads++;
            }
            YAN_Follow_Test_Right_executed = false;
        }
        if(YAN_Follow_Test_Left_executed)
        {        
            if(BALL == LEFT)
            {
                YAN_AddInputBelief(Narsese_AtomicTerm("good_yan"));
                printf("(ball=%d) good\n",BALL);
                score++;
                goods++;
            }
            else
            {
                printf("(ball=%d) bad\n",BALL);
                score--;
                bads++;
            }
            YAN_Follow_Test_Left_executed = false;
        }
        BALL = rand() % 2;
        printf("Score %i step%d=\n", score,i);
        assert(score > -100, "too bad score");
        assert(bads < 500, "too many wrong trials");
        if(score >= 500)
            break;
        YAN_Cycles(10);
    }
    printf("<<YAN Follow test successful goods=%d bads=%d ratio=%f\n",goods,bads, (((float) goods)/(((float) goods) + ((float) bads))));
}

bool YAN_Pong_Left_executed = false;
void YAN_Pong_Left()
{
    YAN_Pong_Left_executed = true;
}
bool YAN_Pong_Right_executed = false;
void YAN_Pong_Right()
{
    YAN_Pong_Right_executed = true;
}
bool YAN_Pong_Stop_executed = false;
void YAN_Pong_Stop()
{
    YAN_Pong_Stop_executed = true;
}
void YAN_Pong2(long iterations)
{
    YAN_INIT();
    puts(">>YAN Pong start");
    YAN_AddOperation(Narsese_AtomicTerm("^left"), YAN_Pong_Left); 
    YAN_AddOperation(Narsese_AtomicTerm("^right"), YAN_Pong_Right); 
    YAN_AddOperation(Narsese_AtomicTerm("^stop"), YAN_Pong_Stop); 
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
        if(iterations != -1 && t++ > iterations)
        {
            exit(0);
        }
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
            YAN_AddInputBelief(Narsese_AtomicTerm("ball_right"));
        }
        else
        if(ballX + batWidth < batX)
        {
            YAN_AddInputBelief(Narsese_AtomicTerm("ball_left"));
        }
        else
        {
            YAN_AddInputBelief(Narsese_AtomicTerm("ball_equal"));
        }
        YAN_AddInputGoal(Narsese_AtomicTerm("good_yan"));
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
                YAN_AddInputBelief(Narsese_AtomicTerm("good_yan"));
                puts("good");
                hits++;
            }
            else
            {
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
        if(YAN_Pong_Left_executed)
        {
            YAN_Pong_Left_executed = false;
            puts("Exec: op_left");
            batVX = -3;
        }
        if(YAN_Pong_Right_executed)
        {
            YAN_Pong_Right_executed = false;
            puts("Exec: op_right");
            batVX = 3;
        }
        if(YAN_Pong_Stop_executed)
        {
            YAN_Pong_Stop_executed = false;
            puts("Exec: op_stop");
            batVX = 0;
        }
        batX=MAX(-batWidth*2,MIN(szX-1+batWidth,batX+batVX*batWidth/2));
        printf("Hits=%d misses=%d ratio=%f time=%ld\n", hits, misses, (float) (((float) hits) / ((float) hits + misses)), currentTime);
        if(iterations == -1)
        {
            nanosleep((struct timespec[]){{0, 20000000L}}, NULL); //POSIX sleep
        }
        //YAN_Cycles(10);
    }
}
//int t=0;
void YAN_Pong(long iterations)
{
    YAN_INIT();
    puts(">>YAN Pong start");
    YAN_AddOperation(Narsese_AtomicTerm("^left"), YAN_Pong_Left); 
    YAN_AddOperation(Narsese_AtomicTerm("^right"), YAN_Pong_Right); 
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
    int t=0;
    while(1)
    {
        if(iterations != -1 && t++ > iterations)
        {
            exit(0);
        }
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
            YAN_AddInputBelief(Narsese_AtomicTerm("ball_right"));
        }
        if(ballX < batX)
        {
            YAN_AddInputBelief(Narsese_AtomicTerm("ball_left"));
        }
        YAN_AddInputGoal(Narsese_AtomicTerm("good_yan"));
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
                YAN_AddInputBelief(Narsese_AtomicTerm("good_yan"));
                puts("good");
                hits++;
            }
            else
            {
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
        if(YAN_Pong_Left_executed)
        {
            YAN_Pong_Left_executed = false;
            puts("Exec: op_left");
            batVX = -2;
        }
        if(YAN_Pong_Right_executed)
        {
            YAN_Pong_Right_executed = false;
            puts("Exec: op_right");
            batVX = 2;
        }
        batX=MAX(0,MIN(szX-1,batX+batVX*batWidth/2));
        printf("Hits=%d misses=%d ratio=%f time=%ld\n", hits, misses, (float) (((float) hits) / ((float) hits + misses)), currentTime);
        if(iterations == -1)
        {
            nanosleep((struct timespec[]){{0, 20000000L}}, NULL); //POSIX sleep
        }
        //YAN_Cycles(10);
    }
}

bool YAN_Lightswitch_GotoSwitch_executed = false;
void YAN_Lightswitch_GotoSwitch()
{
    YAN_Lightswitch_GotoSwitch_executed = true;
    puts("YAN invoked goto switch");
}
bool YAN_Lightswitch_ActivateSwitch_executed = false;
void YAN_Lightswitch_ActivateSwitch()
{
    YAN_Lightswitch_ActivateSwitch_executed = true;
    puts("YAN invoked activate switch");
}
void YAN_Multistep_Test()
{
    MOTOR_BABBLING_CHANCE = 0;
    puts(">>YAN Multistep test start");
    YAN_INIT();
    YAN_AddOperation(Narsese_AtomicTerm("^goto_switch"), YAN_Lightswitch_GotoSwitch); 
    YAN_AddOperation(Narsese_AtomicTerm("^activate_switch"), YAN_Lightswitch_ActivateSwitch); 
    for(int i=0; i<5; i++)
    {
        YAN_AddInputBelief(Narsese_AtomicTerm("start_at"));
        YAN_AddInputBelief(Narsese_AtomicTerm("^goto_switch"));
        YAN_Cycles(1);
        YAN_AddInputBelief(Narsese_AtomicTerm("switch_at"));
        YAN_AddInputBelief(Narsese_AtomicTerm("^activate_switch"));
        YAN_AddInputBelief(Narsese_AtomicTerm("switch_active"));
        YAN_Cycles(1);
        YAN_AddInputBelief(Narsese_AtomicTerm("light_active"));
        YAN_Cycles(10);
    }
    YAN_Cycles(10);
    YAN_AddInputBelief(Narsese_AtomicTerm("start_at"));
    YAN_AddInputGoal(Narsese_AtomicTerm("light_active"));
    YAN_Cycles(10);
    assert(YAN_Lightswitch_GotoSwitch_executed && !YAN_Lightswitch_ActivateSwitch_executed, "YAN needs to go to the switch first");
    YAN_Lightswitch_GotoSwitch_executed = false;
    puts("YAN arrived at the switch");
    YAN_AddInputBelief(Narsese_AtomicTerm("switch_at"));
    YAN_AddInputGoal(Narsese_AtomicTerm("light_active"));
    assert(!YAN_Lightswitch_GotoSwitch_executed && YAN_Lightswitch_ActivateSwitch_executed, "YAN needs to activate the switch");
    YAN_Lightswitch_ActivateSwitch_executed = false;
    puts("<<YAN Multistep test successful");
}
void YAN_Multistep2_Test()
{
    MOTOR_BABBLING_CHANCE = 0;
    puts(">>YAN Multistep2 test start");
    YAN_INIT();
    YAN_AddOperation(Narsese_AtomicTerm("^goto_switch"), YAN_Lightswitch_GotoSwitch); 
    YAN_AddOperation(Narsese_AtomicTerm("^activate_switch"), YAN_Lightswitch_ActivateSwitch); 
    for(int i=0; i<5; i++)
    {
        YAN_AddInputBelief(Narsese_AtomicTerm("start_at"));
        YAN_AddInputBelief(Narsese_AtomicTerm("^goto_switch"));
        YAN_Cycles(1);
        YAN_AddInputBelief(Narsese_AtomicTerm("switch_at"));
        YAN_Cycles(10);
    }
    YAN_Cycles(1000);
    for(int i=0; i<5; i++)
    {
        YAN_AddInputBelief(Narsese_AtomicTerm("switch_at"));
        YAN_AddInputBelief(Narsese_AtomicTerm("^activate_switch"));
        YAN_AddInputBelief(Narsese_AtomicTerm("switch_active"));
        YAN_Cycles(1);
        YAN_AddInputBelief(Narsese_AtomicTerm("light_active"));
        YAN_Cycles(10);
    }
    YAN_Cycles(10);
    YAN_AddInputBelief(Narsese_AtomicTerm("start_at"));
    YAN_AddInputGoal(Narsese_AtomicTerm("light_active"));
    YAN_Cycles(10);
    assert(YAN_Lightswitch_GotoSwitch_executed && !YAN_Lightswitch_ActivateSwitch_executed, "YAN needs to go to the switch first (2)");
    YAN_Lightswitch_GotoSwitch_executed = false;
    puts("YAN arrived at the switch");
    YAN_AddInputBelief(Narsese_AtomicTerm("switch_at"));
    YAN_AddInputGoal(Narsese_AtomicTerm("light_active"));
    assert(!YAN_Lightswitch_GotoSwitch_executed && YAN_Lightswitch_ActivateSwitch_executed, "YAN needs to activate the switch (2)");
    YAN_Lightswitch_ActivateSwitch_executed = false;
    puts("<<YAN Multistep2 test successful");
}

static bool goto_s0 = false;
static bool goto_s1 = false;
static bool goto_s2 = false;
static bool goto_s3 = false;
static bool goto_l0 = false;
static bool goto_l1 = false;
static bool activate = false;
static bool deactivate = false;
void YAN_TestChamber_goto_s0()
{
    goto_s0 = true;
    puts("YAN goto s0");
}
void YAN_TestChamber_goto_s1()
{
    goto_s1 = true;
    puts("YAN goto s1");
}
void YAN_TestChamber_goto_s2()
{
    goto_s2 = true;
    puts("YAN goto s2");
}
void YAN_TestChamber_goto_s3()
{
    goto_s3 = true;
    puts("YAN goto s3");
}
void YAN_TestChamber_goto_l0()
{
    goto_l0 = true;
    puts("YAN goto l0");
}
void YAN_TestChamber_goto_l1()
{
    goto_l1 = true;
    puts("YAN goto l1");
}
void YAN_TestChamber_activate()
{
    activate = true;
    puts("YAN activate");
}
void YAN_TestChamber_deactivate()
{
    deactivate = true;
    puts("YAN deactivate");
}
void YAN_TestChamber()
{
    TRUTH_PROJECTION_DECAY = 0.9; //precise timing isn't so important in this domain, so projection decay can be higher
    ANTICIPATION_CONFIDENCE = 0.3; //neg. evidence accumulation can be stronger
    YAN_INIT();
    MOTOR_BABBLING_CHANCE = 0;
    YAN_AddOperation(Narsese_AtomicTerm("^goto_s0"), YAN_TestChamber_goto_s0); 
    YAN_AddOperation(Narsese_AtomicTerm("^goto_s1"), YAN_TestChamber_goto_s1); 
    YAN_AddOperation(Narsese_AtomicTerm("^goto_s2"), YAN_TestChamber_goto_s2); 
    YAN_AddOperation(Narsese_AtomicTerm("^goto_s3"), YAN_TestChamber_goto_s3); 
    YAN_AddOperation(Narsese_AtomicTerm("^goto_l0"), YAN_TestChamber_goto_l0); 
    YAN_AddOperation(Narsese_AtomicTerm("^goto_l1"), YAN_TestChamber_goto_l1); 
    YAN_AddOperation(Narsese_AtomicTerm("^activate"), YAN_TestChamber_activate); 
    YAN_AddOperation(Narsese_AtomicTerm("^deactivate"), YAN_TestChamber_deactivate); 
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
    YAN_AddInputBelief(Narsese_AtomicTerm("at_s0"));
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
            YAN_AddInputBelief(Narsese_AtomicTerm("s1_is_0"));
            //s1 also closes the door:
            door = false;
            puts("door_is_closed.");
            YAN_AddInputBelief(Narsese_AtomicTerm("door_is_closed"));
        }
        else
        if(pos == pos_s2 && deactivate)
        {
            s2 = false;
            puts("s2_is_0.");
            YAN_AddInputBelief(Narsese_AtomicTerm("s2_is_0"));
            //s2 also deactivates l0:
            l0 = false;
            puts("l0_is_0.");
            YAN_AddInputBelief(Narsese_AtomicTerm("l0_is_0"));
        }
        else
        if(pos == pos_s3 && deactivate)
        {
            s3 = false;
            puts("s3_is_0.");
            YAN_AddInputBelief(Narsese_AtomicTerm("s3_is_0"));
            //s3 also deactivates l1
            l1 = false;
            puts("l1_is_0.");
            YAN_AddInputBelief(Narsese_AtomicTerm("l1_is_0"));
        }
        else
        if(pos == pos_s1 && activate)
        {
            s1 = true;
            puts("s1_is_1.");
            YAN_AddInputBelief(Narsese_AtomicTerm("s1_is_1"));
            //s1 also opens the door:
            door = true;
            puts("door_is_open.");
            YAN_AddInputBelief(Narsese_AtomicTerm("door_is_open"));
        }
        else
        if(pos == pos_s2 && activate)
        {
            s2 = true;
            puts("s2_is_1.");
            YAN_AddInputBelief(Narsese_AtomicTerm("s2_is_1"));
            //s2 also activates l0:
            l0 = true;
            puts("l0_is_1.");
            YAN_AddInputBelief(Narsese_AtomicTerm("l0_is_1"));
        }
        else
        if(pos == pos_s3 && activate)
        {
            s3 = true;
            puts("s3_is_1.");
            YAN_AddInputBelief(Narsese_AtomicTerm("s3_is_1"));
            //s3 also activates l1
            l1 = true;
            puts("l1_is_1.");
            YAN_AddInputBelief(Narsese_AtomicTerm("l1_is_1"));
        }
        activate = deactivate = goto_l0 = goto_l1 = goto_s0 = goto_s1 = goto_s2 = goto_s3 = false;
        //inform YAN about current location
        if(pos == pos_s0)
        {
            puts("at_s0.");
            YAN_AddInputBelief(Narsese_AtomicTerm("at_s0"));
        }
        if(pos == pos_s1)
        {
            puts("at_s1.");
            YAN_AddInputBelief(Narsese_AtomicTerm("at_s1"));
        }
        if(pos == pos_s2)
        {
            puts("at_s2.");
            YAN_AddInputBelief(Narsese_AtomicTerm("at_s2"));
        }
        if(pos == pos_s3)
        {
            puts("at_s3.");
            YAN_AddInputBelief(Narsese_AtomicTerm("at_s3"));
        }
        if(pos == pos_l0)
        {
            puts("at_l0.");
            YAN_AddInputBelief(Narsese_AtomicTerm("at_l0"));
        }
        if(pos == pos_l1)
        {
            puts("at_l1.");
            YAN_AddInputBelief(Narsese_AtomicTerm("at_l1"));
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
            puts("^goto_s0.");
            YAN_AddInputBelief(Narsese_AtomicTerm("^goto_s0"));
        }
        if(c == 'b')
        {
            goto_s1 = true;
            puts("^goto_s1.");
            YAN_AddInputBelief(Narsese_AtomicTerm("^goto_s1"));
        }
        if(c == 'c')
        {
            goto_s2 = true;
            puts("^goto_s2.");
            YAN_AddInputBelief(Narsese_AtomicTerm("^goto_s2"));
        }
        if(c == 'd')
        {
            goto_s3 = true;
            puts("^goto_s3.");
            YAN_AddInputBelief(Narsese_AtomicTerm("^goto_s3"));
        }
        if(c == 'e')
        {
            goto_l0 = true;
            puts("^goto_l0.");
            YAN_AddInputBelief(Narsese_AtomicTerm("^goto_l0"));
        }
        if(c == 'f')
        {
            goto_l1 = true;
            puts("^goto_l1.");
            YAN_AddInputBelief(Narsese_AtomicTerm("^goto_l1"));
        }
        if(c == 'g')
        {
            activate = true;
            puts("^activate.");
            YAN_AddInputBelief(Narsese_AtomicTerm("^activate"));
        }
        if(c == 'h')
        {
            deactivate = true;
            puts("^deactivate.");
            YAN_AddInputBelief(Narsese_AtomicTerm("^deactivate"));
        }
        if(c == 'i')
        {
            puts("door_is_open!");
            YAN_AddInputGoal(Narsese_AtomicTerm("door_is_open"));
            //door should be open
        }
        if(c == 'j')
        {
            puts("door_is_closed!");
            YAN_AddInputGoal(Narsese_AtomicTerm("door_is_closed"));
            //door should be closed
        }
        if(c == 'k')
        {
            puts("s1_is_1!");
            YAN_AddInputGoal(Narsese_AtomicTerm("s1_is_1"));
            //s1 should be 1
        }
        if(c == 'l')
        {
            puts("s1_is_0!");
            YAN_AddInputGoal(Narsese_AtomicTerm("s1_is_0"));
            //s1 should be 0
        }
        if(c == 'm')
        {
            puts("s2_is_1!");
            YAN_AddInputGoal(Narsese_AtomicTerm("s2_is_1"));
            //s2 should be 1
        }
        if(c == 'n')
        {
            puts("s2_is_0!");
            YAN_AddInputGoal(Narsese_AtomicTerm("s2_is_0"));
            //s2 should be 0
        }
        if(c == 'o')
        {
            puts("s3_is_1!");
            YAN_AddInputGoal(Narsese_AtomicTerm("s3_is_1"));
            //s3 should be 1
        }
        if(c == 'p')
        {
            puts("s3_is_0!");
            YAN_AddInputGoal(Narsese_AtomicTerm("s3_is_0"));
            //s3 should be 0
        }
        if(c == 'q')
        {
            puts("l0_is_1!");
            YAN_AddInputGoal(Narsese_AtomicTerm("l0_is_1"));
            //l0 should be 1
        }
        if(c == 'r')
        {
            puts("l0_is_1!");
            YAN_AddInputGoal(Narsese_AtomicTerm("l0_is_0"));
            //l0 should be 0
        }
        if(c == 's')
        {
            puts("l1_is_1!");
            YAN_AddInputGoal(Narsese_AtomicTerm("l1_is_1"));
            //l1 should be 1
        }
        if(c == 't')
        {
            puts("l1_is_0!");
            YAN_AddInputGoal(Narsese_AtomicTerm("l1_is_0"));
            //l1 should be 0
        }
        if(c == 'u')
        {
            puts("at_s0!");
            YAN_AddInputGoal(Narsese_AtomicTerm("at_s0"));
            //you should be at s0!
        }
        if(c == 'v')
        {
            puts("at_s1!");
            YAN_AddInputGoal(Narsese_AtomicTerm("at_s1"));
            //you should be at s1!
        }
        if(c == 'w')
        {
            puts("at_s2!");
            YAN_AddInputGoal(Narsese_AtomicTerm("at_s2"));
            //you should be at s2!
        }
        if(c == 'x')
        {
            puts("at_s3!");
            YAN_AddInputGoal(Narsese_AtomicTerm("at_s3"));
            //you should be at s3!
        }
        if(c == 'y')
        {
            puts("at_l0!");
            YAN_AddInputGoal(Narsese_AtomicTerm("at_l0"));
            //you should be at l0!
        }
        if(c == 'z')
        {
            puts("at_l1!");
            YAN_AddInputGoal(Narsese_AtomicTerm("at_l1"));
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
    YAN_INIT();
    MOTOR_BABBLING_CHANCE = 0;
    puts(">>Sequence test start");
    YAN_AddOperation(Narsese_AtomicTerm("^1"), op_1); 
    YAN_AddOperation(Narsese_AtomicTerm("^2"), op_2); 
    YAN_AddOperation(Narsese_AtomicTerm("^3"), op_3); 
    for(int i=0;i<5;i++)
    {
        YAN_AddInputBelief(Narsese_AtomicTerm("a")); //0 2 4 5
        YAN_AddInputBelief(Narsese_AtomicTerm("b"));
        YAN_AddInputBelief(Narsese_AtomicTerm("^1"));
        YAN_AddInputBelief(Narsese_AtomicTerm("g"));
        YAN_Cycles(100);
    }
    for(int i=0;i<100;i++)
    {
        YAN_AddInputBelief(Narsese_AtomicTerm("a"));
        YAN_AddInputBelief(Narsese_AtomicTerm("^1"));
        YAN_Cycles(100);
    }
    for(int i=0;i<100;i++)
    {
        YAN_AddInputBelief(Narsese_AtomicTerm("b"));
        YAN_AddInputBelief(Narsese_AtomicTerm("^1"));
        YAN_Cycles(100);
    }
    for(int i=0;i<2;i++)
    {
        YAN_AddInputBelief(Narsese_AtomicTerm("b"));
        YAN_AddInputBelief(Narsese_AtomicTerm("^2"));
        YAN_AddInputBelief(Narsese_AtomicTerm("g"));
        YAN_Cycles(100);
    }
    for(int i=0;i<2;i++)
    {
        YAN_AddInputBelief(Narsese_AtomicTerm("a"));
        YAN_AddInputBelief(Narsese_AtomicTerm("^3"));
        YAN_AddInputBelief(Narsese_AtomicTerm("g"));
        YAN_Cycles(100);
    }
    YAN_AddInputBelief(Narsese_AtomicTerm("a"));
    YAN_AddInputBelief(Narsese_AtomicTerm("b"));
    YAN_AddInputGoal(Narsese_AtomicTerm("g"));
    assert(op_1_executed && !op_2_executed && !op_3_executed, "Expected op1 execution");
    op_1_executed = op_2_executed = op_3_executed = false;
    //TODO use "preconditons as operator argument" which then should be equal to (&/,a,b) here
    YAN_Cycles(100);
    YAN_AddInputBelief(Narsese_AtomicTerm("b"));
    YAN_AddInputGoal(Narsese_AtomicTerm("g"));
    assert(!op_1_executed && op_2_executed && !op_3_executed, "Expected op2 execution"); //b here
    op_1_executed = op_2_executed = op_3_executed = false;
    YAN_Cycles(100);
    YAN_AddInputBelief(Narsese_AtomicTerm("a"));
    YAN_AddInputGoal(Narsese_AtomicTerm("g"));
    assert(!op_1_executed && !op_2_executed && op_3_executed, "Expected op3 execution"); //a here
    op_1_executed = op_2_executed = op_3_executed = false;
    MOTOR_BABBLING_CHANCE = MOTOR_BABBLING_CHANCE_INITIAL;
    puts(">>Sequence Test successul");
}

void Parser_Test()
{
    puts(">>Parser test start");
    char* narsese = "<<$sth --> (&,[furry,meowing],animal)> =/> <$sth --> [good]>>";
    printf("Narsese: %s\n", narsese);
    char* preprocessed = Narsese_Expand(narsese);
    printf("Preprocessed: %s\n", preprocessed);
    char **tokens = Narsese_PrefixTransform(preprocessed);
    int k = 0;
    for(;tokens[k] != NULL;k++)
    {
        printf("token: %s\n", tokens[k]);
    }
    Term ret = Narsese_Term(narsese);
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        if(ret.atoms[i] != 0)
        {
            printf("Subterm: %i %d %s\n", i, ret.atoms[i], Narsese_atomNames[ret.atoms[i]-1]);
        }
    }
    puts("Result:");
    Narsese_PrintTerm(&ret);
    puts("");
    puts(">>Parser Test successul");
    Narsese_PrintTerm(&ret);
    puts("");
}

bool YAN_Alien_Left_executed = false;
void YAN_Alien_Left()
{
    puts("YAN invoked left");
    YAN_Alien_Left_executed = true;
}
bool YAN_Alien_Right_executed = false;
void YAN_Alien_Right()
{
    puts("YAN invoked right");
    YAN_Alien_Right_executed = true;
}
bool YAN_Alien_Shoot_executed = false;
void YAN_Alien_Shoot()
{
    puts("YAN invoked shoot");
    YAN_Alien_Shoot_executed = true;
}
void YAN_Alien(long iterations)
{
    YAN_INIT();
    puts(">>YAN Alien1 start");
    YAN_AddOperation(Narsese_Term("^left"), YAN_Alien_Left); 
    YAN_AddOperation(Narsese_Term("^right"), YAN_Alien_Right); 
    YAN_AddOperation(Narsese_Term("^shoot"), YAN_Alien_Shoot); 
    double alien0X = 0.5;
    double defenderX = 0.5;
    double alienWidth = 0.18;
    int hits = 0;
    int shots = 0;
    int t=0;
    while(1)
    {
        if(iterations != -1 && t++ > iterations)
        {
            exit(0);
        }
        if(t++%10000 == 0)
        {
            getchar();
        }
        fputs("\033[1;1H\033[2J", stdout); //POSIX clear screen
        bool cond1 = (defenderX <= alien0X - alienWidth);
        bool cond2 = (defenderX >  alien0X + alienWidth);
        if(cond1)
        {
            YAN_AddInputBelief(Narsese_Term("r0"));
        }
        else if(cond2)
        {
            YAN_AddInputBelief(Narsese_Term("l0"));
        }
        else
        {
            YAN_AddInputBelief(Narsese_Term("c0"));
        }
        YAN_AddInputGoal(Narsese_Term("s0"));
        if(YAN_Alien_Shoot_executed)
        {
            YAN_Alien_Shoot_executed = false;
            shots++;
            if(!cond1 && !cond2)
            {
                hits++;
                YAN_AddInputBelief(Narsese_Term("s0"));
                alien0X = ((double)(rand() % 1000)) / 1000.0;
            }
        }
        if(YAN_Alien_Left_executed)
        {
            YAN_Alien_Left_executed = false;
            defenderX = MAX(0.0, defenderX-0.1);
        }
        if(YAN_Alien_Right_executed)
        {
            YAN_Alien_Right_executed = false;
            defenderX = MIN(1.0, defenderX+0.1);
        }
        printf("shots=%d hits=%d ratio=%f time=%ld\n", shots, hits, (float) (((float) hits) / ((float) shots)), currentTime);
        //nanosleep((struct timespec[]){{0, 10000000L}}, NULL); //POSIX sleep
        //YAN_Cycles(10);
    }
}

void RuleTable_Test()
{
    puts(">>RuleTable test start");
    YAN_INIT();
    YAN_AddInput(Narsese_Term("<cat --> animal>"), EVENT_TYPE_BELIEF, YAN_DEFAULT_TRUTH, true);
    YAN_AddInput(Narsese_Term("<animal --> being>"), EVENT_TYPE_BELIEF, YAN_DEFAULT_TRUTH, true);
    YAN_Cycles(1);
    puts(">>RuleTable test successul");
}

void Stack_Test()
{
    puts(">>Stack test start");
    Stack stack = {0};
    Concept c1 = {0};
    Concept c2 = {0};
    VMItem item1 = { .value = &c1 };
    VMItem item2 = { .value = &c2 };
    Stack_Push(&stack, &item1);
    assert(stack.stackpointer == 1, "Stackpointer wasn't incremented");
    assert(stack.items[0]->value == &c1, "Item should point to c1");
    assert(!Stack_IsEmpty(&stack), "Stack should not be empty");
    VMItem *item1_popped = Stack_Pop(&stack);
    assert(stack.stackpointer == 0, "Stackpointer wasn't decremented");
    assert(item1_popped->value == &c1, "Popped item1 should point to c1 (1)");
    Stack_Push(&stack, &item1);
    Stack_Push(&stack, &item2);
    assert(stack.stackpointer == 2, "Stackpointer wrong");
    VMItem *item2_popped = Stack_Pop(&stack);
    assert(item2_popped->value == &c2, "Popped item2 should point to c2");
    VMItem *item1_popped_again = Stack_Pop(&stack);
    assert(item1_popped_again->value == &c1, "Popped item1 should point to c1 (2)");
    assert(Stack_IsEmpty(&stack), "Stack should be empty");
    puts(">>Stack test successul");
}

void HashTable_Test()
{
    puts(">>HashTable test start");
    HashTable_Init(&HTconcepts);
    assert(HTconcepts.VMStack.stackpointer == CONCEPTS_MAX, "The stack should be full!");
    //Insert a first concept:
    Term term1 = Narsese_Term("<a --> b>");
    Concept c1 = { .id = 1, .term = term1 };
    Concept_SetTerm(&c1, term1);
    HashTable_Set(&HTconcepts, &c1);
    assert(HTconcepts.VMStack.stackpointer == CONCEPTS_MAX-1, "One item should be taken off of the stack");
    assert(HTconcepts.HT[c1.term_hash] != NULL, "Item didn't go in right place");
    //Return it
    Concept *c1_returned = HashTable_Get(&HTconcepts, &term1);
    assert(c1_returned != NULL, "Returned item is null (1)");
    assert(Term_Equal(&c1.term, &c1_returned->term), "Hashtable Get led to different term than we put into (1)");
    //insert another with the same hash:
    Term term2 = Narsese_Term("<c --> d>");
    Concept c2 = { .id = 2, .term = term2, .term_hash = c1.term_hash }; //use different term but same hash, hash collision!
    HashTable_Set(&HTconcepts, &c2);
    //get first one:
    Concept *c1_returned_again = HashTable_Get(&HTconcepts, &term1);
    assert(c1_returned_again != NULL, "Returned item is null (2)");
    assert(Term_Equal(&c1.term, &c1_returned_again->term), "Hashtable Get led to different term than we put into (2)");
    Term term3 = Narsese_Term("<e --> f>");
    Concept c3 = { .id = 3, .term = term3, .term_hash = c1.term_hash }; //use different term but same hash, hash collision!
    HashTable_Set(&HTconcepts, &c3);
    //there should be a chain of 3 concepts now at the hash position:
    assert(Term_Equal(&HTconcepts.HT[c1.term_hash]->value->term, &c1.term), "c1 not there! (1)");
    assert(Term_Equal(&((VMItem*)HTconcepts.HT[c1.term_hash]->next)->value->term, &c2.term), "c2 not there! (1)");
    assert(Term_Equal(&((VMItem*)((VMItem*)HTconcepts.HT[c1.term_hash]->next)->next)->value->term, &c3.term), "c3 not there! (1)");
    //Delete the middle one, c2
    HashTable_Delete(&HTconcepts, &c2);
    assert(((VMItem*)HTconcepts.HT[c1.term_hash]->next)->value->id == 3, "c3 not there according to id! (2)");
    assert(Term_Equal(&HTconcepts.HT[c1.term_hash]->value->term, &c1.term), "c1 not there! (2)");
    assert(Term_Equal(&((VMItem*)HTconcepts.HT[c1.term_hash]->next)->value->term, &c3.term), "c3 not there! (2)");
    //Delete the last one, c3
    HashTable_Delete(&HTconcepts, &c3);
    assert(Term_Equal(&HTconcepts.HT[c1.term_hash]->value->term, &c1.term), "c1 not there! (3)");
    //Delete the first one, which is the last one left, c1
    HashTable_Delete(&HTconcepts, &c1);
    assert(HTconcepts.HT[c1.term_hash] == NULL, "Hash table at hash position must be null");
    assert(HTconcepts.VMStack.stackpointer == CONCEPTS_MAX, "All elements should be free now");
    puts(">>HashTable test successul");
}

int main(int argc, char *argv[])
{
    long iterations = -1;
    //printf("sizeof concept %d\n",(int) sizeof(Concept));
    //exit(0);
    srand(1337);
    if(argc == 3)
    {
        iterations = atol(argv[2]);
    }
    if(argc >= 2)
    {
        if(!strcmp(argv[1],"NAL_GenerateRuleTable"))
        {
            YAN_INIT();
            NAL_GenerateRuleTable();
            exit(0);
        }
        if(!strcmp(argv[1],"pong"))
        {
            YAN_Pong(iterations);
        }
        if(!strcmp(argv[1],"pong2"))
        {
            YAN_Pong2(iterations);
        }
        if(!strcmp(argv[1],"testchamber"))
        {
            YAN_TestChamber();
        }
        if(!strcmp(argv[1],"alien"))
        {
            YAN_Alien(iterations);
        }
        if(!strcmp(argv[1],"shell"))
        {
            Shell_Start();
        }
    }
    YAN_INIT();
    //Term_Test();
    Stamp_Test();
    FIFO_Test();
    PriorityQueue_Test();
    Table_Test();
    YAN_Alphabet_Test();
    YAN_Procedure_Test();
    Memory_Test();
    YAN_Follow_Test();
    YAN_Multistep_Test();
    YAN_Multistep2_Test();
    Sequence_Test();
    Parser_Test();
    RuleTable_Test();
    Stack_Test();
    HashTable_Test();
    puts("\nAll tests ran successfully, if you wish to run examples now, just pass the corresponding parameter:");
    puts("YAN pong (starts Pong example)");
    puts("YAN pong2 (starts Pong2 example)");
    puts("YAN testchamber (starts Test Chamber multistep procedure learning example)");
    puts("YAN alien (starts the alien example)");
    puts("YAN shell (starts the interactive NAL shell)");
    return 0;
}

