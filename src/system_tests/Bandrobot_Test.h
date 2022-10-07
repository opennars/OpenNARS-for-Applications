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


bool NAR_Bandrobot_Left_executed = false;
double NAR_Bandrobot_amount;
Feedback NAR_Bandrobot_Left(Term args) //* {SELF} val
{
    NAR_Bandrobot_Left_executed = true;
    NAR_Bandrobot_amount = 1.0; //Narsese_NumericAtomValue(args.atoms[2]);
    return (Feedback) {0};
}
bool NAR_Bandrobot_Right_executed = false;
Feedback NAR_Bandrobot_Right(Term args)
{
    NAR_Bandrobot_Right_executed = true;
    NAR_Bandrobot_amount = 1.0; //Narsese_NumericAtomValue(args.atoms[2]);
    return (Feedback) {0};
}
bool NAR_Bandrobot_Pick_executed = false;
Feedback NAR_Bandrobot_Pick()
{
    NAR_Bandrobot_Pick_executed = true;
    return (Feedback) {0};
}
bool NAR_Bandrobot_Drop_executed = false;
Feedback NAR_Bandrobot_Drop()
{
    NAR_Bandrobot_Drop_executed = true;
    return (Feedback) {0};
}

void NAR_Bandrobot(long iterations)
{
    char initial[] = "+++++++++++++++++++++|\n"
                     "---------------------|\n"
                     "                     |\n"
                     "                     |\n"
                     "                     |\n";
    puts(">>NAR Bandrobot start");
    NAR_AddOperation("^left", NAR_Bandrobot_Left);
    NAR_AddOperation("^right", NAR_Bandrobot_Right);
    NAR_AddOperation("^pick", NAR_Bandrobot_Pick); 
    NAR_AddOperation("^drop", NAR_Bandrobot_Drop);
    //Shell_ProcessInput("*motorbabbling=0.03");
    //Shell_ProcessInput("*decisionthreshold=0.51");
    long t = 0;
    double minpos = 0.0;
    double maxpos = 20.0;
    double position = 0;
    double lastposition = 0;
    double targetposition = 3; //maxpos; //maxpos/2;
    bool picked = false, lastpicked = false;
    double debug = 0;
    //for testing, this is the learned relevant knowledge to reach the target:
    //Shell_ProcessInput("*volume=100");
    //Shell_ProcessInput("*decisionthreshold=0.6");
    //given since demo concentrates on learning the numeric relationships: (TODO consider curriculum learning)
//    NAR_AddInputNarsese("<(<(position * targetposition) --> (++ . [left])> &/ ^left) =/> picked>. {1.0 0.99}");
//    NAR_AddInputNarsese("<(<(targetposition * position) --> (++ . [left])> &/ ^right) =/> picked>. {1.0 0.99}");
    //NAR_AddInputNarsese("<(aligned &/ ^pick) =/> picked>. {1.0 0.99}");
    //NAR_AddInputNarsese("<(picked &/ ^drop) =/> dropped>. {1.0 0.99}");
    //NAR_AddInputNarsese("<(<(position ~ targetposition) --> [left]> &/ ^left) =/> picked>. {1.0 0.99}");
    //NAR_AddInputNarsese("<(<(targetposition ~ position) --> [left]> &/ ^right) =/> picked>. {1.0 0.99}");
    //NAR_AddInputNarsese("<(<targetposition <-> position> &/ ^pick) =/> picked>. {1.0 0.99}");
    //NAR_AddInputNarsese("<(picked &/ ^drop) =/> dropped>. {1.0 0.99}");
    Shell_ProcessInput("*motorbabbling=0.01");
    while(1)
    {
        if(myrand()%10000 > 9000)
        {
            //position = (((double)myrand()/(double)(MY_RAND_MAX)) * maxpos); 
        }
        if(t > 10000)
        {
            Shell_ProcessInput("*motorbabbling=false");
        }
        if(t++ > iterations && iterations != -1)
        {
            break;
        }
        if(NAR_Bandrobot_Left_executed)
        {
            NAR_Bandrobot_Left_executed = false;
            position -= NAR_Bandrobot_amount;
        }
        if(NAR_Bandrobot_Right_executed)
        {
            NAR_Bandrobot_Right_executed = false;
            position += NAR_Bandrobot_amount;
        }
        position = MIN(maxpos, MAX(minpos, position));
        if(fabs(position - targetposition) <= 3.0 && position != lastposition)
        {
            Shell_ProcessInput("*volume=100");//--
            NAR_AddInputNarsese("aligned. :|:");
            //NAR_Cycles(50);//--
            //puts("DONE CYCLES");//--
            //exit(0);//--
        }
        if(picked)
        {
            targetposition = position;
        }
        //NAR_Bandrobot_Pick_executed = true; //TODO
        if(NAR_Bandrobot_Pick_executed)
        {
            NAR_Bandrobot_Pick_executed = false;
            if(fabs(position - targetposition) < 1.0) //0.000001)
            {
                picked = true;
            }
        }
        //NAR_Bandrobot_Drop_executed = true; //TODO
        if(NAR_Bandrobot_Drop_executed)
        {
            NAR_Bandrobot_Drop_executed = false;
            picked = false;
        }
        //SLEEP;
        CLEAR_SCREEN;
        char world[sizeof(initial)];
        memcpy(world, initial, sizeof(initial));
        DRAW_LINE(position, 2, 0, 1, (char*) world, 'A');
        DRAW_LINE(targetposition, picked ? 3 : 4, 0, 1, (char*) world, 'o');
        NAR_AddInputNarsese("<(<(position * targetposition) --> [left]> &/ ?1) =/> aligned>?");
        NAR_AddInputNarsese("<(<(targetposition * position) --> [left]> &/ ?1) =/> aligned>?");
        NAR_AddInputNarsese("<(?1 * ?2) --> [left]>? :\\:");
        printf("pos=%f, picked: %d t: %ld\n", position, picked, t);
        puts(world);
        char positionStr[200];
        sprintf(positionStr, "<position --> [left]>. :|: {%f 0.9}", ((position-minpos)/(maxpos-minpos))/3.0);
        NAR_AddInputNarsese(positionStr);
        char targetpositionStr[200];
        sprintf(targetpositionStr, "<targetposition --> [left]>. :|: {%f 0.9}", ((targetposition-minpos)/(maxpos-minpos))/3.0);
        NAR_AddInputNarsese(targetpositionStr);
        //if(picked != lastpicked)
        {
            if(picked && !lastpicked)
            {
                //char pickedpositionStr[200];
                //sprintf(pickedpositionStr, "picked. :|:", targetposition);
                NAR_AddInputNarsese("picked. :|:");
                //NAR_AddInputNarsese(pickedpositionStr);
            }
            else
            if(!picked && lastpicked)
            {
                //char droppedpositionStr[200];
                //sprintf(droppedpositionStr, "dropped. :|:", targetposition);
                NAR_AddInputNarsese("dropped. :|:");
                targetposition = (((double)myrand()/(double)(MY_RAND_MAX)) * (maxpos));
            }
            lastpicked = picked;
        }
        if(t % 100 == 0)
        {
            debug = (((double)myrand()/(double)(MY_RAND_MAX)) * maxpos); //myrand()%(int)(maxpos+1);// 
        }
        //char debugStr[200];
        //sprintf(debugStr, "dropped! :|:", targetposition);
        NAR_AddInputNarsese("dropped! :|:");
        //NAR_AddInputNarsese("aligned. :|:");
        //NAR_AddInputNarsese("<(?1 * ?2) --> [left]>? :|:");
        //NAR_Cycles(10);
        //getchar();
        lastposition = position;
    }
}
