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

#include "./../Shell.h"

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
Feedback NAR_Bandrobot_Pick(Term args)
{
    NAR_Bandrobot_Pick_executed = true;
    return (Feedback) {0};
}
bool NAR_Bandrobot_Drop_executed = false;
Feedback NAR_Bandrobot_Drop(Term args)
{
    NAR_Bandrobot_Drop_executed = true;
    return (Feedback) {0};
}

void NAR_Bandrobot(long iterations)
{
    char initial[] = "+++++|\n"
                     "-----|\n"
                     "     |\n"
                     "     |\n"
                     "     |\n"
                     "'''''|\n";
    puts(">>NAR Bandrobot start");
    NAR_AddOperation("^left", NAR_Bandrobot_Left);
    NAR_AddOperation("^right", NAR_Bandrobot_Right);
    NAR_AddOperation("^pick", NAR_Bandrobot_Pick); 
    NAR_AddOperation("^drop", NAR_Bandrobot_Drop);
    Shell_ProcessInput("*questionpriming=0.0"); //questions are only used for debug here, not to influence attention
    Shell_ProcessInput("*space 100 P");
    Shell_ProcessInput("*similaritydistance=100000.0");
    long t = 0;
    int minpos = 0.0;
    int maxpos = 4.0;
    int position = 0;
    int targetposition = 1; //maxpos; //maxpos/2;
    int goalposition = 1;
    bool picked = false, lastpicked = false, hasObj = false;
    int successes = 0;
    while(1)
    {
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
        if(picked)
        {
            targetposition = position;
        }
        if(NAR_Bandrobot_Pick_executed)
        {
            NAR_Bandrobot_Pick_executed = false;
            if(position == targetposition)
            {
                picked = true;
                hasObj = true;
            }
        }
        if(NAR_Bandrobot_Drop_executed)
        {
            NAR_Bandrobot_Drop_executed = false;
            picked = false;
            hasObj = false;
        }
        //SLEEP;
        CLEAR_SCREEN;
        char world[sizeof(initial)];
        memcpy(world, initial, sizeof(initial));
        DRAW_LINE(position, 2, 0, 1, (char*) world, 'A');
        DRAW_LINE(targetposition, picked ? 3 : 4, 0, 1, (char*) world, 'o');
        DRAW_LINE(goalposition, 5, 0, 1, (char*) world, 'U');
        //NAR_AddInputNarsese("<(<({position} * {targetposition}) --> (+ left)> &/ ^right) =/> picked>?");
        //NAR_AddInputNarsese("<(<({targetposition} * {position}) --> (+ left)> &/ ^left) =/> picked>?");
        char questionStr[NARSESE_LEN_MAX] = {0};
        sprintf(questionStr, "%s", "<(?1 * ?2) --> (+ location)>? :\\:\0");
        NAR_AddInputNarsese(questionStr);
        puts(world);
        if(hasObj) //TODO PROVIDE INFORMATION
        {
            NAR_AddInputNarsese("hasObj. :|:");
        }
        else
        {
            NAR_AddInputNarsese("NotHasObj. :|:");
        }
        Shell_ProcessInput("*concurrent");
        char positionStr[NARSESE_LEN_MAX] = {0};
        float v_position = MIN(1.0, MAX(0.0, (((float) (position-minpos))/((float) (maxpos-minpos + 1.0)))));
        sprintf(positionStr, "%s%.2f%s", "<(crane * P_", v_position, ") |-> location>. :|:\0");
        NAR_AddInputNarsese(positionStr);
        //if(!hasObj) //TODO JUST FEED IF AND ELSE
        {
            Shell_ProcessInput("*concurrent");
            char targetpositionStr[NARSESE_LEN_MAX] = {0};
            float v_targetposition = MIN(1.0, MAX(0.0, (((float) (targetposition-minpos))/((float) (maxpos-minpos + 1.0)))));
            sprintf(targetpositionStr, "%s%.2f%s", "<(object * P_", v_targetposition, ") |-> location>. :|:\0");
            NAR_AddInputNarsese(targetpositionStr);
        }
        //else
        {
            Shell_ProcessInput("*concurrent");
            char goalpositionStr[NARSESE_LEN_MAX] = {0};
            float v_goalposition = MIN(1.0, MAX(0.0, (((float) (goalposition-minpos))/((float) (maxpos-minpos + 1.0)))));
            sprintf(goalpositionStr, "%s%.2f%s", "<(goal * P_", v_goalposition, ") |-> location>. :|:\0");
            NAR_AddInputNarsese(goalpositionStr);
        }
        Shell_ProcessInput("3");
        if(picked && !lastpicked)
        {
            //NAR_AddInputNarsese("picked. :|:");
        }
        else
        if(!picked && lastpicked)
        {
            if(position == goalposition)
            {
                NAR_AddInputNarsese("delivered. :|:");
                targetposition = (((double)myrand()/(double)(MY_RAND_MAX)) * (maxpos));
                goalposition = (((double)myrand()/(double)(MY_RAND_MAX)) * (maxpos));
                successes++;
                if(iterations == -1)
                {
                    for(int k=0; k<10; k++)
                    {
                        SLEEP;
                    }
                }
            }
        }
        lastpicked = picked;
        NAR_AddInputNarsese("delivered! :|:");
        int t_includeSleep = 100;
        bool user_viz_sleep = iterations == -1;
        printf("ratio=%d sleepInVisualization=%s time=%ld\n", successes, (user_viz_sleep ? "true" : "false"), t);
        fflush(stdout);
        if(currentTime > 10500)
        {
            SLEEP;
        }
    }
}
