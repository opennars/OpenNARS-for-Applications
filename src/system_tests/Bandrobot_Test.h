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
void NAR_Bandrobot_Left(Term args) //* {SELF} val
{
    NAR_Bandrobot_Left_executed = true;
    NAR_Bandrobot_amount = Narsese_NumericAtomValue(args.atoms[2]);
}
bool NAR_Bandrobot_Right_executed = false;
void NAR_Bandrobot_Right(Term args)
{
    NAR_Bandrobot_Right_executed = true;
    NAR_Bandrobot_amount = Narsese_NumericAtomValue(args.atoms[2]);
}
bool NAR_Bandrobot_Pick_executed = false;
void NAR_Bandrobot_Pick()
{
    NAR_Bandrobot_Pick_executed = true;
}
bool NAR_Bandrobot_Drop_executed = false;
void NAR_Bandrobot_Drop()
{
    NAR_Bandrobot_Drop_executed = true;
}

void NAR_Bandrobot(long iterations)
{
    char initial[] = "                     |\n"
                     "     -----------     |\n"
                     "                     |\n"
                     "                     |\n"
                     "                     |\n";
    puts(">>NAR Bandrobot start");
    NAR_AddOperation(Narsese_AtomicTerm("^left"), NAR_Bandrobot_Left);
    Shell_ProcessInput("*setoprange 1 0 20 int");
    NAR_AddOperation(Narsese_AtomicTerm("^right"), NAR_Bandrobot_Right);
    Shell_ProcessInput("*setoprange 2 0 20 int");
    NAR_AddOperation(Narsese_AtomicTerm("^pick"), NAR_Bandrobot_Pick); 
    NAR_AddOperation(Narsese_AtomicTerm("^drop"), NAR_Bandrobot_Drop);
    Shell_ProcessInput("*motorbabbling=0.03");
    Shell_ProcessInput("*decisionthreshold=0.6");
    long t = 0;
    double minpos = 0.0;
    double maxpos = 20.0;
    double position = 0.0;
    double targetposition = maxpos/2;
    bool picked = false;
    //this for instance is the learned relevant knowledge to reach the target:
    //NAR_AddInputNarsese("<(<(f- * ($1 * #1)) --> position> &/ <({SELF} * #1) --> ^right>) =/> <$1 --> position>>.");
    //NAR_AddInputNarsese("<(<(f+ * ($1 * #1)) --> position> &/ <({SELF} * #1) --> ^left>) =/> <$1 --> position>>.");
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
            }
        }
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
        printf("%f\n", position);
        puts(world);
        char positionStr[200];
        sprintf(positionStr, "<%f --> position>. :|:", position);
        NAR_AddInputNarsese((char*) positionStr);
        if(picked)
        {
            NAR_AddInputNarsese("<object --> [picked]>. :|:");
        }
        else
        {
            NAR_AddInputNarsese("<object --> [dropped]>. :|:");
        }
        char targetpositionStr[200];
        sprintf(targetpositionStr, "<%f --> targetposition>. :|:", targetposition);
        NAR_AddInputNarsese(targetpositionStr);
        //NAR_AddInputNarsese("<10 --> position>! :|:");
        NAR_AddInputNarsese("<18 --> targetposition>! :|:");
        NAR_Cycles(1);
    }
}
