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

static bool goto_s0 = false;
static bool goto_s1 = false;
static bool goto_s2 = false;
static bool goto_s3 = false;
static bool goto_l0 = false;
static bool goto_l1 = false;
static bool activate = false;
static bool deactivate = false;
Feedback NAR_TestChamber_goto_s0()
{
    goto_s0 = true;
    puts("NAR goto s0");
    return (Feedback) {0};
}
Feedback NAR_TestChamber_goto_s1()
{
    goto_s1 = true;
    puts("NAR goto s1");
    return (Feedback) {0};
}
Feedback NAR_TestChamber_goto_s2()
{
    goto_s2 = true;
    puts("NAR goto s2");
    return (Feedback) {0};
}
Feedback NAR_TestChamber_goto_s3()
{
    goto_s3 = true;
    puts("NAR goto s3");
    return (Feedback) {0};
}
Feedback NAR_TestChamber_goto_l0()
{
    goto_l0 = true;
    puts("NAR goto l0");
    return (Feedback) {0};
}
Feedback NAR_TestChamber_goto_l1()
{
    goto_l1 = true;
    puts("NAR goto l1");
    return (Feedback) {0};
}
Feedback NAR_TestChamber_activate()
{
    activate = true;
    puts("NAR activate");
    return (Feedback) {0};
}
Feedback NAR_TestChamber_deactivate()
{
    deactivate = true;
    puts("NAR deactivate");
    return (Feedback) {0};
}
void NAR_TestChamber()
{
    MOTOR_BABBLING_CHANCE = 0;
    NAR_AddOperation("^goto_s0", NAR_TestChamber_goto_s0);
    NAR_AddOperation("^goto_s1", NAR_TestChamber_goto_s1);
    NAR_AddOperation("^goto_s2", NAR_TestChamber_goto_s2);
    NAR_AddOperation("^goto_s3", NAR_TestChamber_goto_s3);
    NAR_AddOperation("^goto_l0", NAR_TestChamber_goto_l0);
    NAR_AddOperation("^goto_l1", NAR_TestChamber_goto_l1);
    NAR_AddOperation("^activate", NAR_TestChamber_activate);
    NAR_AddOperation("^deactivate", NAR_TestChamber_deactivate);
    //metrics:
    int goals = 0;
    int reached = 0;
    //world:
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
    char lastcommand = 'a';
    char c = ' ';
    char* nongoal = "none";
    char *goal = nongoal;
    while(1)
    {
        //movement
        if(goto_s0)
        {
            goto_s0 = false;
            pos = pos_s0;
        }
        else
        if(goto_s1)
        {
            goto_s1 = false;
            pos = pos_s1;
        }
        else
        if(goto_s3)
        {
            goto_s3 = false;
            pos = pos_s3;
        }
        else
        if(goto_l1)
        {
            goto_l1 = false;
            pos = pos_l1;
        }
        else
        if(goto_l0)
        {
            goto_l0 = false;
            if(door)
            {
                pos = pos_l0;
            }
        }
        else
        if(goto_s2)
        {
            goto_s2 = false;
            if(door)
            {
                pos = pos_s2;
            }
        }
        //inform NAR about current location
        if(!(activate || deactivate)) //if we took manipulation action we didn't change position
        {
            if(pos == pos_s0)
            {
                NAR_AddInputBelief(Narsese_AtomicTerm("at_s0"));
                if(lastcommand == 'u')
                    reached++;
            }
            if(pos == pos_s1)
            {
                NAR_AddInputBelief(Narsese_AtomicTerm("at_s1"));
                if(lastcommand == 'v')
                    reached++;
            }
            if(pos == pos_s2)
            {
                NAR_AddInputBelief(Narsese_AtomicTerm("at_s2"));
                if(lastcommand == 'w')
                    reached++;
            }
            if(pos == pos_s3)
            {
                NAR_AddInputBelief(Narsese_AtomicTerm("at_s3"));
                if(lastcommand == 'x')
                    reached++;
            }
            if(pos == pos_l0)
            {
                NAR_AddInputBelief(Narsese_AtomicTerm("at_l0"));
                if(lastcommand == 'y')
                    reached++;
            }
            if(pos == pos_l1)
            {
                NAR_AddInputBelief(Narsese_AtomicTerm("at_l1"));
                if(lastcommand == 'z')
                    reached++;
            }
        }
        //manipulation
        bool deactivated = deactivate;
        bool activated = activate;
        deactivate = activate = false;
        if(pos == pos_s1 && (deactivated || (!s1 && !activated)))
        {
            s1 = false;
            NAR_AddInputBelief(Narsese_AtomicTerm("s1_is_0"));
            if(lastcommand == 'l')
                reached++;
            //s1 also closes the door:
            door = false;
            NAR_AddInputBelief(Narsese_AtomicTerm("door_is_closed"));
            if(lastcommand == 'j')
                reached++;
        }
        else
        if(pos == pos_s2 && (deactivated || (!s2 && !activated)))
        {
            s2 = false;
            NAR_AddInputBelief(Narsese_AtomicTerm("s2_is_0"));
            if(lastcommand == 'n')
                reached++;
            //s2 also deactivates l0:
            l0 = false;
            NAR_AddInputBelief(Narsese_AtomicTerm("l0_is_0"));
            if(lastcommand == 'r')
                reached++;
        }
        else
        if(pos == pos_s3 && (deactivated || (!s3 && !activated)))
        {
            s3 = false;
            NAR_AddInputBelief(Narsese_AtomicTerm("s3_is_0"));
            if(lastcommand == 'p')
                reached++;
            //s3 also deactivates l1
            l1 = false;
            NAR_AddInputBelief(Narsese_AtomicTerm("l1_is_0"));
            if(lastcommand == 't')
                reached++;
        }
        else
        if(pos == pos_s1 && (activated || s1))
        {
            s1 = true;
            NAR_AddInputBelief(Narsese_AtomicTerm("s1_is_1"));
            if(lastcommand == 'k')
                reached++;
            //s1 also opens the door:
            door = true;
            NAR_AddInputBelief(Narsese_AtomicTerm("door_is_open"));
            if(lastcommand == 'i')
                reached++;
        }
        else
        if(pos == pos_s2 && (activated || s2))
        {
            s2 = true;
            NAR_AddInputBelief(Narsese_AtomicTerm("s2_is_1"));
            if(lastcommand == 'm')
                reached++;
            //s2 also activates l0:
            l0 = true;
            NAR_AddInputBelief(Narsese_AtomicTerm("l0_is_1"));
            if(lastcommand == 'q')
                reached++;
        }
        else
        if(pos == pos_s3 && (activated || s3))
        {
            s3 = true;
            NAR_AddInputBelief(Narsese_AtomicTerm("s3_is_1"));
            if(lastcommand == 'o')
                reached++;
            //s3 also activates l1
            l1 = true;
            NAR_AddInputBelief(Narsese_AtomicTerm("l1_is_1"));
            if(lastcommand == 's')
                reached++;
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
        CLEAR_SCREEN;
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
        printf("\nCurrent goal: %s", goal);
        printf("\ngoals=%d, reached=%d, ratio=%f \n", (int) goals, (int) reached, ((float) reached)/((float) goals));
        puts("\nCommand:");
        char probe = '\n';
        if(c >= 'a' && c <= 'z')
        {
            probe = getchar(); //skip next newline or space
            if(c == 'Q')
                exit(0);
        }
        if (probe >= 'a' && probe <= 'z')
        {
            c = probe; //but if it's not a space use it as next input
        }
        else
        {
            c = getchar(); //else it's time to get a new character command
            if(c == 'Q')
                exit(0);
        }
        char command = c;
        if(!(c >= 'a' && c <= 'z'))
        {
            if(lastcommand >= 'i' && lastcommand <= 'z') //only repeat goals not actions
            {
                command = lastcommand;
            }
        }
        if(command == 'a')
        {
            goto_s0 = true;
            NAR_AddInputBelief(Narsese_AtomicTerm("^goto_s0"));
        }
        if(command == 'b')
        {
            goto_s1 = true;
            NAR_AddInputBelief(Narsese_AtomicTerm("^goto_s1"));
        }
        if(command == 'c')
        {
            goto_s2 = true;
            NAR_AddInputBelief(Narsese_AtomicTerm("^goto_s2"));
        }
        if(command == 'd')
        {
            goto_s3 = true;
            NAR_AddInputBelief(Narsese_AtomicTerm("^goto_s3"));
        }
        if(command == 'e')
        {
            goto_l0 = true;
            NAR_AddInputBelief(Narsese_AtomicTerm("^goto_l0"));
        }
        if(command == 'f')
        {
            goto_l1 = true;
            NAR_AddInputBelief(Narsese_AtomicTerm("^goto_l1"));
        }
        if(command == 'g')
        {
            activate = true;
            NAR_AddInputBelief(Narsese_AtomicTerm("^activate"));
        }
        if(command == 'h')
        {
            deactivate = true;
            NAR_AddInputBelief(Narsese_AtomicTerm("^deactivate"));
        }
        goal = nongoal;
        if(command == 'i')
        {
            NAR_AddInputGoal(Narsese_AtomicTerm("door_is_open"));
            goal = "door should be open!";
            if(lastcommand != 'i')
                goals++;
        }
        if(command == 'j')
        {
            NAR_AddInputGoal(Narsese_AtomicTerm("door_is_closed"));
            goal = "door should be closed!";
            if(lastcommand != 'j')
                goals++;
        }
        if(command == 'k')
        {
            NAR_AddInputGoal(Narsese_AtomicTerm("s1_is_1"));
            goal = "s1 should be 1!";
            if(lastcommand != 'k')
                goals++;
        }
        if(command == 'l')
        {
            NAR_AddInputGoal(Narsese_AtomicTerm("s1_is_0"));
            goal = "s1 should be 0!";
            if(lastcommand != 'l')
                goals++;
        }
        if(command == 'm')
        {
            NAR_AddInputGoal(Narsese_AtomicTerm("s2_is_1"));
            goal = "s2 should be 1!";
            if(lastcommand != 'm')
                goals++;
        }
        if(command == 'n')
        {
            NAR_AddInputGoal(Narsese_AtomicTerm("s2_is_0"));
            goal = "s2 should be 0!";
            if(lastcommand != 'n')
                goals++;
        }
        if(command == 'o')
        {
            NAR_AddInputGoal(Narsese_AtomicTerm("s3_is_1"));
            goal = "s3 should be 1!";
            if(lastcommand != 'o')
                goals++;
        }
        if(command == 'p')
        {
            NAR_AddInputGoal(Narsese_AtomicTerm("s3_is_0"));
            goal = "s3 should be 0!";
            if(lastcommand != 'p')
                goals++;
        }
        if(command == 'q')
        {
            NAR_AddInputGoal(Narsese_AtomicTerm("l0_is_1"));
            goal = "l0 should be 1!";
            if(lastcommand != 'q')
                goals++;
        }
        if(command == 'r')
        {
            NAR_AddInputGoal(Narsese_AtomicTerm("l0_is_0"));
            goal = "l0 should be 0!";
            if(lastcommand != 'r')
                goals++;
        }
        if(command == 's')
        {
            NAR_AddInputGoal(Narsese_AtomicTerm("l1_is_1"));
            goal = "l1 should be 1!";
            if(lastcommand != 's')
                goals++;
        }
        if(command == 't')
        {
            NAR_AddInputGoal(Narsese_AtomicTerm("l1_is_0"));
            goal = "l1 should be 0!";
            if(lastcommand != 't')
                goals++;
        }
        if(command == 'u')
        {
            NAR_AddInputGoal(Narsese_AtomicTerm("at_s0"));
            goal = "you should be at s0!";
            if(lastcommand != 'u')
                goals++;
        }
        if(command == 'v')
        {
            NAR_AddInputGoal(Narsese_AtomicTerm("at_s1"));
            goal = "you should be at s1!";
            if(lastcommand != 'v')
                goals++;
        }
        if(command == 'w')
        {
            NAR_AddInputGoal(Narsese_AtomicTerm("at_s2"));
            goal = "you should be at s2!";
            if(lastcommand != 'w')
                goals++;
        }
        if(command == 'x')
        {
            NAR_AddInputGoal(Narsese_AtomicTerm("at_s3"));
            goal = "you should be at s3!";
            if(lastcommand != 'x')
                goals++;
        }
        if(command == 'y')
        {
            NAR_AddInputGoal(Narsese_AtomicTerm("at_l0"));
            goal = "you should be at l0!";
            if(lastcommand != 'y')
                goals++;
        }
        if(command == 'z')
        {
            NAR_AddInputGoal(Narsese_AtomicTerm("at_l1"));
            goal = "you should be at l1!";
            if(lastcommand != 'z')
                goals++;
        }
        lastcommand = command;
    }
}
