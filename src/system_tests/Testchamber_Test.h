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
void NAR_TestChamber_goto_s0()
{
    goto_s0 = true;
    puts("NAR goto s0");
}
void NAR_TestChamber_goto_s1()
{
    goto_s1 = true;
    puts("NAR goto s1");
}
void NAR_TestChamber_goto_s2()
{
    goto_s2 = true;
    puts("NAR goto s2");
}
void NAR_TestChamber_goto_s3()
{
    goto_s3 = true;
    puts("NAR goto s3");
}
void NAR_TestChamber_goto_l0()
{
    goto_l0 = true;
    puts("NAR goto l0");
}
void NAR_TestChamber_goto_l1()
{
    goto_l1 = true;
    puts("NAR goto l1");
}
void NAR_TestChamber_activate()
{
    activate = true;
    puts("NAR activate");
}
void NAR_TestChamber_deactivate()
{
    deactivate = true;
    puts("NAR deactivate");
}
void NAR_TestChamber()
{
    TRUTH_PROJECTION_DECAY = 0.9; //precise timing isn't so important in this domain, so projection decay can be higher
    ANTICIPATION_CONFIDENCE = 0.3; //neg. evidence accumulation can be stronger
    NAR_INIT();
    MOTOR_BABBLING_CHANCE = 0;
    NAR_AddOperation(Narsese_AtomicTerm("^goto_s0"), NAR_TestChamber_goto_s0); 
    NAR_AddOperation(Narsese_AtomicTerm("^goto_s1"), NAR_TestChamber_goto_s1); 
    NAR_AddOperation(Narsese_AtomicTerm("^goto_s2"), NAR_TestChamber_goto_s2); 
    NAR_AddOperation(Narsese_AtomicTerm("^goto_s3"), NAR_TestChamber_goto_s3); 
    NAR_AddOperation(Narsese_AtomicTerm("^goto_l0"), NAR_TestChamber_goto_l0); 
    NAR_AddOperation(Narsese_AtomicTerm("^goto_l1"), NAR_TestChamber_goto_l1); 
    NAR_AddOperation(Narsese_AtomicTerm("^activate"), NAR_TestChamber_activate); 
    NAR_AddOperation(Narsese_AtomicTerm("^deactivate"), NAR_TestChamber_deactivate); 
    int32_t size = 7;
    char world[7][13] = { "_________    ",
                          "| l0  s2| s1 ",
                          "| (0)-0 |-0  ",
                          "|_______|    ",
                          "          s0 ",
                          " l1  s3   .< ",
                          " (0)-0       " };
    //positions:
    const int32_t pos_s0 = 0;
    const int32_t pos_s1 = 1;
    const int32_t pos_s2 = 2;
    const int32_t pos_s3 = 3;
    const int32_t pos_l0 = 4;
    const int32_t pos_l1 = 5;
    //states:
    int32_t pos = pos_s0; //agent position
    bool s1 = false;
    bool s2 = false;
    bool s3 = false;
    bool l0 = false;
    bool l1 = false;
    bool door = false; //door closed
    puts("at_s0");
    NAR_AddInputBelief(Narsese_AtomicTerm("at_s0"));
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
            NAR_AddInputBelief(Narsese_AtomicTerm("s1_is_0"));
            //s1 also closes the door:
            door = false;
            puts("door_is_closed.");
            NAR_AddInputBelief(Narsese_AtomicTerm("door_is_closed"));
        }
        else
        if(pos == pos_s2 && deactivate)
        {
            s2 = false;
            puts("s2_is_0.");
            NAR_AddInputBelief(Narsese_AtomicTerm("s2_is_0"));
            //s2 also deactivates l0:
            l0 = false;
            puts("l0_is_0.");
            NAR_AddInputBelief(Narsese_AtomicTerm("l0_is_0"));
        }
        else
        if(pos == pos_s3 && deactivate)
        {
            s3 = false;
            puts("s3_is_0.");
            NAR_AddInputBelief(Narsese_AtomicTerm("s3_is_0"));
            //s3 also deactivates l1
            l1 = false;
            puts("l1_is_0.");
            NAR_AddInputBelief(Narsese_AtomicTerm("l1_is_0"));
        }
        else
        if(pos == pos_s1 && activate)
        {
            s1 = true;
            puts("s1_is_1.");
            NAR_AddInputBelief(Narsese_AtomicTerm("s1_is_1"));
            //s1 also opens the door:
            door = true;
            puts("door_is_open.");
            NAR_AddInputBelief(Narsese_AtomicTerm("door_is_open"));
        }
        else
        if(pos == pos_s2 && activate)
        {
            s2 = true;
            puts("s2_is_1.");
            NAR_AddInputBelief(Narsese_AtomicTerm("s2_is_1"));
            //s2 also activates l0:
            l0 = true;
            puts("l0_is_1.");
            NAR_AddInputBelief(Narsese_AtomicTerm("l0_is_1"));
        }
        else
        if(pos == pos_s3 && activate)
        {
            s3 = true;
            puts("s3_is_1.");
            NAR_AddInputBelief(Narsese_AtomicTerm("s3_is_1"));
            //s3 also activates l1
            l1 = true;
            puts("l1_is_1.");
            NAR_AddInputBelief(Narsese_AtomicTerm("l1_is_1"));
        }
        activate = deactivate = goto_l0 = goto_l1 = goto_s0 = goto_s1 = goto_s2 = goto_s3 = false;
        //inform NAR about current location
        if(pos == pos_s0)
        {
            puts("at_s0.");
            NAR_AddInputBelief(Narsese_AtomicTerm("at_s0"));
        }
        if(pos == pos_s1)
        {
            puts("at_s1.");
            NAR_AddInputBelief(Narsese_AtomicTerm("at_s1"));
        }
        if(pos == pos_s2)
        {
            puts("at_s2.");
            NAR_AddInputBelief(Narsese_AtomicTerm("at_s2"));
        }
        if(pos == pos_s3)
        {
            puts("at_s3.");
            NAR_AddInputBelief(Narsese_AtomicTerm("at_s3"));
        }
        if(pos == pos_l0)
        {
            puts("at_l0.");
            NAR_AddInputBelief(Narsese_AtomicTerm("at_l0"));
        }
        if(pos == pos_l1)
        {
            puts("at_l1.");
            NAR_AddInputBelief(Narsese_AtomicTerm("at_l1"));
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
        for(int32_t i=0; i<size; i++)
        {
            for(int32_t j=0; j<13; j++)
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
            NAR_AddInputBelief(Narsese_AtomicTerm("^goto_s0"));
        }
        if(c == 'b')
        {
            goto_s1 = true;
            puts("^goto_s1.");
            NAR_AddInputBelief(Narsese_AtomicTerm("^goto_s1"));
        }
        if(c == 'c')
        {
            goto_s2 = true;
            puts("^goto_s2.");
            NAR_AddInputBelief(Narsese_AtomicTerm("^goto_s2"));
        }
        if(c == 'd')
        {
            goto_s3 = true;
            puts("^goto_s3.");
            NAR_AddInputBelief(Narsese_AtomicTerm("^goto_s3"));
        }
        if(c == 'e')
        {
            goto_l0 = true;
            puts("^goto_l0.");
            NAR_AddInputBelief(Narsese_AtomicTerm("^goto_l0"));
        }
        if(c == 'f')
        {
            goto_l1 = true;
            puts("^goto_l1.");
            NAR_AddInputBelief(Narsese_AtomicTerm("^goto_l1"));
        }
        if(c == 'g')
        {
            activate = true;
            puts("^activate.");
            NAR_AddInputBelief(Narsese_AtomicTerm("^activate"));
        }
        if(c == 'h')
        {
            deactivate = true;
            puts("^deactivate.");
            NAR_AddInputBelief(Narsese_AtomicTerm("^deactivate"));
        }
        if(c == 'i')
        {
            puts("door_is_open!");
            NAR_AddInputGoal(Narsese_AtomicTerm("door_is_open"));
            //door should be open
        }
        if(c == 'j')
        {
            puts("door_is_closed!");
            NAR_AddInputGoal(Narsese_AtomicTerm("door_is_closed"));
            //door should be closed
        }
        if(c == 'k')
        {
            puts("s1_is_1!");
            NAR_AddInputGoal(Narsese_AtomicTerm("s1_is_1"));
            //s1 should be 1
        }
        if(c == 'l')
        {
            puts("s1_is_0!");
            NAR_AddInputGoal(Narsese_AtomicTerm("s1_is_0"));
            //s1 should be 0
        }
        if(c == 'm')
        {
            puts("s2_is_1!");
            NAR_AddInputGoal(Narsese_AtomicTerm("s2_is_1"));
            //s2 should be 1
        }
        if(c == 'n')
        {
            puts("s2_is_0!");
            NAR_AddInputGoal(Narsese_AtomicTerm("s2_is_0"));
            //s2 should be 0
        }
        if(c == 'o')
        {
            puts("s3_is_1!");
            NAR_AddInputGoal(Narsese_AtomicTerm("s3_is_1"));
            //s3 should be 1
        }
        if(c == 'p')
        {
            puts("s3_is_0!");
            NAR_AddInputGoal(Narsese_AtomicTerm("s3_is_0"));
            //s3 should be 0
        }
        if(c == 'q')
        {
            puts("l0_is_1!");
            NAR_AddInputGoal(Narsese_AtomicTerm("l0_is_1"));
            //l0 should be 1
        }
        if(c == 'r')
        {
            puts("l0_is_1!");
            NAR_AddInputGoal(Narsese_AtomicTerm("l0_is_0"));
            //l0 should be 0
        }
        if(c == 's')
        {
            puts("l1_is_1!");
            NAR_AddInputGoal(Narsese_AtomicTerm("l1_is_1"));
            //l1 should be 1
        }
        if(c == 't')
        {
            puts("l1_is_0!");
            NAR_AddInputGoal(Narsese_AtomicTerm("l1_is_0"));
            //l1 should be 0
        }
        if(c == 'u')
        {
            puts("at_s0!");
            NAR_AddInputGoal(Narsese_AtomicTerm("at_s0"));
            //you should be at s0!
        }
        if(c == 'v')
        {
            puts("at_s1!");
            NAR_AddInputGoal(Narsese_AtomicTerm("at_s1"));
            //you should be at s1!
        }
        if(c == 'w')
        {
            puts("at_s2!");
            NAR_AddInputGoal(Narsese_AtomicTerm("at_s2"));
            //you should be at s2!
        }
        if(c == 'x')
        {
            puts("at_s3!");
            NAR_AddInputGoal(Narsese_AtomicTerm("at_s3"));
            //you should be at s3!
        }
        if(c == 'y')
        {
            puts("at_l0!");
            NAR_AddInputGoal(Narsese_AtomicTerm("at_l0"));
            //you should be at l0!
        }
        if(c == 'z')
        {
            puts("at_l1!");
            NAR_AddInputGoal(Narsese_AtomicTerm("at_l1"));
            //you should be at l1!
        }
    }
}
