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
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "NAR.h"
#include "./unit_tests/unit_tests.h"
#include "./system_tests/system_tests.h"
#include "Shell.h"
#include "./NetworkNAR/UDPNAR.h"

void Process_Args(int argc, char *argv[])
{
    bool inspectionOnExit = false;
    long iterations = -1;
    if(argc >= 4)
    {
        if(!strcmp(argv[3],"InspectionOnExit"))
        {
            inspectionOnExit = true;
        }
    }
    if(argc >= 3)
    {
        if(!strcmp(argv[2],"InspectionOnExit"))
        {
            inspectionOnExit = true;
        }
    }
    if(argc >= 2)
    {
        NAR_INIT();
        if(!strcmp(argv[1],"NAL_GenerateRuleTable"))
        {
            NAL_GenerateRuleTable();
            exit(0);
        }
        if(!strcmp(argv[1],"shell"))
        {
            Shell_Start();
        }
        for(int i=1; i<argc; i++)
        {
            iterations = i+1 < argc ? atol(argv[i+1]) : -1;
            if(!strcmp(argv[i],"pong"))
            {
                NAR_Pong(iterations);
            }
            else
            if(!strcmp(argv[i],"pong2"))
            {
                NAR_Pong2(iterations);
            }
            else
            if(!strcmp(argv[i],"testchamber"))
            {
                NAR_TestChamber();
            }
            else
            if(!strcmp(argv[i],"alien"))
            {
                NAR_Alien(iterations);
            }
            else
            if(!strcmp(argv[i],"cartpole"))
            {
                NAR_Cartpole(iterations);
            }
            else
            if(!strcmp(argv[i],"robot"))
            {
                NAR_Robot(iterations);
            }
            else
            if(!strcmp(argv[i],"bandrobot"))
            {
                NAR_Bandrobot(iterations);
            }
        }
        if(!strcmp(argv[1],"UDPNAR")) // ./NAR UDPNAR IP PORT timestep(ns per cycle) printDerivations
        {
            char *ip = argv[2];
            int port = atoi(argv[3]);
            long timestep = atol(argv[4]);
            bool printDerivations = !strcmp("true", argv[5]);
            PRINT_DERIVATIONS = printDerivations;
            UDPNAR_Start(ip, port, timestep);
            puts("//press any key and enter to quit!");
            fflush(stdout);
            getchar();
            UDPNAR_Stop();
        }
    }
    if(inspectionOnExit)
    {
        Shell_ProcessInput("*opconfig");
        Shell_ProcessInput("*concepts");
        Shell_ProcessInput("*cycling_belief_events");
        Shell_ProcessInput("*cycling_goal_events");
        Shell_ProcessInput("*stats");
    }
}

void Display_Help()
{
    puts("\nAll C tests ran successfully, run python3 evaluation.py for more comprehensive evaluation!"); 
    puts("");
    puts("Welcome to `OpenNARS for Applications`!");
    puts("```````````````````````````````````````");
    puts(" __        ");
    puts("/ \\`-+-.__ ");
    puts("```  |  /o\\");
    puts("     |  ```");
    puts("  __/ \\__  ");
    puts("  ```````  ");
    puts("If you wish to run examples now, just pass the corresponding parameter:");
    puts("NAR pong (starts Pong example)");
    puts("NAR pong2 (starts Pong2 example)");
    puts("NAR testchamber (starts Test Chamber multistep procedure learning example)");
    puts("NAR alien (starts the alien example)");
    puts("NAR cartpole (starts the cartpole example)");
    puts("NAR bandrobot (starts the band robot example)");
    puts("NAR robot (starts the grid robot example)");
    puts("NAR shell (starts the interactive NAL shell)");
}

int main(int argc, char *argv[])
{
#ifdef SEED
    mysrand(SEED);
#else
    mysrand(666);
#endif
    Process_Args(argc, argv);
    if(argc == 1)
    {
        NAR_INIT();
        Run_Unit_Tests();
        Run_System_Tests();
        Display_Help();
    }
    return 0;
}

