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

#ifndef H_SHELL
#define H_SHELL

/////////////
//  Shell  //
/////////////
//The shell for interaction with NAR
//It accepts Narsese input and output,
//together with various commands starting with *

//Data structure//
//--------------//
#define SHELL_CONTINUE 0
#define SHELL_RESET 1
#define SHELL_EXIT 2

//References//
//----------//
#include "NAR.h"
#include "Stats.h"

//Methods//
//-------//
//Initializes the shell NAR and runs it with stdin/stdout
void Shell_Start();
//Only initializes the shell NAR with the default ops, but can be used differently
void Shell_NARInit();
//Process a shell input line, can be comments, timesteps, Narsese, and commands, returns if system reset was issued
int Shell_ProcessInput(char *line);

#endif
