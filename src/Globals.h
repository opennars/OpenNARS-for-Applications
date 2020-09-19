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

#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdbool.h>
#include <string.h>
#include <ctype.h> 

/*---------*/
/* Globals */
/*---------*/
//Whether debug mode should be on
#define DEBUG false
//Whether input should be printed
#define PRINT_INPUT_INITIAL true
//Whether derivations should be printed
#define PRINT_DERIVATIONS_INITIAL false
//Whether control information should be printed
#define PRINT_CONTROL_INFO false
//Priority threshold for printing derivations
#define PRINT_DERIVATIONS_PRIORITY_THRESHOLD 0.0
//Console colors (replace with "" to disable colors)
#define COLOR(X) X
#define COLOR_BOLD COLOR("\x1B[1m")
#define COLOR_RED COLOR("\x1B[31m")
#define COLOR_GREEN COLOR("\x1B[32m")
#define COLOR_YELLOW COLOR("\x1B[33m")
#define COLOR_BLUE COLOR("\x1B[34m")
#define COLOR_MAGENTA COLOR("\x1B[35m")
#define COLOR_CYAN COLOR("\x1B[36m")
#define COLOR_RESET COLOR("\x1B[0m")
//Debug macros, debug printing, assert:
#define IN_DEBUG(x) {if(DEBUG){ x } }
//assert, printing message and exiting if b=false
void Globals_assert(bool b, char* message);
#define assert Globals_assert
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
//Number of elements of compile time allocated array:
#define NUM_ELEMENTS(array) (sizeof(array)/sizeof(array[0]))
//Generic hash function on byte array
#define HASH_TYPE long
HASH_TYPE Globals_Hash(HASH_TYPE *data, int pieces);
//Random number generator for reproducibility across platforms
int myrand(void);
void mysrand(unsigned int seed);
#define MY_RAND_MAX 32767

#endif
