#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdbool.h>

/*-------*/
/* Flags */
/*-------*/
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

//Debug macros, debug printing, assert:
#define DEBUG_INFO(x) {if(DEBUG == 1){}}
#ifdef DEBUG
#define PRINTD printf
#else
#define PRINTD(format, args...) ((void)0)
#endif
#define IN_DEBUG(x) {if(DEBUG){ x } }
void assert(bool b, char* message);
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#endif
