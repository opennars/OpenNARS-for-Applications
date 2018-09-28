#ifndef TASK_H
#define TASK_H

#include "SDR.h"
#include "Stamp.h"
#include "Truth.h"
#include "Attention.h"

#define GOAL 1
#define JUDGMENT 2

typedef struct {
    Attention attention;
    SDR sdr;
    char type; //either JUDGMENT or GOAL
    Truth truth;
    Stamp stamp;
} Task;

#endif
