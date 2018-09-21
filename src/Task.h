#ifndef TASK_H
#define TASK_H

#include "SDR.h"
#include "Stamp.h"
#include "TruthValue.h"

#define GOAL 1
#define JUDGMENT 2

typedef struct {
	SDR sdr;
	char type; //either JUDGMENT or GOAL
	TruthValue truth;
	Stamp stamp;	
	double priority;
} Task;

#endif
