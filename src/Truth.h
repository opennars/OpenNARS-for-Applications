#ifndef TRUTH_H
#define TRUTH_H

////////////////////////////////////////////
//  ANSNA truth value and truth functions //
////////////////////////////////////////////

//References//
//-----------//
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

//Data structure//
//--------------//
typedef struct {
    /** frequency */
    double frequency;
    /** confidence */
    double confidence;
} Truth;

//Parameters//
//----------//
#define TRUTH_EVIDENTAL_HORIZON 1.0
#define TRUTH_PROJECTION_DECAY 0.99

//Methods//
//-------//
double and(double a, double b);
double or(double a, double b);
double w2c(double w);
double c2w(double c);
double Truth_Expectation(Truth v);
Truth Truth_Revision(Truth v1, Truth v2);
Truth Truth_Deduction(Truth v1, Truth v2);
Truth Truth_Analogy(Truth v1, Truth v2);
Truth Truth_Induction(Truth v1, Truth v2);
Truth Truth_Abduction(Truth v1, Truth v2);
Truth Truth_Intersection(Truth v1, Truth v2);
Truth Truth_Eternalize(Truth v);
Truth Truth_Projection(Truth v, long originalTime, long targetTime);
void Truth_Print(Truth *truth);

#endif
