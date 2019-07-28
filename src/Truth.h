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
    //Frequency
    double frequency;
    //Confidence
    double confidence;
} Truth;

//Parameters//
//----------//
#define TRUTH_EVIDENTAL_HORIZON 1.0
#define TRUTH_PROJECTION_DECAY 0.95

//Methods//
//-------//
double Truth_and(double a, double b);
double Truth_or(double a, double b);
double Truth_w2c(double w);
double Truth_c2w(double c);
double Truth_Expectation(Truth v);
Truth Truth_Revision(Truth v1, Truth v2);
Truth Truth_Deduction(Truth v1, Truth v2);
Truth Truth_Induction(Truth v1, Truth v2);
Truth Truth_Abduction(Truth v1, Truth v2);
Truth Truth_Intersection(Truth v1, Truth v2);
Truth Truth_Eternalize(Truth v);
Truth Truth_Projection(Truth v, long originalTime, long targetTime);
void Truth_Print(Truth *truth);

#endif
