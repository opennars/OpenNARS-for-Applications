#ifndef TRUTH_H
#define TRUTH_H

////////////////////////////////////////////
//  ANSNA truth value and truth functions //
////////////////////////////////////////////

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
#define EVIDENTAL_HORIZON 1.0

//Methods//
//-------//
double and(double a, double b);
double or(double a, double b);
double w2c(double w);
double c2w(double c);
Truth Truth_GetTruth(double frequency, double confidence);
Truth Truth_Revision(Truth v1, Truth v2);
Truth Truth_Deduction(Truth v1, Truth v2);
Truth Truth_Induction(Truth v1, Truth v2);
Truth Truth_Abduction(Truth v1, Truth v2);
Truth Truth_Intersection(Truth v1, Truth v2);

#endif
