#ifndef TRUTHVALUE_H
#define TRUTHVALUE_H

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
} TruthValue;

//Parameters//
//----------//
#define EVIDENTAL_HORIZON 1.0

//Methods//
//-------//
double and(double a, double b);
double or(double a, double b);
double w2c(double w);
double c2w(double c);
TruthValue getTruthValue(double frequency, double confidence);
TruthValue revision(TruthValue v1, TruthValue v2);
TruthValue deduction(TruthValue v1, TruthValue v2);
TruthValue induction(TruthValue v1, TruthValue v2);
TruthValue abduction(TruthValue v1, TruthValue v2);
TruthValue intersection(TruthValue v1, TruthValue v2);

#endif
