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
TruthValue TruthValue_GetTruthValue(double frequency, double confidence);
TruthValue TruthValue_Revision(TruthValue v1, TruthValue v2);
TruthValue TruthValue_Deduction(TruthValue v1, TruthValue v2);
TruthValue TruthValue_Induction(TruthValue v1, TruthValue v2);
TruthValue TruthValue_Abduction(TruthValue v1, TruthValue v2);
TruthValue TruthValue_Intersection(TruthValue v1, TruthValue v2);

#endif
