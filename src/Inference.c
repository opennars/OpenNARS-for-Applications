#include "Inference.h"

TruthValue getTruthValue(double frequency, double confidence)
{
	TruthValue result;
	result.frequency = frequency;
	result.confidence = confidence;
	return result;
}

double and(double a, double b)
{        
	return a*b;
}

double or(double a, double b) 
{
	return 1 - ((1 - a) * (1 - b));
}

double w2c(double w)
{
	return w / (w + EVIDENTAL_HORIZON);
}

double c2w(double c)
{
	return EVIDENTAL_HORIZON * c / (1 - c);
}

TruthValue revision(TruthValue v1, TruthValue v2)
{
	double f1 = v1.frequency;
	double f2 = v2.frequency;
	double w1 = c2w(v1.confidence);
	double w2 = c2w(v2.confidence);
	double w = w1 + w2;
	double f = (w1 * f1 + w2 * f2) / w;
	double c = w2c(w);
	return getTruthValue(f,c);
}

TruthValue deduction(TruthValue v1, TruthValue v2)
{
	double f1 = v1.frequency;
	double f2 = v2.frequency;
	double c1 = v1.confidence;
	double c2 = v2.confidence;
	double f = and(f1, f2);
	double c = and(and(c1, c2), f);
	return getTruthValue(f,c);
}

TruthValue induction(TruthValue v1, TruthValue v2)
{
	return abduction(v2, v1);
}

TruthValue abduction(TruthValue v1, TruthValue v2)
{
	double f1 = v1.frequency;
	double f2 = v2.frequency;
	double c1 = v1.confidence;
	double c2 = v2.confidence;
	double w = and(f2, and(c1, c2));
	double c = w2c(w);
	return getTruthValue(f1, c);
}

TruthValue intersection(TruthValue v1, TruthValue v2)
{
	double f1 = v1.frequency;
	double f2 = v2.frequency;
	double c1 = v1.confidence;
	double c2 = v2.confidence;
	double f = and(f1, f2);
	double c = and(c1, c2);
	return getTruthValue(f, c);
}
