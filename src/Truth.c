#include "Truth.h"

double TRUTH_EVIDENTAL_HORIZON = TRUTH_EVIDENTAL_HORIZON_INITIAL;
double TRUTH_PROJECTION_DECAY = TRUTH_PROJECTION_DECAY_INITIAL;

double Truth_w2c(double w)
{
    return w / (w + TRUTH_EVIDENTAL_HORIZON);
}

double Truth_c2w(double c)
{
    return TRUTH_EVIDENTAL_HORIZON * c / (1 - c);
}

double Truth_Expectation(Truth v)
{
    return (v.confidence * (v.frequency - 0.5) + 0.5);
}

Truth Truth_Revision(Truth v1, Truth v2)
{
    double f1 = v1.frequency;
    double f2 = v2.frequency;
    double w1 = Truth_c2w(v1.confidence);
    double w2 = Truth_c2w(v2.confidence);
    double w = w1 + w2;
    double f = MIN(1.0, (w1 * f1 + w2 * f2) / w);
    double c = Truth_w2c(w);
    return (Truth) {.frequency = f, .confidence = MIN(1.0-TRUTH_EPSILON, MAX(MAX(c, v1.confidence), v2.confidence))};
}

#define TruthValues(v1,v2, f1,c1, f2,c2) double f1 = v1.frequency; double f2 = v2.frequency; double c1 = v1.confidence; double c2 = v2.confidence;

Truth Truth_Deduction(Truth v1, Truth v2)
{
    TruthValues(v1,v2, f1,c1, f2,c2);
    double f = f1 * f2;
    double c = c1 * c2 * f;
    return (Truth) {.frequency = f, .confidence = c};
}

Truth Truth_Abduction(Truth v1, Truth v2)
{
    TruthValues(v1,v2, f1,c1, f2,c2);
    double w = f2 * c1 * c2;
    double c = Truth_w2c(w);
    return (Truth) {.frequency = f1, .confidence = c};
}

Truth Truth_Induction(Truth v1, Truth v2)
{
    return Truth_Abduction(v2, v1);
}

Truth Truth_Intersection(Truth v1, Truth v2)
{
    TruthValues(v1,v2, f1,c1, f2,c2);
    double f = f1 * f2;
    double c = c1 * c2;
    return (Truth) {.frequency = f, .confidence = c};
}

Truth Truth_Eternalize(Truth v)
{
    float f = v.frequency;
    float c = v.confidence;
    return (Truth) {.frequency = f, .confidence = Truth_w2c(c)};
}

Truth Truth_Projection(Truth v, long originalTime, long targetTime)
{
    if(originalTime == OCCURRENCE_ETERNAL)
    {
        return v;
    }
    double difference = labs(targetTime - originalTime);
    return (Truth) { .frequency = v.frequency, .confidence = v.confidence * pow(TRUTH_PROJECTION_DECAY,difference)};
}

void Truth_Print(Truth *truth)
{
    printf("Truth: frequency=%f, confidence=%f\n", truth->frequency, truth->confidence);
}

//not part of MSC:

Truth Truth_Exemplification(Truth v1, Truth v2)
{
    TruthValues(v1,v2, f1,c1, f2,c2);
    double w = f1 * f2 * c1 * c2;
    double c = Truth_w2c(w);
    return (Truth) {.frequency = 1.0, .confidence = c};
}

static inline double or(double a, double b)
{
    return 1.0 - (1.0 - a) * (1.0 - b);
}

Truth Truth_Comparison(Truth v1, Truth v2)
{
    TruthValues(v1,v2, f1,c1, f2,c2);
    double f0 = or(f1, f2);
    double f = (f0 == 0.0) ? 0.0 : ((f1*f2) / f0);
    double w = f0 * c1 * c2;
    double c = Truth_w2c(w);
    return (Truth) {.frequency = f, .confidence = c};
}

Truth Truth_Analogy(Truth v1, Truth v2)
{
    TruthValues(v1,v2, f1,c1, f2,c2);
    double f = f1 * f2;
    double c = c1 * c2 * f2;
    return (Truth) {.frequency = f, .confidence = c};
}

Truth Truth_Resemblance(Truth v1, Truth v2)
{
    TruthValues(v1,v2, f1,c1, f2,c2);
    double f = f1 * f2;
    double c = c1 * c2 * or(f1, f2);
    return (Truth) {.frequency = f, .confidence = c};
}
