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

Truth Truth_Deduction(Truth v1, Truth v2)
{
    double f1 = v1.frequency;
    double f2 = v2.frequency;
    double c1 = v1.confidence;
    double c2 = v2.confidence;
    double f = f1 * f2;
    double c = c1 * c2 * f;
    return (Truth) {.frequency = f, .confidence = c};
}

Truth Truth_Induction(Truth v1, Truth v2)
{
    double f1 = v2.frequency;
    double f2 = v1.frequency;
    double c1 = v2.confidence;
    double c2 = v1.confidence;
    double w = f2 * c1 * c2;
    double c = Truth_w2c(w);
    return (Truth) {.frequency = f1, .confidence = c};;
}

Truth Truth_Intersection(Truth v1, Truth v2)
{
    double f1 = v1.frequency;
    double f2 = v2.frequency;
    double c1 = v1.confidence;
    double c2 = v2.confidence;
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
    double difference = labs(targetTime - originalTime);
    return (Truth) { .frequency = v.frequency, .confidence = v.confidence * pow(TRUTH_PROJECTION_DECAY,difference)};
}

void Truth_Print(Truth *truth)
{
    printf("Truth: frequency=%f, confidence=%f\n", truth->frequency, truth->confidence);
}
