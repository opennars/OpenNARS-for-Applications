#ifndef ATTENTIONVALUE_H
#define ATTENTIONVALUE_H

typedef struct {
    //short term importance
    double priority;
    //long term importance
    double quality;
} AttentionValue;

#endif
