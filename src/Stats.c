/* 
 * The MIT License
 *
 * Copyright 2020 The OpenNARS authors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Stats.h"

long Stats_countConceptsMatchedTotal = 0;
long Stats_countConceptsMatchedMax = 0;

void Stats_Print(long currentTime)
{
    double Stats_averageBeliefEventPriority = 0.0;
    for(int i=0; i<cycling_belief_events.itemsAmount; i++)
    {
        Stats_averageBeliefEventPriority += cycling_belief_events.items[i].priority;
    }
    Stats_averageBeliefEventPriority /= (double) CYCLING_BELIEF_EVENTS_MAX;
    double Stats_averageGoalEventPriority = 0.0;
    for(int layer=0; layer<CYCLING_GOAL_EVENTS_LAYERS; layer++)
    {
        for(int i=0; i<cycling_goal_events[layer].itemsAmount; i++)
        {
            Stats_averageGoalEventPriority += cycling_goal_events[layer].items[i].priority;
        }
    }
    Stats_averageGoalEventPriority /= (double) CYCLING_GOAL_EVENTS_MAX;
    double Stats_averageConceptPriority = 0.0;
    for(int i=0; i<concepts.itemsAmount; i++)
    {
        Concept *c = concepts.items[i].address;
        Stats_averageConceptPriority += c->priority;
    }
    Stats_averageConceptPriority /= (double) CONCEPTS_MAX;
    double Stats_averageConceptUsefulness = 0.0;
    for(int i=0; i<concepts.itemsAmount; i++)
    {
        Stats_averageConceptUsefulness += concepts.items[i].priority;
    }
    Stats_averageConceptUsefulness /= (double) CONCEPTS_MAX;
    puts("Statistics\n----------");
    printf("countConceptsMatchedTotal:\t%ld\n", Stats_countConceptsMatchedTotal);
    printf("countConceptsMatchedMax:\t%ld\n", Stats_countConceptsMatchedMax);
    long countConceptsMatchedAverage = Stats_countConceptsMatchedTotal / currentTime;
    printf("countConceptsMatchedAverage:\t%ld\n", countConceptsMatchedAverage);
    printf("currentTime:\t\t\t%ld\n", currentTime);
    printf("total concepts:\t\t\t%d\n", concepts.itemsAmount);
    printf("current average concept priority:\t%f\n", Stats_averageConceptPriority);
    printf("current average concept usefulness:\t%f\n", Stats_averageConceptUsefulness);
    printf("current belief events cnt:\t\t%d\n", cycling_belief_events.itemsAmount);
    int goal_events_cnt = 0;
    for(int layer=0; layer<CYCLING_GOAL_EVENTS_LAYERS; layer++)
    {
        goal_events_cnt += cycling_goal_events[layer].itemsAmount;
    }
    printf("current goal events cnt:\t\t%d\n", goal_events_cnt);
    printf("current average belief event priority:\t%f\n", Stats_averageBeliefEventPriority);
    printf("current average goal event priority:\t%f\n", Stats_averageGoalEventPriority);
    printf("Maximum chain length in concept hashtable: %d\n", HashTable_MaximumChainLength(&HTconcepts));
    printf("Maximum chain length in atoms hashtable: %d\n", HashTable_MaximumChainLength(&HTatoms));
    fflush(stdout);
}
