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
 
#include "Concept.h"

Event Concept_SelectBelief(Concept *c, long queryTime)
{
    Event belief = c->belief;
    Event future_belief = c->predicted_belief;
    //but if there is a predicted one in the event's window, use this one
    if(queryTime != OCCURRENCE_ETERNAL && future_belief.type != EVENT_TYPE_DELETED &&
       labs(queryTime - future_belief.occurrenceTime) < EVENT_BELIEF_DISTANCE) //take event as belief if it's stronger
    {
        future_belief.truth = Truth_Projection(future_belief.truth, future_belief.occurrenceTime, queryTime);
        future_belief.occurrenceTime = queryTime;
        belief = future_belief;
    }
    //unless there is an actual belief which falls into the event's window
    Event project_belief = c->belief_spike;
    if(queryTime != OCCURRENCE_ETERNAL && project_belief.type != EVENT_TYPE_DELETED &&
       labs(queryTime - project_belief.occurrenceTime) < EVENT_BELIEF_DISTANCE) //take event as belief if it's stronger
    {
        project_belief.truth = Truth_Projection(project_belief.truth, project_belief.occurrenceTime, queryTime);
        project_belief.occurrenceTime = queryTime;
        belief = project_belief;
    }
    return belief;
}


double Concept_Priority(Concept *c, long currentTime)
{
    double belief_prio = MAX(MAX(Event_Priority(&c->belief, currentTime), 
                                 Event_Priority(&c->belief_spike, currentTime)),
                                 Event_Priority(&c->predicted_belief, currentTime));
    double goal_prio = MAX(Event_Priority(&c->goal_spike, currentTime), 
                           Event_Priority(&c->goal, currentTime));
    return MAX(belief_prio, goal_prio);
}
