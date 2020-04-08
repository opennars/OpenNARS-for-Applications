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

#include "Event.h"

void Event_SetTerm(Event *event, Term term)
{
    event->term = term;
}

long base = 1;
Event Event_InputEvent(Term term, char type, Truth truth, long currentTime)
{
    return (Event) { .term = term,
                     .type = type, 
                     .truth = truth, 
                     .stamp = (Stamp) { .evidentalBase = { base++ } }, 
                     .occurrenceTime = currentTime,
                     .creationTime = currentTime };
}

void Event_INIT()
{
    base = 1;
}

void Event_Print(Event *event)
{
    printf("Event: \n");
    Term_Print(&event->term);
    printf(event->type == EVENT_TYPE_GOAL ? "type=goal\n" : (EVENT_TYPE_BELIEF ? "type=belief\n" : "type=deleted\n" ));
    Truth_Print(&event->truth);
    Stamp_print(&event->stamp);
    printf("occurrenceTime=%ld\n\n", event->occurrenceTime);
}

bool Event_Equal(Event *event, Event *existing)
{
    return Truth_Equal(&event->truth, &existing->truth) && Term_Equal(&event->term, &existing->term);
}
