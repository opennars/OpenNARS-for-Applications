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

long base = 1;
Stamp importstamp = {0};

Event Event_InputEvent(Term term, char type, Truth truth, double occurrenceTimeOffset, long currentTime)
{
    Stamp stamp = { .evidentalBase = { base++ } };
    if(importstamp.evidentalBase[0])
    {
        stamp = importstamp;
        base--; //the stamp ID wasn't used
        importstamp = (Stamp) {0};
    }
    return (Event) { .term = term,
                     .type = type, 
                     .truth = truth, 
                     .stamp = stamp,
                     .occurrenceTime = currentTime,
                     .occurrenceTimeOffset = occurrenceTimeOffset,
                     .creationTime = currentTime,
                     .input = true };
}

void Event_INIT()
{
    base = 1;
}

bool Event_Equal(Event *event, Event *existing)
{
    return Truth_Equal(&event->truth, &existing->truth) && event->occurrenceTime == existing->occurrenceTime && Term_Equal(&event->term, &existing->term) && Stamp_Equal(&event->stamp, &existing->stamp);
}

Event Event_Eternalized(Event *event)
{
    Event eternal_event = *event;
    if(event->occurrenceTime != OCCURRENCE_ETERNAL)
    {
        eternal_event.occurrenceTime = OCCURRENCE_ETERNAL;
        eternal_event.truth = Truth_Eternalize(event->truth);
    }
    return eternal_event;
}

bool Event_EqualTermEqualStampLessConfidentThan(Event *event, Event *existing)
{
    return event->truth.confidence <= existing->truth.confidence && event->occurrenceTime == existing->occurrenceTime && Term_Equal(&event->term, &existing->term) && Stamp_Equal(&event->stamp, &existing->stamp);
}
