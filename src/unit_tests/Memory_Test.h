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

void Memory_Test()
{
    NAR_INIT();
    puts(">>Memory test start");
    Event e = Event_InputEvent(Narsese_AtomicTerm("a"), 
                               EVENT_TYPE_BELIEF, 
                               (Truth) { .frequency = 1, .confidence = 0.9 }, 
                               0, 0);
    Memory_AddInputEvent(&e, 0);
    Memory_Conceptualize(&e.term, 1);
    Concept *c1 = Memory_FindConceptByTerm(&e.term);
    assert(c1 != NULL, "Concept should have been created!");
    Event e2 = Event_InputEvent(Narsese_AtomicTerm("b"), 
                               EVENT_TYPE_BELIEF, 
                               (Truth) { .frequency = 1, .confidence = 0.9 }, 
                               0, 0);
    Memory_AddInputEvent(&e2, 0);
    Memory_Conceptualize(&e2.term, 1);
    Concept *c2 = Memory_FindConceptByTerm(&e2.term);
    assert(c2 != NULL, "Concept should have been created!");
    puts("<<Memory test successful");
}
