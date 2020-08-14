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

void InvertedAtomIndex_Test()
{
    puts(">>Inverted atom index test start");
    NAR_INIT();
    Term term = Narsese_Term("<a --> (b & c)>");
    Concept c = { .term = term };
    InvertedAtomIndex_AddConcept(term, &c);
    InvertedAtomIndex_Print();
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("a")]->c == &c, "There was no concept reference added for key a!");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("b")]->c == &c, "There was no concept reference added for key b!");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("c")]->c == &c, "There was no concept reference added for key c!");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex(":")] == NULL, "There was a concept reference added for key inheritance!");
    InvertedAtomIndex_RemoveConcept(term, &c);
    InvertedAtomIndex_Print();
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("a")] == NULL, "Concept reference was not removed for key a!");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("b")] == NULL, "Concept reference was not removed for key b!");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("c")] == NULL, "Concept reference was not removed for key c!");
    InvertedAtomIndex_AddConcept(term, &c);
    Term term2 = Narsese_Term("<b --> d>");
    Concept c2 = { .term = term2 };
    InvertedAtomIndex_AddConcept(term2, &c2);
    InvertedAtomIndex_Print();
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("a")]->c == &c, "There was no concept reference added for key a! (2)");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("b")]->c == &c, "There was no concept reference added for key b! (2)");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("c")]->c == &c, "There was no concept reference added for key c! (2)");
    assert(((ConceptChainElement*) invertedAtomIndex[Narsese_AtomicTermIndex("b")]->next)->c == &c2, "There was no concept2 reference added for key b! (2)");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("d")]->c == &c2, "There was no concept2 reference added for key d! (2)");
    InvertedAtomIndex_RemoveConcept(term, &c);
    puts("after removal");
    InvertedAtomIndex_Print();
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("b")]->c == &c2, "There was no concept2 reference remaining for key b! (3)");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("d")]->c == &c2, "There was no concept2 reference remaining for key d! (3)");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("a")] == NULL, "Concept reference was not removed for key a! (3)");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("c")] == NULL, "Concept reference was not removed for key c! (3)");
    InvertedAtomIndex_RemoveConcept(term2, &c2);
    puts("after removal2");
    InvertedAtomIndex_Print();
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("a")] == NULL, "Concept reference was not removed for key a! (4)");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("b")] == NULL, "Concept reference was not removed for key b! (4)");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("c")] == NULL, "Concept reference was not removed for key c! (4)");
    assert(invertedAtomIndex[Narsese_AtomicTermIndex("d")] == NULL, "Concept reference was not removed for key d! (4)");
    puts(">>Inverted atom index test successul");
}
