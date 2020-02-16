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

void Table_Test()
{
    puts(">>Table test start");
    Concept sourceConcept = {0};
    Table table = {0};
    for(int i=TABLE_SIZE*2; i>=1; i--)
    {
        Implication imp = { .term = Narsese_AtomicTerm("test"), 
                            .truth = { .frequency = 1.0, .confidence = 1.0/((double)(i+1)) },
                            .stamp = { .evidentalBase = { i } },
                            .occurrenceTimeOffset = 10,
                            .sourceConcept = &sourceConcept };
        Table_Add(&table, &imp);
    }
    for(int i=0; i<TABLE_SIZE; i++)
    {
        assert(i+1 == table.array[i].stamp.evidentalBase[0], "Item at table position has to be right");
    }
    Implication imp = { .term = Narsese_AtomicTerm("test"), 
                        .truth = { .frequency = 1.0, .confidence = 0.9},
                        .stamp = { .evidentalBase = { TABLE_SIZE*2+1 } },
                        .occurrenceTimeOffset = 10,
                        .sourceConcept = &sourceConcept };
    assert(table.array[0].truth.confidence==0.5, "The highest confidence one should be the first.");
    Table_AddAndRevise(&table, &imp);
    assert(table.array[0].truth.confidence>0.5, "The revision result should be more confident than the table element that existed.");
    puts("<<Table test successful");
}
