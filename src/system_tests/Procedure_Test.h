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

bool NAR_Procedure_Test_Op_executed = false;
Feedback NAR_Procedure_Test_Op(Term args)
{
    puts("op executed by NAR");
    NAR_Procedure_Test_Op_executed = true;
    return (Feedback) {0};
}
void NAR_Procedure_Test()
{
    NAR_INIT();
    puts(">>NAR Procedure test start");
    NAR_AddOperation("^op", NAR_Procedure_Test_Op);
    NAR_AddInputBelief(Narsese_AtomicTerm("a"));
    NAR_Cycles(1);
    puts("---------------");
    NAR_AddInputBelief(Narsese_AtomicTerm("^op"));
    NAR_Cycles(1);
    puts("---------------");
    NAR_AddInputBelief(Narsese_AtomicTerm("result"));
    NAR_Cycles(1);
    puts("---------------");
    NAR_AddInputBelief(Narsese_AtomicTerm("a"));
    NAR_Cycles(1);
    puts("---------------");
    NAR_AddInputGoal(Narsese_AtomicTerm("result"));
    NAR_Cycles(1);
    puts("---------------");
    assert(NAR_Procedure_Test_Op_executed, "NAR should have executed op!");
    puts("<<NAR Procedure test successful");
}
