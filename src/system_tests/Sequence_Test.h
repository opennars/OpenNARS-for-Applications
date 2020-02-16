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

bool op_1_executed = false;
void op_1()
{
    op_1_executed = true;
}
bool op_2_executed = false;
void op_2()
{
    op_2_executed = true;
}
bool op_3_executed = false;
void op_3()
{
    op_3_executed = true;
}
void Sequence_Test()
{
    YAN_INIT();
    MOTOR_BABBLING_CHANCE = 0;
    puts(">>Sequence test start");
    YAN_AddOperation(Narsese_AtomicTerm("^1"), op_1); 
    YAN_AddOperation(Narsese_AtomicTerm("^2"), op_2); 
    YAN_AddOperation(Narsese_AtomicTerm("^3"), op_3); 
    for(int i=0;i<5;i++)
    {
        YAN_AddInputBelief(Narsese_AtomicTerm("a")); //0 2 4 5
        YAN_AddInputBelief(Narsese_AtomicTerm("b"));
        YAN_AddInputBelief(Narsese_AtomicTerm("^1"));
        YAN_AddInputBelief(Narsese_AtomicTerm("g"));
        YAN_Cycles(100);
    }
    for(int i=0;i<100;i++)
    {
        YAN_AddInputBelief(Narsese_AtomicTerm("a"));
        YAN_AddInputBelief(Narsese_AtomicTerm("^1"));
        YAN_Cycles(100);
    }
    for(int i=0;i<100;i++)
    {
        YAN_AddInputBelief(Narsese_AtomicTerm("b"));
        YAN_AddInputBelief(Narsese_AtomicTerm("^1"));
        YAN_Cycles(100);
    }
    for(int i=0;i<2;i++)
    {
        YAN_AddInputBelief(Narsese_AtomicTerm("b"));
        YAN_AddInputBelief(Narsese_AtomicTerm("^2"));
        YAN_AddInputBelief(Narsese_AtomicTerm("g"));
        YAN_Cycles(100);
    }
    for(int i=0;i<2;i++)
    {
        YAN_AddInputBelief(Narsese_AtomicTerm("a"));
        YAN_AddInputBelief(Narsese_AtomicTerm("^3"));
        YAN_AddInputBelief(Narsese_AtomicTerm("g"));
        YAN_Cycles(100);
    }
    YAN_AddInputBelief(Narsese_AtomicTerm("a"));
    YAN_AddInputBelief(Narsese_AtomicTerm("b"));
    YAN_AddInputGoal(Narsese_AtomicTerm("g"));
    assert(op_1_executed && !op_2_executed && !op_3_executed, "Expected op1 execution");
    op_1_executed = op_2_executed = op_3_executed = false;
    //TODO use "preconditons as operator argument" which then should be equal to (&/,a,b) here
    YAN_Cycles(100);
    YAN_AddInputBelief(Narsese_AtomicTerm("b"));
    YAN_AddInputGoal(Narsese_AtomicTerm("g"));
    assert(!op_1_executed && op_2_executed && !op_3_executed, "Expected op2 execution"); //b here
    op_1_executed = op_2_executed = op_3_executed = false;
    YAN_Cycles(100);
    YAN_AddInputBelief(Narsese_AtomicTerm("a"));
    YAN_AddInputGoal(Narsese_AtomicTerm("g"));
    assert(!op_1_executed && !op_2_executed && op_3_executed, "Expected op3 execution"); //a here
    op_1_executed = op_2_executed = op_3_executed = false;
    MOTOR_BABBLING_CHANCE = MOTOR_BABBLING_CHANCE_INITIAL;
    puts(">>Sequence Test successul");
}
