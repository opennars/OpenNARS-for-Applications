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

bool NAR_Lightswitch_GotoSwitch_executed = false;
Feedback NAR_Lightswitch_GotoSwitch()
{
    NAR_Lightswitch_GotoSwitch_executed = true;
    puts("NAR invoked goto switch");
    return (Feedback) {0};
}
bool NAR_Lightswitch_ActivateSwitch_executed = false;
Feedback NAR_Lightswitch_ActivateSwitch()
{
    NAR_Lightswitch_ActivateSwitch_executed = true;
    puts("NAR invoked activate switch");
    return (Feedback) {0};
}
void NAR_Multistep_Test()
{
    MOTOR_BABBLING_CHANCE = 0;
    puts(">>NAR Multistep test start");
    NAR_INIT();
    NAR_AddOperation("^goto_switch", NAR_Lightswitch_GotoSwitch);
    NAR_AddOperation("^activate_switch", NAR_Lightswitch_ActivateSwitch);
    for(int i=0; i<5; i++)
    {
        NAR_AddInputBelief(Narsese_AtomicTerm("start_at"));
        NAR_AddInputBelief(Narsese_AtomicTerm("^goto_switch"));
        NAR_Cycles(1);
        NAR_AddInputBelief(Narsese_AtomicTerm("switch_at"));
        NAR_AddInputBelief(Narsese_AtomicTerm("^activate_switch"));
        NAR_AddInputBelief(Narsese_AtomicTerm("switch_active"));
        NAR_Cycles(1);
        NAR_AddInputBelief(Narsese_AtomicTerm("light_active"));
        NAR_Cycles(10);
    }
    NAR_Cycles(10);
    NAR_AddInputBelief(Narsese_AtomicTerm("start_at"));
    NAR_AddInputGoal(Narsese_AtomicTerm("light_active"));
    NAR_Cycles(10);
    assert(NAR_Lightswitch_GotoSwitch_executed && !NAR_Lightswitch_ActivateSwitch_executed, "NAR needs to go to the switch first");
    NAR_Lightswitch_GotoSwitch_executed = false;
    puts("NAR arrived at the switch");
    NAR_AddInputBelief(Narsese_AtomicTerm("switch_at"));
    NAR_AddInputGoal(Narsese_AtomicTerm("light_active"));
    assert(!NAR_Lightswitch_GotoSwitch_executed && NAR_Lightswitch_ActivateSwitch_executed, "NAR needs to activate the switch");
    NAR_Lightswitch_ActivateSwitch_executed = false;
    puts("<<NAR Multistep test successful");
}
