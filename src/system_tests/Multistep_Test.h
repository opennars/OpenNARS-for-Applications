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

bool YAN_Lightswitch_GotoSwitch_executed = false;
void YAN_Lightswitch_GotoSwitch()
{
    YAN_Lightswitch_GotoSwitch_executed = true;
    puts("YAN invoked goto switch");
}
bool YAN_Lightswitch_ActivateSwitch_executed = false;
void YAN_Lightswitch_ActivateSwitch()
{
    YAN_Lightswitch_ActivateSwitch_executed = true;
    puts("YAN invoked activate switch");
}
void YAN_Multistep_Test()
{
    MOTOR_BABBLING_CHANCE = 0;
    puts(">>YAN Multistep test start");
    YAN_INIT();
    YAN_AddOperation(Narsese_AtomicTerm("^goto_switch"), YAN_Lightswitch_GotoSwitch); 
    YAN_AddOperation(Narsese_AtomicTerm("^activate_switch"), YAN_Lightswitch_ActivateSwitch); 
    for(int i=0; i<5; i++)
    {
        YAN_AddInputBelief(Narsese_AtomicTerm("start_at"));
        YAN_AddInputBelief(Narsese_AtomicTerm("^goto_switch"));
        YAN_Cycles(1);
        YAN_AddInputBelief(Narsese_AtomicTerm("switch_at"));
        YAN_AddInputBelief(Narsese_AtomicTerm("^activate_switch"));
        YAN_AddInputBelief(Narsese_AtomicTerm("switch_active"));
        YAN_Cycles(1);
        YAN_AddInputBelief(Narsese_AtomicTerm("light_active"));
        YAN_Cycles(10);
    }
    YAN_Cycles(10);
    YAN_AddInputBelief(Narsese_AtomicTerm("start_at"));
    YAN_AddInputGoal(Narsese_AtomicTerm("light_active"));
    YAN_Cycles(10);
    assert(YAN_Lightswitch_GotoSwitch_executed && !YAN_Lightswitch_ActivateSwitch_executed, "YAN needs to go to the switch first");
    YAN_Lightswitch_GotoSwitch_executed = false;
    puts("YAN arrived at the switch");
    YAN_AddInputBelief(Narsese_AtomicTerm("switch_at"));
    YAN_AddInputGoal(Narsese_AtomicTerm("light_active"));
    assert(!YAN_Lightswitch_GotoSwitch_executed && YAN_Lightswitch_ActivateSwitch_executed, "YAN needs to activate the switch");
    YAN_Lightswitch_ActivateSwitch_executed = false;
    puts("<<YAN Multistep test successful");
}
