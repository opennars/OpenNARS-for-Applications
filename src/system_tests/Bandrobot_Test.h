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


bool NAR_Bandrobot_Left_executed = false;
Term op_args = {0};
void NAR_Bandrobot_Left(Term args)
{
    NAR_Bandrobot_Left_executed = true;
    op_args = args;
}
bool NAR_Bandrobot_Right_executed = false;
void NAR_Bandrobot_Right(Term args)
{
    NAR_Bandrobot_Right_executed = true;
    op_args = args;
}
bool NAR_Bandrobot_Pick_executed = false;
void NAR_Bandrobot_Pick(Term args)
{
    NAR_Bandrobot_Pick_executed = true;
    op_args = args;
}
bool NAR_Bandrobot_Drop_executed = false;
void NAR_Bandrobot_Drop(Term args)
{
    NAR_Bandrobot_Drop_executed = true;
    op_args = args;
}

void NAR_Bandrobot(long iterations)
{
    puts(">>NAR Bandrobot start");
    NAR_AddOperation(Narsese_AtomicTerm("^left"), NAR_Bandrobot_Left); 
    NAR_AddOperation(Narsese_AtomicTerm("^right"), NAR_Bandrobot_Right); 
    NAR_AddOperation(Narsese_AtomicTerm("^pick"), NAR_Bandrobot_Pick); 
    NAR_AddOperation(Narsese_AtomicTerm("^drop"), NAR_Bandrobot_Drop);
    long t = 0;
    while(1)
    {
        if(t++ > iterations && iterations != -1)
        {
            break;
        }
        CLEAR_SCREEN;
        if(NAR_Bandrobot_Left_executed)
        {
			NAR_Bandrobot_Left_executed = false;
			
		}
		if(NAR_Bandrobot_Right_executed)
        {
			NAR_Bandrobot_Left_executed = false;
			
		}
		if(NAR_Bandrobot_Drop_executed)
        {
			NAR_Bandrobot_Left_executed = false;
			
		}
		if(NAR_Bandrobot_Left_executed)
        {
			NAR_Bandrobot_Left_executed = false;
			
		}
		//NAR_AddInputNarsese(
    }
}
