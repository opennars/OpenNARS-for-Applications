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

bool NAR_Follow_Test_Left_executed = false;
Feedback NAR_Follow_Test_Left()
{
    puts("left executed by NAR");
    NAR_Follow_Test_Left_executed = true;
    return (Feedback) {0};
}
bool NAR_Follow_Test_Right_executed = false;
Feedback NAR_Follow_Test_Right()
{
    puts("right executed by NAR");
    NAR_Follow_Test_Right_executed = true;
    return (Feedback) {0};
}
void NAR_Follow_Test()
{
    NAR_INIT();
    puts(">>NAR Follow test start");
    NAR_AddOperation("^left", NAR_Follow_Test_Left);
    NAR_AddOperation("^right", NAR_Follow_Test_Right);
    int simsteps = 1000000;
    int LEFT = 0;
    int RIGHT = 1;
    int BALL = RIGHT;
    int score = 0;
    int goods = 0;
    int bads = 0;
    for(int i=0;i<simsteps; i++)
    {
        puts(BALL == LEFT ? "LEFT" : "RIGHT");
        NAR_AddInputBelief(BALL == LEFT ? Narsese_AtomicTerm("ball_left") : Narsese_AtomicTerm("ball_right"));
        NAR_AddInputGoal(Narsese_AtomicTerm("good_nar"));
        if(NAR_Follow_Test_Right_executed)
        {
            if(BALL == RIGHT)
            {
                NAR_AddInputBelief(Narsese_AtomicTerm("good_nar"));
                printf("(ball=%d) good\n",BALL);
                score++;
                goods++;
            }
            else
            {
                printf("(ball=%d) bad\n",BALL);
                score--;
                bads++;
            }
            NAR_Follow_Test_Right_executed = false;
        }
        if(NAR_Follow_Test_Left_executed)
        {        
            if(BALL == LEFT)
            {
                NAR_AddInputBelief(Narsese_AtomicTerm("good_nar"));
                printf("(ball=%d) good\n",BALL);
                score++;
                goods++;
            }
            else
            {
                printf("(ball=%d) bad\n",BALL);
                score--;
                bads++;
            }
            NAR_Follow_Test_Left_executed = false;
        }
        BALL = myrand() % 2;
        printf("Score %i step%d=\n", score,i);
        assert(score > -100, "too bad score");
        assert(bads < 500, "too many wrong trials");
        if(score >= 500)
            break;
        NAR_Cycles(10);
    }
    printf("<<NAR Follow test successful goods=%d bads=%d ratio=%f\n",goods,bads, (((float) goods)/(((float) goods) + ((float) bads))));
}
