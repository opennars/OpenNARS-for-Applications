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

#include "Stamp.h"

Stamp Stamp_make(Stamp *stamp1, Stamp *stamp2)
{
    Stamp ret = {0};
    bool processStamp1 = true;
    bool processStamp2 = true;
    for (int j=0, i=0; i<STAMP_SIZE; i++)
    {
        if(processStamp1)
        {
            if(stamp1->evidentalBase[i] != STAMP_FREE)
            {
                ret.evidentalBase[j] = stamp1->evidentalBase[i];
                j++;
                if(j >= STAMP_SIZE)
                {
                    break;
                }
            }
            else
            {
                processStamp1 = false;
            }
        }
        if(processStamp2)
        {
            if(stamp2->evidentalBase[i] != STAMP_FREE)
            {
                ret.evidentalBase[j] = stamp2->evidentalBase[i];
                j++;
                if(j >= STAMP_SIZE)
                {
                    break;
                }
            }
            else
            {
                processStamp2 = false;
            }
        }
        if(!processStamp1 && !processStamp2)
        {
            break;
        }
    }
    return ret;
}

bool Stamp_checkOverlap(Stamp *a, Stamp *b)
{
    for(int i=0;i<STAMP_SIZE;i++)
    {
        if(a->evidentalBase[i] == STAMP_FREE)
        {
            break;
        }
        for(int j=0;j<STAMP_SIZE;j++)
        {
            if(b->evidentalBase[j] == STAMP_FREE)
            {
                break;
            }
            if(a->evidentalBase[i] == b->evidentalBase[j])
            {
                return true;
            }
        }
    }
    return false;
}

bool Stamp_Equal(Stamp *a, Stamp *b)
{
    for (int i=0;i<STAMP_SIZE;i++)
    {
        if (a->evidentalBase[i] == STAMP_FREE)
        {
            return b->evidentalBase[i] == STAMP_FREE;
        }
        bool contained = false;
        for (int j=0;j<STAMP_SIZE;j++)
        {
            if (b->evidentalBase[j] == STAMP_FREE)
            {
                return a->evidentalBase[i] == STAMP_FREE;
            }
            if (a->evidentalBase[i] == b->evidentalBase[j])
            {
                contained = true;
                break;
            }
        }
        if(!contained)
        {
            return false;
        }
    }
    return true;
}

void Stamp_print(Stamp *stamp)
{
    fputs("Stamp=[", stdout);
    for(int i=0; i<STAMP_SIZE; i++)
    {
        if(stamp->evidentalBase[i] == STAMP_FREE)
        {
            break;
        }
        if(i+1 >= STAMP_SIZE || stamp->evidentalBase[i+1] == STAMP_FREE)
        {
            printf("%ld", stamp->evidentalBase[i]);
        }
        else
        {
            printf("%ld,", stamp->evidentalBase[i]);
        }
    }
    fputs("]", stdout);
}
