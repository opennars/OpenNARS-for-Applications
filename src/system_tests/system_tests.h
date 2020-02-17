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

#include "Alphabet_Test.h"
#include "Procedure_Test.h"
#include "Follow_Test.h"
#include "Pong2_Test.h"
#include "Pong_Test.h"
#include "Multistep_Test.h"
#include "Multistep2_Test.h"
#include "Testchamber_Test.h"
#include "Sequence_Test.h"
#include "Alien_Test.h"
#include "UDPNAR_Test.h"

void Run_System_Tests()
{
    NAR_Alphabet_Test();
    NAR_Procedure_Test();
    NAR_Follow_Test();
    NAR_Multistep_Test();
    NAR_Multistep2_Test();
    NAR_Sequence_Test();
    NAR_UDPNAR_Test();
}
