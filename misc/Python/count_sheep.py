"""
 * The MIT License
 *
 * Copyright 2022 The OpenNARS authors.
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
 * """

import NAR

NAR.AddInput("*setopname 1 ^count")
NAR.AddInput("*motorbabbling=false")
NAR.AddInput("*volume=0")
NAR.AddInput("10")
NAR.AddInput("<(<(sheep * fence) --> jump> &/ ^count) =/> G>.")

cnt = 0
def count(executions):
    global cnt
    if len(executions) > 0:
        cnt += 1

sheeps = 20
for t in range(sheeps):
    NAR.AddInput("<sheep --> [white]>. :|:")
    NAR.AddInput("<{ex%d} --> [white]>. :|:" % t)
    NAR.AddInput("<({ex%d} * fence) --> jump>. :|:" % t)
    NAR.AddInput("5")
    executions = NAR.AddInput("G! :|:")["executions"]
    for i in range(5):
        executions += NAR.AddInput("1")["executions"]
    NAR.AddInput("500")
    count(executions)
NAR.AddInput("*stats", Print=True)
print("sheep counted:", cnt, "of", str(sheeps))
