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

NAR.AddInput("*motorbabbling=false")
NAR.AddInput("*volume=0")
NAR.AddInput("<<({SELF} * $1) --> collect> ==> <(<$1 --> [left]> &/ ^left) =/> <$1 --> [centered]>>>.")
NAR.AddInput("<<({SELF} * $1) --> collect> ==> <(<$1 --> [centered]> &/ ^pick) =/> G>>.")
NAR.AddInput("<<({SELF} * $1) --> collect> ==> <(<$1 --> [right]> &/ ^right) =/> <$1 --> [centered]>>>.")
NAR.AddInput("<you <-> {SELF}>.")

selfpos = "centered"
objects = [("box","left"), ("bottle","right")]

test = ""
while not test.startswith("<"):
    test = input()
NAR.AddInput(test)
NAR.AddInput("20")

def sim_step():
    global selfpos
    for (obj, location) in objects:
        relLoc = "centered"
        if (selfpos == "left" and (location == "centered" or location == "right")) or (selfpos == "centered" and location == "right"):
            relLoc = "right"
        if (selfpos == "right" and (location == "centered" or location == "left")) or (selfpos == "centered" and location == "left"):
            relLoc = "left"
        NAR.AddInput("<%s --> [%s]>. :|:" % (obj, relLoc))
    executions = NAR.AddInput("G! :|:")["executions"]
    executions += NAR.AddInput("10")["executions"]
    if len(executions) > 0:
        op = executions[0]["operator"]
        if op == "^left":
            if selfpos == "centered":
                selfpos = "left"
            elif selfpos == "right":
                selfpos = "centered"
        elif op == "^right":
            if selfpos == "centered":
                selfpos = "right"
            elif selfpos == "left":
                selfpos = "centered"
sim_step()
sim_step()

