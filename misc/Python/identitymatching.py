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

from math import exp
import NAR
import random
import json
import sys

loud = not "silent" in sys.argv
def printloud(x=""):
    if loud:
        print(x)
seed = "random"
for x in sys.argv:
    if x.startswith("seed="):
        seed = int(x.split("seed=")[1]) 
if seed != "random":
    random.seed(seed)

NAR.AddInput("*volume=0", Print=loud)
NAR.AddInput("*babblingops=2", Print=loud)
NAR.AddInput("*motorbabbling=0.2", Print=loud)
NAR.AddInput("*setopname 1 ^left", Print=loud)
NAR.AddInput("*setopname 2 ^right", Print=loud)

conditions = [["<A1 --> [sample]>. :|:", "<A1 --> [left]>. :|:", "<A2 --> [right]>. :|:", "^left"],
              ["<A1 --> [sample]>. :|:", "<A2 --> [left]>. :|:",
                  "<A1 --> [right]>. :|:", "^right"],
              ["<A2 --> [sample]>. :|:", "<A1 --> [left]>. :|:",
                  "<A2 --> [right]>. :|:", "^right"],
              ["<A2 --> [sample]>. :|:", "<A2 --> [left]>. :|:", "<A1 --> [right]>. :|:", "^left"]]

test_conditions = [["<A3 --> [sample]>. :|:", "<A3 --> [left]>. :|:", "<A4 --> [right]>. :|:", "^left"],
              ["<A3 --> [sample]>. :|:", "<A4 --> [left]>. :|:",
                  "<A3 --> [right]>. :|:", "^right"],
              ["<A4 --> [sample]>. :|:", "<A3 --> [left]>. :|:",
                  "<A4 --> [right]>. :|:", "^right"],
              ["<A4 --> [sample]>. :|:", "<A4 --> [left]>. :|:", "<A3 --> [right]>. :|:", "^left"]]

seq = [0, 1, 2, 3]
printloud("\x1b[35mBaseline\x1b[0m")
printloud("\x1b[35m***************\x1b[0m")
printloud()

for b in range(10):
    printloud("\x1b[35mStarting baseline block " + str(b) + ".\x1b[0m")
    printloud()
    s = random.sample(seq, 4)
    correct = 0

    for i in s:
        trial = conditions[i]
        sample = trial[0]
        left = trial[1]
        right = trial[2]
        expected_op = trial[3]

        NAR.AddInput(sample, Print=False)
        NAR.AddInput(left, Print=False)
        NAR.AddInput(right, Print=False)

        response = NAR.AddInput("G! :|:", Print=False)
        executions = response["executions"]
        if executions:
            op = executions[0]["operator"]

            if op == expected_op:
                printloud("\x1B[32mCORRECT: " + op + "\x1B[0m")
                correct += 1
                # NAR.AddInput("G. :|:", Print=False)
            elif op != expected_op:
                printloud("\x1B[31mINCORRECT: " + op + "\x1B[0m")
                # NAR.AddInput("G. :|: {0.0 0.9}", Print=True)      

        NAR.AddInput("50", Print=False)
    
    printloud("\x1B[34mCorrect = " + str(correct/4) + "\x1B[0m")

    printloud("End of block")
    printloud()

printloud("\x1b[33m")
NAR.AddInput("<((<A1 --> [sample]> &/ <A1 --> [left]>) &/ ^left) =/> G>?", Print=loud)
NAR.AddInput("<((<A1 --> [sample]> &/ <A1 --> [right]>) &/ ^right) =/> G>?", Print=loud)
NAR.AddInput("<((<A2 --> [sample]> &/ <A2 --> [left]>) &/ ^left) =/> G>?", Print=loud)
NAR.AddInput("<((<A2 --> [sample]> &/ <A2 --> [right]>) &/ ^right) =/> G>?", Print=loud)
NAR.AddInput("<((<#1 --> [sample]> &/ <#1 --> [left]>) &/ ^left) =/> G>?", Print=loud)
NAR.AddInput("<((<#1 --> [sample]> &/ <#1 --> [right]>) &/ ^right) =/> G>?", Print=loud)
printloud("\x1b[0m")

seq = [0, 1, 2, 3]
printloud("\x1b[35mLearning A1->A1 and A2->A2\x1b[0m")
printloud("\x1b[35m***************\x1b[0m")
printloud()

for b in range(10):
    printloud("\x1b[35mStarting block " + str(b) + ".\x1b[0m")
    printloud()
    s = random.sample(seq, 4)
    correct = 0

    for i in s:
        trial = conditions[i]
        sample = trial[0]
        left = trial[1]
        right = trial[2]
        expected_op = trial[3]

        NAR.AddInput(sample, Print=False)
        NAR.AddInput(left, Print=False)
        NAR.AddInput(right, Print=False)

        response = NAR.AddInput("G! :|:", Print=loud)
        executions = response["executions"]
        if executions:
            op = executions[0]["operator"]

            if op == expected_op:
                printloud("\x1B[32mCORRECT: " + op + "\x1B[0m")
                NAR.AddInput("G. :|:", Print=False)
                correct += 1
            elif op != expected_op:
                printloud("\x1B[31mINCORRECT: " + op + "\x1B[0m")
                NAR.AddInput("G. :|: {0.0 0.9}", Print=False)      

        NAR.AddInput("50", Print=False)

    printloud("\x1b[33m")
    response = NAR.AddInput("<((<A1 --> [sample]> &/ <A1 --> [left]>) &/ ^left) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        s1 = float(c)
    else:
        s1 = 0

    response = NAR.AddInput("<((<A1 --> [sample]> &/ <A1 --> [right]>) &/ ^right) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        s2 = float(c)
    else:
        s2 = 0

    response = NAR.AddInput("<((<A2 --> [sample]> &/ <A2 --> [left]>) &/ ^left) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        s3 = float(c)
    else:
        s3 = 0

    response = NAR.AddInput("<((<A2 --> [sample]> &/ <A2 --> [right]>) &/ ^right) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        s4 = float(c)
    else:
        s4 = 0

    response = NAR.AddInput("<((<#1 --> [sample]> &/ <#1 --> [left]>) &/ ^left) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        g1 = float(c)
    else:
        g1 = 0

    response = NAR.AddInput("<((<#1 --> [sample]> &/ <#1 --> [right]>) &/ ^right) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        g2 = float(c)
    else:
        g2 = 0

    printloud("\x1b[0m")

    specific = (s1 + s2 + s3 + s4)/4
    generic = (g1 + g2)/2

    printloud("specific " + str(specific))
    printloud("generic " + str(generic))

    printloud("\x1B[34mCorrect " + str(correct/4) + "\x1B[0m")

    printloud("End of block")
    printloud()


specifics = []
generics = []
corrects = []

seq = [0, 1, 2, 3]
printloud("\x1b[35mTESTING A1->A1 and A2->A2\x1b[0m")
printloud("\x1b[35m***************\x1b[0m")
printloud()

for b in range(4):
    printloud("\x1b[35mStarting testing block " + str(b) + ".\x1b[0m")
    printloud()
    s = random.sample(seq, 4)
    correct = 0

    for i in s:
        trial = conditions[i]
        sample = trial[0]
        left = trial[1]
        right = trial[2]
        expected_op = trial[3]

        NAR.AddInput(sample, Print=False)
        NAR.AddInput(left, Print=False)
        NAR.AddInput(right, Print=False)

        response = NAR.AddInput("G! :|:", Print=False)

        if response["executions"]:
            op = response["executions"][0]["operator"]

            if op == expected_op:
                printloud("\x1B[32mCORRECT: " + op + "\x1B[0m")
                correct += 1
            elif op != expected_op:
                printloud("\x1b[31mINCORRECT: " + op +"\x1b[0m")
        else:
            printloud("\x1b[31mINCORRECT: " + op +"\x1b[0m")

        NAR.AddInput("50", Print=False)

    printloud("\x1b[33m")
    response = NAR.AddInput("<((<A1 --> [sample]> &/ <A1 --> [left]>) &/ ^left) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        s1 = float(c)
    else:
        s1 = 0

    response = NAR.AddInput("<((<A1 --> [sample]> &/ <A1 --> [right]>) &/ ^right) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        s2 = float(c)
    else:
        s2 = 0

    response = NAR.AddInput("<((<A2 --> [sample]> &/ <A2 --> [left]>) &/ ^left) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        s3 = float(c)
    else:
        s3 = 0

    response = NAR.AddInput("<((<A2 --> [sample]> &/ <A2 --> [right]>) &/ ^right) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        s4 = float(c)
    else:
        s4 = 0

    response = NAR.AddInput("<((<#1 --> [sample]> &/ <#1 --> [left]>) &/ ^left) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        g1 = float(c)
    else:
        g1 = 0

    response = NAR.AddInput("<((<#1 --> [sample]> &/ <#1 --> [right]>) &/ ^right) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        g2 = float(c)
    else:
        g2 = 0

    printloud("\x1b[0m")

    specific = (s1 + s2 + s3 + s4)/4
    generic = (g1 + g2)/2

    printloud("specific " + str(specific))
    printloud("generic " + str(generic))
    printloud("\x1B[34mCorrect " + str(correct/4) + "\x1B[0m")
    specifics.append(specific)
    generics.append(generic)
    corrects.append(correct/4)
    printloud("End of block")
    printloud()

printloud("\x1b[33m")
NAR.AddInput("<((<A1 --> [sample]> &/ <A1 --> [left]>) &/ ^left) =/> G>?", Print=loud)
NAR.AddInput("<((<A1 --> [sample]> &/ <A1 --> [right]>) &/ ^right) =/> G>?", Print=loud)
NAR.AddInput("<((<A2 --> [sample]> &/ <A2 --> [left]>) &/ ^left) =/> G>?", Print=loud)
NAR.AddInput("<((<A2 --> [sample]> &/ <A2 --> [right]>) &/ ^right) =/> G>?", Print=loud)
NAR.AddInput("<((<#1 --> [sample]> &/ <#1 --> [left]>) &/ ^left) =/> G>?", Print=loud)
NAR.AddInput("<((<#1 --> [sample]> &/ <#1 --> [right]>) &/ ^right) =/> G>?", Print=loud)
printloud("\x1b[0m")


seq = [0, 1, 2, 3]
printloud("\x1b[35mTESTING A3->A3 and A4->A4\x1b[0m")
printloud("\x1b[35m***************\x1b[0m")
printloud()

for b in range(4):
    printloud("\x1b[35mStarting generalized testing block " + str(b) + ".\x1b[0m")
    printloud()
    s = random.sample(seq, 4)
    correct = 0

    for i in s:
        trial = test_conditions[i]
        sample = trial[0]
        left = trial[1]
        right = trial[2]
        expected_op = trial[3]

        NAR.AddInput(sample, Print=False)
        NAR.AddInput(left, Print=False)
        NAR.AddInput(right, Print=False)

        response = NAR.AddInput("G! :|:", Print=False)

        if response["executions"]:
            op = response["executions"][0]["operator"]

            if op == expected_op:
                printloud("\x1B[32mCORRECT: " + op + "\x1B[0m")
                correct += 1
            elif op != expected_op:
                printloud("\x1b[31mINCORRECT: " + op + "\x1b[0m")
        else:
            printloud("\x1b[31mINCORRECT: " + op + "\x1b[0m")

        NAR.AddInput("50", Print=False)

    printloud("\x1B[34mCorrect " + str(correct/4) + "\x1B[0m")

    printloud("\x1b[33m")
    response = NAR.AddInput("<((<A1 --> [sample]> &/ <A1 --> [left]>) &/ ^left) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        s1 = float(c)
    else:
        s1 = 0

    response = NAR.AddInput("<((<A1 --> [sample]> &/ <A1 --> [right]>) &/ ^right) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        s2 = float(c)
    else:
        s2 = 0

    response = NAR.AddInput("<((<A2 --> [sample]> &/ <A2 --> [left]>) &/ ^left) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        s3 = float(c)
    else:
        s3 = 0

    response = NAR.AddInput("<((<A2 --> [sample]> &/ <A2 --> [right]>) &/ ^right) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        s4 = float(c)
    else:
        s4 = 0

    response = NAR.AddInput("<((<#1 --> [sample]> &/ <#1 --> [left]>) &/ ^left) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        g1 = float(c)
    else:
        g1 = 0

    response = NAR.AddInput("<((<#1 --> [sample]> &/ <#1 --> [right]>) &/ ^right) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        g2 = float(c)
    else:
        g2 = 0

    printloud("\x1b[0m")

    specific = (s1 + s2 + s3 + s4)/4
    generic = (g1 + g2)/2

    printloud("specific " + str(specific))
    printloud("generic " + str(generic))
    specifics.append(specific)
    generics.append(generic)
    corrects.append(correct/4)
    printloud("End of block")
    printloud()

printloud("\x1b[33m")
NAR.AddInput("<((<A1 --> [sample]> &/ <A1 --> [left]>) &/ ^left) =/> G>?", Print=loud)
NAR.AddInput("<((<A1 --> [sample]> &/ <A1 --> [right]>) &/ ^right) =/> G>?", Print=loud)
NAR.AddInput("<((<A2 --> [sample]> &/ <A2 --> [left]>) &/ ^left) =/> G>?", Print=loud)
NAR.AddInput("<((<A2 --> [sample]> &/ <A2 --> [right]>) &/ ^right) =/> G>?", Print=loud)
NAR.AddInput("<((<#1 --> [sample]> &/ <#1 --> [left]>) &/ ^left) =/> G>?", Print=loud)
NAR.AddInput("<((<#1 --> [sample]> &/ <#1 --> [right]>) &/ ^right) =/> G>?", Print=loud)
printloud("\x1b[0m")

print("Identity matching task results:")
print("correctness ratio average\t%f\naverage specific confidence\t%f\naverage generics confidence\t%f" % (sum(corrects)/float(len(corrects)), sum(specifics)/float(len(specifics)), sum(generics)/float(len(generics))))
