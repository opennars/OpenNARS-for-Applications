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

conditions = [["<A1 --> [sample]>. :|:", "<B1 --> [left]>. :|:", "<B2 --> [right]>. :|:", "^left"],
               ["<A1 --> [sample]>. :|:", "<B2 --> [left]>. :|:", "<B1 --> [right]>. :|:", "^right"],
              ["<A2 --> [sample]>. :|:", "<B1 --> [left]>. :|:",
                  "<B2 --> [right]>. :|:", "^right"],
              ["<A2 --> [sample]>. :|:", "<B2 --> [left]>. :|:",
                  "<B1 --> [right]>. :|:", "^left"]]

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
                # NAR.AddInput("G. :|: {0.0 0.9}", Print=loud)      

        NAR.AddInput("100", Print=False)
    
    printloud("\x1B[34mCorrect = " + str(correct/4) + "\x1B[0m")

    printloud("End of block")
    printloud()

printloud("\x1b[33m")
NAR.AddInput("<((<A1 --> [sample]> &/ <B1 --> [left]>) &/ ^left) =/> G>?", Print=loud)
NAR.AddInput("<((<A1 --> [sample]> &/ <B1 --> [right]>) &/ ^right) =/> G>?", Print=loud)
NAR.AddInput("<((<A2 --> [sample]> &/ <B2 --> [left]>) &/ ^left) =/> G>?", Print=loud)
NAR.AddInput("<((<A2 --> [sample]> &/ <B2 --> [right]>) &/ ^right) =/> G>?", Print=loud)
printloud("\x1b[0m")


seq = [0, 1, 2, 3]
printloud("\x1b[35mLearning A1->B1, and A2->B2\x1b[0m")
printloud("\x1b[35m***************\x1b[0m")
printloud()

for b in range(20):
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

        NAR.AddInput("100", Print=False)

    printloud("\x1b[33m")
    response = NAR.AddInput("<((<A1 --> [sample]> &/ <B1 --> [left]>) &/ ^left) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        s1 = float(c)
    else:
        s1 = 0

    response = NAR.AddInput("<((<A1 --> [sample]> &/ <B1 --> [right]>) &/ ^right) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        s2 = float(c)
    else:
        s2 = 0

    response = NAR.AddInput("<((<A2 --> [sample]> &/ <B2 --> [left]>) &/ ^left) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        s3 = float(c)
    else:
        s3 = 0

    response = NAR.AddInput("<((<A2 --> [sample]> &/ <B2 --> [right]>) &/ ^right) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        s4 = float(c)
    else:
        s4 = 0

    printloud("\x1b[0m")

    specific = (s1 + s2 + s3 + s4)/4

    printloud("specific " + str(specific))

    printloud("\x1B[34mCorrect " + str(correct/4) + "\x1B[0m")

    printloud("End of block")
    printloud()


seq = [0, 1, 2, 3]
printloud("\x1b[35mTESTING A1->B1, and A2->B2\x1b[0m")
printloud("\x1b[35m***************\x1b[0m")
printloud()
corrects = 0
incorrects = 0

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
                corrects += 1
            elif op != expected_op:
                printloud("\x1b[31mINCORRECT: " + op +"\x1b[0m")
                incorrects += 1
        else:
            printloud("\x1b[31mINCORRECT: " + op +"\x1b[0m")
            incorrects += 1
        NAR.AddInput("100", Print=False)

    # printloud("\x1b[33m")
    # response = NAR.AddInput("<((<A1 --> [sample]> &/ <A1 --> [left]>) &/ ^left) =/> G>?", Print=False)
    # answers = response["answers"]
    # if answers[0].get("truth"):
    #     c = answers[0]["truth"]["confidence"]
    #     s1 = float(c)
    # else:
    #     s1 = 0

    # response = NAR.AddInput("<((<A1 --> [sample]> &/ <A1 --> [right]>) &/ ^right) =/> G>?", Print=False)
    # answers = response["answers"]
    # if answers[0].get("truth"):
    #     c = answers[0]["truth"]["confidence"]
    #     s2 = float(c)
    # else:
    #     s2 = 0

    # response = NAR.AddInput("<((<A2 --> [sample]> &/ <A2 --> [left]>) &/ ^left) =/> G>?", Print=False)
    # answers = response["answers"]
    # if answers[0].get("truth"):
    #     c = answers[0]["truth"]["confidence"]
    #     s3 = float(c)
    # else:
    #     s3 = 0

    # response = NAR.AddInput("<((<A2 --> [sample]> &/ <A2 --> [right]>) &/ ^right) =/> G>?", Print=False)
    # answers = response["answers"]
    # if answers[0].get("truth"):
    #     c = answers[0]["truth"]["confidence"]
    #     s4 = float(c)
    # else:
    #     s4 = 0

    # printloud("\x1b[0m")

    # specific = (s1 + s2 + s3 + s4)/4

    # printloud("specific " + str(specific))

    # printloud("\x1B[34mCorrect " + str(correct/4) + "\x1B[0m")

    printloud("End of block")
    printloud()

# printloud("\x1b[33m")
# NAR.AddInput("<((<A1 --> [sample]> &/ <A1 --> [left]>) &/ ^left) =/> G>?", Print=loud)
# NAR.AddInput("<((<A1 --> [sample]> &/ <A1 --> [right]>) &/ ^right) =/> G>?", Print=loud)
# NAR.AddInput("<((<A2 --> [sample]> &/ <A2 --> [left]>) &/ ^left) =/> G>?", Print=loud)
# NAR.AddInput("<((<A2 --> [sample]> &/ <A2 --> [right]>) &/ ^right) =/> G>?", Print=loud)
# printloud("\x1b[0m")

print("Conditioning task results:")
print("Correct\t\t" + str(corrects))
print("Incorrect\t" + str(incorrects))
