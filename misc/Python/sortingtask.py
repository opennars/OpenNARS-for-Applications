from math import exp
import NAR
import random
import sys
import time

silent = "silent" in sys.argv
seed=42
for x in sys.argv:
    if x.startswith("seed="):
       seed = int(x.split("seed=")[1]) 
random.seed(seed)
NAR.AddInput("*volume=0", Print=not silent)
NAR.AddInput("*motorbabbling=0.4", Print=not silent)
NAR.AddInput("*setopname 1 ^left", Print=not silent)
NAR.AddInput("*setopname 2 ^mid", Print=not silent)
NAR.AddInput("*setopname 3 ^right", Print=not silent)

conditions = [["boh. :|:", "^left"],
              ["gax. :|:", "^mid"],
              ["sol. :|:", "^right"]]

NAR.AddInput("<A1 <=> B1>.", Print=not silent)
NAR.AddInput("<A1 <=> C1>.", Print=not silent)

seq = [0, 1, 2]
correct_cnt = 0
incorrect_cnt = 0
noexecutions_cnt = 0
executions_cnt = 0

for b in range(20):
    s = random.sample(seq, 3)

    exec = 0
    noexec = 0
    correct = 0
    incorrect = 0

    for i in s:
        trial = conditions[i]
        sample = trial[0]
        expected_op = trial[1]

        NAR.AddInput(sample, Print=False)

        response = NAR.AddInput("B1! :|:", Print=False)
        executions = response["executions"]
        if executions:
            exec += 1
            op = executions[0]["operator"]

            if op == expected_op:
                correct += 1
                if not silent:
                    print("\x1B[32mCORRECT: " + op + "\x1B[0m")
                NAR.AddInput("C1. :|:", Print=False)
            elif op != expected_op:
                incorrect += 1
                if not silent:
                    print("\x1B[31mINCORRECT: " + op + "\x1B[0m")
        else:
            noexec += 1
        NAR.AddInput("100", Print=not silent)

        response = NAR.AddInput("<(boh &/ ^left) =/> B1>?", Print=False)
        term = response["answers"][0]["term"]

        if not silent:
            if term != "None":
                f = response["answers"][0]["truth"]["frequency"]
                c = response["answers"][0]["truth"]["confidence"]
                print("\x1B[34m<(boh &/ ^left) =/> B1>?" + "\x1B[0m")
                print("\x1B[34mf: " + f + "\x1B[0m")
                print("\x1B[34mc: " + c + "\x1B[0m")
                print()

            response = NAR.AddInput("<(gax &/ ^mid) =/> B1>?", Print=False)
            term = response["answers"][0]["term"]

            if term != "None":
                f = response["answers"][0]["truth"]["frequency"]
                c = response["answers"][0]["truth"]["confidence"]
                print("\x1B[34m<(gax &/ ^mid) =/> B1>?" + "\x1B[0m")
                print("\x1B[34mf: " + f + "\x1B[0m")
                print("\x1B[34mc: " + c + "\x1B[0m")
                print()

            response = NAR.AddInput("<(sol &/ ^right) =/> B1>?", Print=False)
            term = response["answers"][0]["term"]

            if term != "None":
                f = response["answers"][0]["truth"]["frequency"]
                c = response["answers"][0]["truth"]["confidence"]
                print("\x1B[34m<(sol &/ ^right) =/> B1>?" + "\x1B[0m")
                print("\x1B[34mf: " + f + "\x1B[0m")
                print("\x1B[34mc: " + c + "\x1B[0m")
                print()

    executions_cnt += exec
    noexecutions_cnt += noexec
    correct_cnt += correct
    incorrect_cnt += incorrect
    if not silent:
        print("\x1B[35mBlock: " + str(b) + "\x1B[0m")
        print("\x1B[35mExecutions " + str(exec) + "\x1B[0m")
        print("\x1B[35mNo executions " + str(noexec) + "\x1B[0m")
        print("\x1B[35mCorrect " + str(correct) + "\x1B[0m")
        print("\x1B[35mIncorrect " + str(incorrect) + "\x1B[0m")
        print("\x1B[35m***************\x1B[0m")
    
print("Word sorting task results:")
print("Total executions " + str(executions_cnt))
print("Total no executions " + str(noexecutions_cnt))
print("Total correct " + str(correct_cnt))
print("Total incorrect " + str(incorrect_cnt))
