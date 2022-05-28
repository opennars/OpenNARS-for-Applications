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
NAR.AddInput("*setopname 1 ^clap", Print=loud)
NAR.AddInput("*setopname 2 ^wave", Print=loud)

conditions = [["B1. :|:", "^clap"],
              ["B2. :|:", "^wave"]]

test_conditions = [["C1. :|:", "^clap"],
                   ["C2. :|:", "^wave"]]

printloud("\x1B[34mTraining\x1B[0m")
printloud("\x1B[34m***************\x1B[0m")

seq = [0, 1]
for _ in range(16):
    s = random.sample(seq, 2)

    for i in s:
        trial = conditions[i]
        sample = trial[0]
        expected_op = trial[1]

        NAR.AddInput(sample, Print=False)

        response = NAR.AddInput("G! :|:", Print=loud)
        executions = response["executions"]
        if executions:
            op = executions[0]["operator"]

            if op == expected_op:
                printloud("\x1B[32mCORRECT: " + op + "\x1B[0m")
                NAR.AddInput("G. :|:", Print=False)
            elif op != expected_op:
                printloud("\x1B[31mINCORRECT: " + op + "\x1B[0m")

        NAR.AddInput("100", Print=loud)

# NAR.AddInput("<(B1 &/ ^clap) =/> G>.")
# NAR.AddInput("<(B2 &/ ^wave) =/> G>.")

printloud("\x1B[34mTesting\x1B[0m")
printloud("\x1B[34m***************\x1B[0m")

NAR.AddInput("<A1 <=> B1>.", Print=loud)
NAR.AddInput("<A1 <=> C1>.", Print=loud)
NAR.AddInput("<A2 <=> B2>.", Print=loud)
NAR.AddInput("<A2 <=> C2>.", Print=loud)

seq = [0, 1]
exec = 0
noexec = 0
correct = 0
incorrect = 0
for _ in range(5):
    s = random.sample(seq, 2)

    for i in s:
        trial = test_conditions[i]
        sample = trial[0]
        expected_op = trial[1]

        NAR.AddInput(sample, Print=loud)

        response = NAR.AddInput("G! :|:", Print=loud)
        executions = response["executions"]
        if executions:
            op = executions[0]["operator"]
            exec += 1

            if op == expected_op:
                printloud("\x1B[32mCORRECT: " + op + "\x1B[0m")
                correct += 1
#                NAR.AddInput("G. :|:", Print=False)
            elif op != expected_op:
                printloud("\x1B[31mINCORRECT: " + op + "\x1B[0m")
                incorrect += 1
        else:
            noexec +=1

        NAR.AddInput("100", Print=loud)

print("Discriminative function task results:")
print("Executions\t" + str(exec))
print("No executions\t" + str(noexec))
print("Correct\t\t" + str(correct))
print("Incorrect\t" + str(incorrect))
