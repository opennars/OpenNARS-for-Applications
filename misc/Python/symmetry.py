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

#################################################################################
symmetry = [([             "<A1 --> [sample]>. :|:", #A
"<B1 --> [left]>. :|:",                                  "<B2 --> [right]>. :|:", 
"^left"],
#################################################################################
            [              "<B1 --> [sample]>. :|:", #A'
"<A1 --> [left]>. :|:",                                  "<A2 --> [right]>. :|:", 
"^left"]),
#################################################################################
            ([             "<A1 --> [sample]>. :|:", #B
"<B2 --> [left]>. :|:",                                 "<B1 --> [right]>. :|:", 
                                                         "^right"],
#################################################################################
            [              "<B1 --> [sample]>. :|:", #B'
"<A2 --> [left]>. :|:",                                 "<A1 --> [right]>. :|:", 
                                                        "^right"]),
#################################################################################
            ([              "<A2 --> [sample]>. :|:", #C
"<B1 --> [left]>. :|:",                                 "<B2 --> [right]>. :|:", 
                                                        "^right"],
#################################################################################
            [               "<B2 --> [sample]>. :|:", #C'
"<A1 --> [left]>. :|:",                                 "<A2 --> [right]>. :|:", 
                                                        "^right"]),
#################################################################################
            ([              "<A2 --> [sample]>. :|:", #D
"<B2 --> [left]>. :|:",                                 "<B1 --> [right]>. :|:", 
"^left"],
#################################################################################
            [               "<B2 --> [sample]>. :|:", #D'
"<A2 --> [left]>. :|:",                                "<A1 --> [right]>. :|:", 
"^left"])]
#################################################################################


correct = 0
printloud("\x1b[35mLearning A1->B1, and B1->A1\x1b[0m")
printloud("\x1b[35m***************\x1b[0m")
printloud()


def AddExample(sample, left, right, expected_op, correct, Print=loud):
    NAR.AddInput(sample, Print=Print)
    NAR.AddInput(left, Print=Print)
    NAR.AddInput(right, Print=Print)

    response = NAR.AddInput("G! :|:", Print=Print)
    executions = response["executions"]
    if executions:
        op = executions[0]["operator"]
        printloud("Input: " + str(op) + ". :|:")
        NAR.AddInput("*volume=0", Print=Print and loud)
        if op == expected_op:
            printloud("\x1B[32mCORRECT: " + op + "\x1B[0m")
            NAR.AddInput("G. :|:", Print=Print and loud)
            correct += 1
        elif op != expected_op:
            printloud("\x1B[31mINCORRECT: " + op + "\x1B[0m")
            NAR.AddInput("G. :|: {0.0 0.9}", Print=Print and loud)      
    else:
        printloud("Input: NO EXECUTION")
    NAR.AddInput("50", Print=Print and loud)
    return correct


NAR.AddInput("*volume=0", Print=loud)
for b in range(10):
   # break #DISABLED LOOP!!!!!!!!!
    printloud("\x1b[35mStarting block " + str(b) + ".\x1b[0m")
    printloud()
    correct = 0

    for i in random.sample(range(len(symmetry)), len(symmetry)):
        trial = symmetry[i]
        sample_original, sample_reverse = (trial[0][0], trial[1][0])
        left_original, left_reverse = (trial[0][1], trial[1][1])
        right_original, right_reverse = (trial[0][2], trial[1][2])
        expected_op_original, expected_op_reverse = (trial[0][3], trial[1][3])

        #original:
        correct = AddExample(sample_original, left_original, right_original, expected_op_original, correct)
        correct = AddExample(sample_reverse, left_reverse, right_reverse, expected_op_reverse, correct)

    printloud("\x1B[34mCorrect " + str(correct/4) + "\x1B[0m")

    printloud("End of block")
    printloud()
    NAR.AddInput("100", Print=loud)


printloud("\x1b[33m")
NAR.AddInput("<(((<$1 --> [sample]> &/ <$2 --> [left]>) &/ ^left) &/ G) ==> <((<$2 --> [sample]> &/ <$1 --> [left]>) &/ ^left) =/> G>>?", Print=loud)
NAR.AddInput("<(((<$1 --> [sample]> &/ <$2 --> [right]>) &/ ^right) &/ G) ==> <((<$2 --> [sample]> &/ <$1 --> [right]>) &/ ^right) =/> G>>?", Print=loud)

printloud("\x1b[0m")


#################################################################################
testsymmetry = [([         "<X1 --> [sample]>. :|:", #A
"<Y1 --> [left]>. :|:",                                  "<Y2 --> [right]>. :|:", 
"^left"],
#################################################################################
            [              "<Y1 --> [sample]>. :|:", #A'
"<X1 --> [left]>. :|:",                                  "<X2 --> [right]>. :|:", 
"^left"]),
#################################################################################
            ([             "<X1 --> [sample]>. :|:", #B
"<Y2 --> [left]>. :|:",                                 "<Y1 --> [right]>. :|:", 
                                                         "^right"],
#################################################################################
            [              "<Y1 --> [sample]>. :|:", #B'
"<X2 --> [left]>. :|:",                                 "<X1 --> [right]>. :|:", 
                                                        "^right"]),
#################################################################################
            ([              "<X2 --> [sample]>. :|:", #C
"<Y1 --> [left]>. :|:",                                 "<Y2 --> [right]>. :|:", 
                                                        "^right"],
#################################################################################
            [               "<Y2 --> [sample]>. :|:", #C'
"<X1 --> [left]>. :|:",                                 "<X2 --> [right]>. :|:", 
                                                        "^right"]),
#################################################################################
            ([              "<X2 --> [sample]>. :|:", #D
"<Y2 --> [left]>. :|:",                                 "<Y1 --> [right]>. :|:", 
"^left"],
#################################################################################
            [               "<Y2 --> [sample]>. :|:", #D'
"<X2 --> [left]>. :|:",                                "<X1 --> [right]>. :|:", 
"^left"])]
#################################################################################

printloud("\x1b[35mLearning A1->B1, and A2->B2\x1b[0m")
printloud("\x1b[35m***************\x1b[0m")
printloud()

for z in range(5):
    printloud("\x1b[35mStarting block " + str(b) + ".\x1b[0m")
    #printloud()
    #s = random.sample(seq, 4)
    correct = 0


    for b in range(len(testsymmetry)):
        trial = testsymmetry[b]
        sample_original, sample_reverse = (trial[0][0], trial[1][0])
        left_original, left_reverse = (trial[0][1], trial[1][1])
        right_original, right_reverse = (trial[0][2], trial[1][2])
        expected_op_original, expected_op_reverse = (trial[0][3], trial[1][3])

        NAR.AddInput(sample_original, Print=False)
        NAR.AddInput(left_original, Print=False)
        NAR.AddInput(right_original, Print=False)

        response = NAR.AddInput("G! :|:", Print=loud)
        executions = response["executions"]
        if executions:
            op = executions[0]["operator"]

            if op == expected_op_original:
                printloud("\x1B[32mCORRECT: " + op + "\x1B[0m")
                NAR.AddInput("G. :|:", Print=False)
                correct += 1
            elif op != expected_op_original:
                printloud("\x1B[31mINCORRECT: " + op + "\x1B[0m")
                NAR.AddInput("G. :|: {0.0 0.9}", Print=False)      

        NAR.AddInput("100", Print=False)

    printloud("\x1b[33m")
    response = NAR.AddInput("<((<X1 --> [sample]> &/ <Y1 --> [left]>) &/ ^left) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        s1 = float(c)
    else:
        s1 = 0

    response = NAR.AddInput("<((<X1 --> [sample]> &/ <Y1 --> [right]>) &/ ^right) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        s2 = float(c)
    else:
        s2 = 0

    response = NAR.AddInput("<((<X2 --> [sample]> &/ <Y2 --> [left]>) &/ ^left) =/> G>?", Print=False)
    answers = response["answers"]
    if answers[0].get("truth"):
        c = answers[0]["truth"]["confidence"]
        s3 = float(c)
    else:
        s3 = 0

    response = NAR.AddInput("<((<X2 --> [sample]> &/ <Y2 --> [right]>) &/ ^right) =/> G>?", Print=False)
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

correct_total = 0
printloud("Start testing")
correct = 0

for b in range(len(testsymmetry)):
    trial = testsymmetry[b]
    sample_original, sample_reverse = (trial[0][0], trial[1][0])
    left_original, left_reverse = (trial[0][1], trial[1][1])
    right_original, right_reverse = (trial[0][2], trial[1][2])
    expected_op_original, expected_op_reverse = (trial[0][3], trial[1][3])

    #original:
    correct_temp = correct
    NAR.AddInput("*motorbabbling=false", Print=False)
    #correct = AddExample(sample_original, left_original, right_original, expected_op_original, correct, Print=False)
    if True: #correct > correct_temp:
        #the choice was correct
        NAR.AddInput("100", Print=False)
        NAR.AddInput("<((<Y1 --> [sample]> &/ <X1 --> [left]>) &/ ^left) =/> G>?", Print=loud)
        NAR.AddInput("<(((<$1 --> [sample]> &/ <$2 --> [left]>) &/ ^left) &/ G) ==> <((<$2 --> [sample]> &/ <$1 --> [left]>) &/ ^left) =/> G>>?", Print=loud)
        NAR.AddInput("(((<X1 --> [sample]> &/ <Y1 --> [left]>) &/ ^left) &/ G)? :|:", Print=loud)
        correct_temp = correct
        NAR.AddInput("*motorbabbling=false", Print=False)
        correct = AddExample(sample_reverse, left_reverse, right_reverse, expected_op_original, correct)
        if correct > correct_temp:
            correct_total += 1

print("Symmetry task results:")
print("Correctness ratio", correct_total/4)
