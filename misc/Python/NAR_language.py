"""
 * The MIT License
 *
 * Copyright 2023 The OpenNARS authors.
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
NAR.AddInput("*volume=0")

#NAL truth functions
def Truth_w2c(w):
    return w / (w + 1.0)

def Truth_c2w(c):
    return c / (1.0 - c)

def Truth_Expectation(v):
    return (v[1] * (v[0] - 0.5) + 0.5)

def Truth_Negation(v):
    return (1-v[0], v[1])

def Truth_Revision(v1, v2):
    (f1, c1) = v1
    (f2, c2) = v2
    w1 = Truth_c2w(c1)
    w2 = Truth_c2w(c2)
    w = w1 + w2
    return (min(1.0, (w1 * f1 + w2 * f2) / w), 
            min(0.99, max(max(Truth_w2c(w), c1), c2)))
#NAL truth functions end

#Basic NARS memory
memory = {}
def AddBelief(belief, Truth=(1.0, 0.9)):
    global memory
    if belief in memory:
        memory[belief] = Truth_Revision(memory[belief], Truth)
    else:
        memory[belief] = Truth

def Query(term, isRelation=None):
    if "?1" in term: #simple query matching
            parts = term.split("?1")
            truth_expectation = lambda f,c: (c * (f - 0.5) + 0.5)
            bestTerm, bestTruth = (None, (0.0, 0.5))
            bestAssignment = ""
            for term2 in memory:
                if term2.startswith(parts[0]) and term2.endswith(parts[1]):
                    assignment = term2[len(parts[0]):-len(parts[1])]
                    if isRelation is not None:
                        termRel, truthRel, _ = Query(f"<{assignment} --> RELATION>")
                        if not isRelation:
                            truthRel = Truth_Negation(truthRel)
                        if Truth_Expectation(truthRel) <= 0.5:
                            continue
                    (f2, c2) = memory[term2]
                    if truth_expectation(f2, c2) > truth_expectation(bestTruth[0], bestTruth[1]):
                        bestAssignment = ("?1", assignment)
                        bestTerm = term2
                        bestTruth = (f2, c2)
            if bestTerm is not None:
                return bestTerm, bestTruth, [bestAssignment]
    else:
        if term in memory:
            return term, memory[term], []
    return term, (0.5, 0.0), []
#Basic NARS memory end

def resolveViaChoice(word, i, ITEM, isRelation):
    term, truth, unifier = Query(f"<({word} * ?1) --> R>", isRelation)
    if unifier:
        if Truth_Expectation(truth) >= Truth_Expectation(ITEM[2]):
            return (unifier[0][1], i, truth)
    return ITEM

def getNounRelNoun(words):
    RELATION = (None, -1, (0.5,0.0))
    SUBJECT = (None, -1, (0.5,0.0))
    OBJECT = (None, -1, (0.5,0.0))
    for i, word in enumerate(words):
        RELATION = resolveViaChoice(word, i, RELATION, isRelation=True)
    if RELATION is None:
        return (None, None, None)
    words.pop(RELATION[1])
    for j, word in enumerate(words):
        VALUE = resolveViaChoice(word, j, (None, -1, (0.5,0.0)), isRelation=False)
        if Truth_Expectation(VALUE[2]) > Truth_Expectation(SUBJECT[2]):
            OBJECT = SUBJECT
            SUBJECT = VALUE
        elif Truth_Expectation(VALUE[2]) > Truth_Expectation(OBJECT[2]):
            OBJECT = VALUE
    #flip S and O if S is after O (SO permutation is handled separately)
    if SUBJECT[1] > OBJECT[1]:
        temp = SUBJECT
        SUBJECT = OBJECT
        OBJECT = SUBJECT
    print(">>>",SUBJECT[0], RELATION[0], OBJECT[0])
    return (SUBJECT[0], RELATION[0], OBJECT[0])

def produceSentenceNarsese(words):
    S,R,O = getNounRelNoun(words)
    if S is None or R is None or O is None:
        return
    if Truth_Expectation(Query(f"<{R} --> [flipped]>")[1]) > 0.5:
        temp = O
        O = S
        S = temp
    if R == "IS":
        NAR.AddInput(f"<{S} --> {O}>. :|:")
    else:
        NAR.AddInput(f"<({S} * {O}) --> {R}>. :|:")

global words
def newSentence(sentence):
    global words
    words = sentence.split(" ")
    print("//WORDS: ", words)
    if not Training:
        produceSentenceNarsese(words)

def newConcept(term):
    global SUBJECT, RELATION, OBJECT, Training
    if "-->" not in term:
        return
    subject = term.split(" -->")[0][1:]
    predicate = term.split("--> ")[1].replace(" :|:","")[:-1]
    if "*" in subject:
        RELATION = predicate
        SUBJECT = subject.split(" * ")[0][1:]
        OBJECT = subject.split(" * ")[1][:-1]
    else:
        SUBJECT = subject
        RELATION = "IS"
        OBJECT = predicate
    AddBelief("<" + SUBJECT + " --> RELATION>", (0.0, 0.9))
    AddBelief("<" + OBJECT + " --> RELATION>", (0.0, 0.9))
    AddBelief("<" + RELATION + " --> RELATION>")
    print("//SRO:", [SUBJECT, RELATION, OBJECT])
    if not Training:
        NAR.AddInput(term + ". :|:")

def correlate():
    for x in words:
        for y in [SUBJECT, RELATION, OBJECT]:
            AddBelief(f"<({x} * {y}) --> R>")
    S,R,O = getNounRelNoun(words)
    print(S, R, O, SUBJECT, RELATION, OBJECT)
    if S is not None and R is not None and O is not None and S == OBJECT and O == SUBJECT:
        print("Grammatical flip detected", S, R, O, SUBJECT, RELATION, OBJECT)
        AddBelief(f"<{RELATION} --> [flipped]>")

def processInput(inp):
    print("//Input: " + inp)
    if inp.isdigit():
        correlate()
        return
    if inp.startswith("<") or inp.startswith("("):
        newConcept(inp[:-1])
    else:
        newSentence(inp)

def TrainStart():
    global Training
    print("//Training Start")
    Training = True

def TrainEnd():
    global Training
    print("//Training End")
    Training = False

def TrainOnData():
    TrainStart()
    processInput("<HUMAN --> [LEFT]>.")
    processInput("human is left")
    processInput("1")
    processInput("<HUMAN --> [RIGHT]>.")
    processInput("human is right")
    processInput("1")
    processInput("<HUMAN --> [FRONT]>.")
    processInput("human in front")
    processInput("1")
    processInput("<BOX --> [RIGHT]>.")
    processInput("box to the right")
    processInput("1")
    processInput("<BALL --> [RIGHT]>.")
    processInput("ball to the right")
    processInput("1")
    processInput("<BOX --> [LEFT]>.")
    processInput("box to the left")
    processInput("1")
    TrainEnd()

def Test1():
    TrainOnData()
    print(Query(f"<(human * ?1) --> R>", isRelation=False))
    #Output: ('<(human * HUMAN) --> R>', (1.0, 0.9642857142857143), [('?1', 'HUMAN')])
    print(Query(f"<(right * ?1) --> R>", isRelation=False))
    #Output: ('<(right * [RIGHT]) --> R>', (1.0, 0.9642857142857143), [('?1', '[RIGHT]')])
    print(Query(f"<(left * ?1) --> R>", isRelation=False))
    #Output: ('<(left * [LEFT]) --> R>', (1.0, 0.9473684210526316), [('?1', '[LEFT]')])
    processInput("the human is to the left")
    #Output: <HUMAN --> [LEFT]>. :|:
    processInput("the human is to the right")
    #Output: <HUMAN --> [RIGHT]>. :|:

def Test2():
    TrainStart()
    processInput("<CAT --> [see]>.")
    processInput("cat")
    processInput("1")
    processInput("<CAT --> [hear]>.")
    processInput("cat")
    processInput("1")
    processInput("<HUMAN --> [LEFT]>.")
    processInput("left eser human")
    processInput("1")
    processInput("<HUMAN --> [RIGHT]>.")
    processInput("right eser human")
    processInput("1")
    processInput("<STONE --> [RIGHT]>.")
    processInput("right")
    processInput("1")
    processInput("<CAT --> [LEFT]>.")
    processInput("left eser cat")
    processInput("1")
    processInput("<HUMAN --> [RIGHT]>.")
    processInput("right eser human")
    processInput("1")
    TrainEnd()
    print(Query(f"<(human * ?1) --> R>", isRelation=False))
    #Output: ('<(human * HUMAN) --> R>', (1.0, 0.9642857142857143), [('?1', 'HUMAN')])
    print(Query(f"<(right * ?1) --> R>", isRelation=False))
    #Output: ('<(right * [RIGHT]) --> R>', (1.0, 0.9642857142857143), [('?1', '[RIGHT]')])
    print(Query(f"<(left * ?1) --> R>", isRelation=False))
    #Output: ('<(left * [LEFT]) --> R>', (1.0, 0.9473684210526316), [('?1', '[LEFT]')])
    processInput("left eser cat")
    processInput("right eser cat")

Training=False
if __name__ == "__main__":
    while True:
        try:
            inp = input().rstrip("\n")
        except:
            exit(0)
        if inp.startswith("//"):
            print(inp)
            continue
        if inp.startswith("*train=true"):
            TrainStart()
            continue
        elif inp.startswith("*train=false"):
            TrainEnd()
            continue
        elif inp.startswith("*train"):
            TrainOnData()
            continue
        elif inp.startswith("*test1"):
            Test1()
            continue
        elif inp.startswith("*test2"):
            Test2()
            continue
        elif inp.startswith("*"):
            print(inp)
            continue
        processInput(inp)



