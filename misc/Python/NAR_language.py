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
import json
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
                        #whether the word refers to a relational concept or not and if we want it so
                        _, truthRel, _ = Query(f"<{assignment} --> RELATION>")
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
            return (unifier[0][1], i, truth, word)
    return ITEM

def getNounRelNoun(words):
    EMPTY = (None, -1, (0.5,0.0), "")
    RELATION = (None, -1, (0.5,0.0), "")
    SUBJECT = (None, -1, (0.5,0.0), "")
    OBJECT = (None, -1, (0.5,0.0), "")
    SUBJECT_MOD = (None, -1, (0.5,0.0), "")
    OBJECT_MOD = (None, -1, (0.5,0.0), "")
    for i, word in enumerate(words):
        _, truthAssigned, _ = Query(f"<{word} --> [ASSIGNED]>")
        if Truth_Expectation(truthAssigned) < 0.1:
            #whether this word was used to refer to a concept, else we don't consider it
            print("//Unassigned1:", word, Truth_Expectation(truthAssigned))
            continue
        RELATION_TEMP = resolveViaChoice(word, i, RELATION, isRelation=True)
        quRelation = Query(f"<({word} * ?1) --> R>", isRelation=True)
        quConcept = Query(f"<({word} * ?1) --> R>", isRelation=False)
        if Truth_Expectation(quRelation[1]) > Truth_Expectation(quConcept[1]):
            RELATION = (quRelation[2][0][1], i, quRelation[1], word)
            break
        RELATION = RELATION_TEMP
    if RELATION[0] is None:
        return (None, None, None)
    VALUES = []
    for j, word in enumerate(words):
        if j == RELATION[1]:
            continue
        _, truthAssigned, _ = Query(f"<{word} --> [ASSIGNED]>")
        if Truth_Expectation(truthAssigned) < 0.1:
            #whether this word was used to refer to a concept, else we don't consider it
            print("//Unassigned2:", word, Truth_Expectation(truthAssigned))
            continue
        VALUE = resolveViaChoice(word, j, (None, -1, (0.5,0.0), ""), isRelation=False)
        if VALUE[0] is not None:
            VALUES.append(VALUE)
    if len(VALUES) == 2:
        SUBJECT = VALUES[0]
        OBJECT = VALUES[1]
    if len(VALUES) == 4:
        SUBJECT_MOD = VALUES[0]
        SUBJECT = VALUES[1]
        OBJECT_MOD = VALUES[2]
        OBJECT = VALUES[3]
    if len(VALUES) == 3:
        if VALUES[0][0][0] == '[':
            SUBJECT_MOD = VALUES[0]
            SUBJECT = VALUES[1] #modifiers are symmetric anyway
            OBJECT = VALUES[2]
        elif VALUES[1][0][0] == '[':
            SUBJECT = VALUES[0]
            OBJECT_MOD = VALUES[1]
            OBJECT = VALUES[2]
    if SUBJECT[3] != "":
        AddBelief(f"<{SUBJECT[3]} --> [ASSIGNED]>")
    if RELATION[3] != "":
        AddBelief(f"<{RELATION[3]} --> [ASSIGNED]>")
    if OBJECT[3] != "":
        AddBelief(f"<{OBJECT[3]} --> [ASSIGNED]>")
    if SUBJECT_MOD[3] != "":
        AddBelief(f"<{SUBJECT_MOD[3]} --> [ASSIGNED]>")
    if OBJECT_MOD[3] != "":
        AddBelief(f"<{OBJECT_MOD[3]} --> [ASSIGNED]>")
    for x in words:
        cond1 = x != SUBJECT[3] and x != RELATION[3] and x != OBJECT[3]
        cond2 = x != SUBJECT_MOD[3] and x != OBJECT_MOD[3]
        if cond1 and cond2:
            AddBelief(f"<{x} --> [ASSIGNED]>", (0.0, 0.9))
    modify = lambda a,b: a[0] if b[0] is None else (f"({b[0]} & {a[0]})" if b[0][0] == '[' else f"({b[0]} * {a[0]})")
    S, R, O = (modify(SUBJECT, SUBJECT_MOD), RELATION[0], modify(OBJECT, OBJECT_MOD))
    print("//SRO: ", (S, R, O))
    return (S, R, O)

def produceSentenceNarsese(words):
    S,R,O = getNounRelNoun(words)
    if S is None or R is None or O is None:
        return
    if Truth_Expectation(Query(f"<{R} --> [FLIPPED]>")[1]) > 0.5:
        temp = O
        O = S
        S = temp
    if R == "IS":
        NAR.AddInput(f"<{S} --> {O}>. :|:")
    elif R == "LIKE":
        NAR.AddInput(f"<{S} <-> {O}>. :|:")
    else:
        NAR.AddInput(f"<({S} * {O}) --> {R}>. :|:")

global words
def newSentence(sentence):
    global words
    words = sentence.split(" ")
    #print("//WORDS: ", words)
    if not Training:
        produceSentenceNarsese(words)

def newConcept(term):
    global SUBJECT, RELATION, OBJECT, Training
    if "-->" not in term and "<->" not in term:
        return
    copula = "-->"
    if "<->" in term:
        copula = "<->"
    subject = term.split(f" {copula}")[0][1:]
    predicate = term.split(f"{copula} ")[1].replace(" :|:","")[:-1]
    if "*" in subject:
        RELATION = predicate
        SUBJECT = subject.split(" * ")[0][1:]
        OBJECT = subject.split(" * ")[1][:-1]
    else:
        SUBJECT = subject
        RELATION = "IS" if copula == "-->" else "LIKE"
        OBJECT = predicate
    AddBelief("<" + SUBJECT + " --> RELATION>", (0.0, 0.9))
    AddBelief("<" + OBJECT + " --> RELATION>", (0.0, 0.9))
    AddBelief("<" + RELATION + " --> RELATION>")
    print("//SRO:", (SUBJECT, RELATION, OBJECT))
    if not Training:
        NAR.AddInput(term + ". :|:")

def correlate():
    for x in words:
        for y in [SUBJECT, RELATION, OBJECT]:
            AddBelief(f"<({x} * {y}) --> R>")
    S,R,O = getNounRelNoun(words)
    if S is not None and R is not None and O is not None and S == OBJECT and O == SUBJECT:
        print("//Grammatical flip detected", S, R, O, SUBJECT, RELATION, OBJECT)
        AddBelief(f"<{RELATION} --> [FLIPPED]>")

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

Training=False
fname = "language.json"
if __name__ == "__main__":
    while True:
        try:
            inp = input().rstrip("\n")
        except:
            exit(0)
        if inp.startswith("//"):
            print(inp)
            continue
        if inp.startswith("*reset"):
            memory = {}
            continue
        if inp.startswith("*save"):
            with open(fname, 'w') as f:
                json.dump(memory, f)
        if inp.startswith("*load"):
            memory = {}
            if exists(fname):
                with open(fname) as json_file:
                    memory = json.load(json_file)
            continue
        if inp.startswith("*train=true"):
            TrainStart()
            continue
        elif inp.startswith("*train=false"):
            TrainEnd()
            continue
        elif inp.startswith("*"):
            print(inp)
            continue
        processInput(inp)



