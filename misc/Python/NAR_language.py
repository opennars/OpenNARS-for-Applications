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
import sys
NAR.AddInput("*volume=100")

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
    RELATIONS = []
    REL_FOUND = False
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
            RELATIONS.append((quRelation[2][0][1], i, quRelation[1], word))
        if not REL_FOUND:
            if Truth_Expectation(quRelation[1]) > Truth_Expectation(quConcept[1]):
                RELATION = (quRelation[2][0][1], i, quRelation[1], word)
                REL_FOUND = True
            else:
                RELATION = RELATION_TEMP
    if RELATION not in RELATIONS:
        RELATIONS = [RELATION] + RELATIONS
    if RELATION[0] is None:
        return [(None, None, None)]
    VALUES = []
    for j, word in enumerate(words):
        Continue = False
        for R in RELATIONS:
            if j == R[1]:
                Continue = True
        if Continue:
            continue
        _, truthAssigned, _ = Query(f"<{word} --> [ASSIGNED]>")
        if Truth_Expectation(truthAssigned) < 0.1:
            #whether this word was used to refer to a concept, else we don't consider it
            print("//Unassigned2:", word, Truth_Expectation(truthAssigned))
            continue
        VALUE = resolveViaChoice(word, j, (None, -1, (0.5,0.0), ""), isRelation=False)
        if VALUE[0] is not None:
            VALUES.append(VALUE)
    Cs = [] #concepts
    Ms = [] #modifiers (same len)
    nextmod = EMPTY
    nextmodmod = EMPTY
    ASSIGN = True
    for i,x in enumerate(VALUES + [None]):
        if x is None:
            if nextmod != EMPTY:
                Cs.append(nextmod)
                Ms.append(nextmodmod)
        elif x[0][0] == '[' or (i+1 < len(VALUES) and VALUES[i+1][1] - x[1] == 1 and x[3] + " " + VALUES[i+1][3] in sentence):
            if nextmod != EMPTY and x[0][0] == '[':
                nextmodmod = nextmod
                nextmod = x
            else:
                nextmod = x
        else:
            Cs.append(x)
            Ms.append(nextmod)
            nextmod = EMPTY
            nextmodmod = EMPTY
    if len(Cs) == 1 and len([x for x in Ms if x != EMPTY]) == 1: #just one concept and 1 modifier, so the concept being modified is the sentence
        if Ms[0][1] < Cs[0][1]:
            Cs = [Ms[0], Cs[0]]
        else:
            Cs = [Cs[0], Ms[0]]
        Ms = [EMPTY for i in range(2)]
    if ASSIGN:
        for x in Cs + Ms + [RELATION]:
            if x != EMPTY:
                AddBelief(f"<{x[3]} --> [ASSIGNED]>")
    for x in words:
        Break = False
        for y in Cs + Ms + [RELATION]:
            if x == y[3] and ASSIGN:
                Break = True
                break
        if Break:
            break
        AddBelief(f"<{x} --> [ASSIGNED]>", (0.0, 0.9))
    modify = lambda a,b: a[0] if b[0] is None else (f"({b[0]} & {a[0]})" if b[0][0] == '[' else f"({b[0]} * {a[0]})")
    if not ASSIGN or len(Cs) < 2:
        return [(None, None, None)]
    SROs = []
    print("//R,C,M: ", RELATIONS, Cs, Ms)
    for i in range(0, len(Cs)-1, 2):
        if int(i/2) >= len(RELATIONS):
            break
        S, R, O = (modify(Cs[i], Ms[i]), RELATIONS[int(i/2)][0], modify(Cs[i+1], Ms[i+1]))
        SROs.append((S, R, O))
    return SROs

def produceSentenceNarsese(words):
    SROs = getNounRelNoun(words)
    for (S,R,O) in SROs:
        if S is None or R is None or O is None:
            return
        if Truth_Expectation(Query(f"<{R} --> [FLIPPED]>")[1]) > 0.5:
            temp = O
            O = S
            S = temp
        if R == "IS":
            print("Input:", NAR.PrintedTask(NAR.AddInput(f"<{S} --> {O}>. :|:", Print=False)["input"][0]))
        elif R == "LIKE":
            print("Input:", NAR.PrintedTask(NAR.AddInput(f"<{S} <-> {O}>. :|:", Print=False)["input"][0]))
        else:
            print("Input:", NAR.PrintedTask(NAR.AddInput(f"<({S} * {O}) --> {R}>. :|:", Print=False)["input"][0]))

def sub_lists(l):
    lists = []
    for i in range(len(l) + 1):
        for j in range(i):
            subseq = l[j: i]
            lists.append((i-len(subseq), subseq, i))
    lists.sort(key=lambda x: x[0])
    return lists

sequenceMem=set([])
def findSequences(st):
    global sequenceMem
    sequenceMem.add(st)
    subsequences = sub_lists(st.split(" "))
    words = [" ".join(x[1]) for x in subsequences]
    startIndices = [x[0] for x in subsequences]
    endIndices = [x[2] for x in subsequences]
    sequences=[]
    minStartIndex=0
    for j,x in enumerate(words):
        if x in sequenceMem and startIndices[j] >= minStartIndex:
            sequences.append(x.replace(" ","_"))
            minStartIndex = endIndices[j]
    return sequences

localist_tokens = False
sentence = ""
def newSentence(s):
    global sentence, words, localist_tokens
    sentence = s
    if " " not in sentence:
        localist_tokens = True
    if localist_tokens and not "genericTokenization" in sys.argv:
        words = sentence.split(" ")
    else:
        words = findSequences(sentence)
    if not Training:
        produceSentenceNarsese(words)

def newConcept(term):
    global SUBJECT, RELATION, OBJECT, Training
    if "-->" not in term and "<->" not in term or "&&" in term or "==>" in term:
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

def correlate():
    global SUBJECT, RELATION, OBJECT, words
    print("//Cross-correlating: ", [SUBJECT, RELATION, OBJECT], "with", words)
    for x in words:
        for y in [SUBJECT, RELATION, OBJECT]:
            AddBelief(f"<({x} * {y}) --> R>")
    SROs = getNounRelNoun(words)
    for (S,R,O) in SROs:
        if S is not None and R is not None and O is not None:
            if S == OBJECT and O == SUBJECT:
                print("//Grammatical relation flip detected", S, R, O, SUBJECT, RELATION, OBJECT)
                AddBelief(f"<{RELATION} --> [FLIPPED]>", (1.0, 0.9))
            if S == SUBJECT and O == OBJECT:
                print("//Grammatical relation order detected", S, R, O, SUBJECT, RELATION, OBJECT)
                AddBelief(f"<{RELATION} --> [FLIPPED]>", (0.0, 0.9))
    (SUBJECT, RELATION, OBJECT, words) = (None, None, None, None)

def processInput(inp, Print=True):
    if Print: print("//Input: " + inp)
    if inp.isdigit():
        if "words" in globals() and "SUBJECT" in globals() and words is not None and SUBJECT is not None:
            if words[0] is not None:
                correlate()
        return
    if inp.startswith("<") or inp.startswith("("):
        newConcept(inp[:-1])
    else:
        newSentence(inp)
        processInput("1", Print=False)

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
            if inp=="":
                inp = "1"
        except:
            exit(0)
        if inp.startswith("//"):
            print(inp)
            continue
        if not Training and (inp.isdigit() or inp.startswith("<") or inp.endswith(". :|:") or inp.endswith("! :|:")):
            ret = NAR.AddInput(inp, Print=False)
            if ret["input"]:
                print("Input:", NAR.PrintedTask(ret["input"][0]))
                if "answers" in ret and ret["answers"]:
                    print("Answer:", NAR.PrintedTask(ret["answers"][0]))
                if "executions" in ret and ret["executions"]:
                    print(ret["executions"])
            executions = ret["executions"]
            if executions:
                for execution in executions:
                    if execution["operator"] == "^say":
                        arguments = [x.replace("*","").replace("(","").replace(")","") for x in execution["arguments"].split(" ")]
                        for concept in arguments:
                            print("^say result: " + Query(f"<(?1 * {concept}) --> R>")[2][0][1])
            if ret["selections"]:
                print("Selected:", NAR.PrintedTask(ret["selections"][0]))
                processInput(ret["selections"][0]["term"] + ret["selections"][0]["punctuation"])
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
        elif inp.startswith("*R "):
            arg = inp.split("*R ")[1]
            print("//RELATION?", Query(f"<{arg} --> RELATION>"))
            print("//YES:", Query(f"<({arg} * ?1) --> R>", isRelation=True))
            print("//NO:", Query(f"<({arg} * ?1) --> R>", isRelation=False))
            print("//FLIPPED:", Query(f"<{arg} --> [FLIPPED]>"))
            print("//ASSIGNED:", Query(f"<{arg} --> [ASSIGNED]>"))
            continue
        elif inp.startswith("*"):
            print(inp)
            continue
        processInput(inp)



