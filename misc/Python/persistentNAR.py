import NAR
import json
from os.path import exists
import sys

fname = "mem.json"
Print = "Print" in sys.argv
memory = {}
if exists(fname):
    with open(fname) as json_file:
        print("//Loaded memory content from", fname)
        memory = json.load(json_file)
        maxBaseId = 0
        for key in memory:
            maxBaseId = max([maxBaseId] + memory[key][3])
        NAR.AddInput("*stampid=" + str(maxBaseId + 1))
retrieved = set([])

def Allow_requery_if_not_in_ONA(term):
    #check if previously queried item is not in ONA memory anymore else we need
    #to set it up for re-query by removing it from retrieved
    if term in retrieved:
        ret = NAR.AddInput(term + "?", Print=False)
        if "answers" in ret and ret["answers"]:
            answer = ret["answers"][0]
            if "truth" not in answer and answer["term"] == "None":
                retrieved.remove(term)

def query(term):
    Allow_requery_if_not_in_ONA(term)
    if term not in retrieved and term in memory:
        retrieved.add(term)
        (f, c, _, stamp) = memory[term]
        NAR.AddInput("*stampimport=" + str(stamp), Print=False)
        ProcessNAROutput(NAR.AddInput(f"{term}. {{{f} {c}}}", Print=Print))
    if "?1" in term: #simple query matching
        parts = term.split("?1")
        truth_expectation = lambda f,c: (c * (f - 0.5) + 0.5)
        bestTerm, bestTruth, bestStamp = (None, (0.0, 0.5), [])
        for term2 in memory:
            (f2, c2, _, stamp) = memory[term2]
            if term2.startswith(parts[0]) and term2.endswith(parts[1]):
                if truth_expectation(f2, c2) > truth_expectation(bestTruth[0], bestTruth[1]):
                    bestTerm = term2
                    bestTruth = (f2, c2)
                    bestStamp = stamp
        if bestTerm is not None:
            Allow_requery_if_not_in_ONA(bestTerm)
        if bestTerm is not None and bestTerm not in retrieved:
            retrieved.add(bestTerm)
            NAR.AddInput("*stampimport=" + str(bestStamp), Print=False)
            ProcessNAROutput(NAR.AddInput(f"{bestTerm}. {{{bestTruth[0]} {bestTruth[1]}}}", Print=Print))
    retrieved.add(term)

def ProcessNAROutput(ret, backups = ["input", "answers", "derivations"]):
    for backup in backups:
        for derivation in ret[backup]:
            if derivation["punctuation"] == "." and derivation["occurrenceTime"] == "eternal" and derivation["term"] != "None":
                term = derivation["term"]
                if term.startswith("dt="): #we don't need to store time deltas
                    term = " ".join(term.split(" ")[1:])
                query(term)
                f2 = float(derivation["truth"]["frequency"])
                c2 = float(derivation["truth"]["confidence"])
                stamp = derivation["Stamp"]
                usefulnessAddition = 1000000 if "Priority" not in derivation or derivation["Priority"] == 1.0 else 1
                if term in memory:
                    (f, c, usefulness, _) = memory[term]
                    if c2 >= c:
                        memory[term] = (f2, c2, usefulness + usefulnessAddition, stamp)
                else:
                    memory[term] = (f2, c2, usefulnessAddition, stamp)

def SimplisticTermNormalizer(inp): #at least handle space variations in involved parenthesis in input
    inp=inp.replace("-->","--> ").replace("<->"," <-> ").replace("  "," ").replace(
                    "==>","==> ").replace("<=>"," <=> ").replace("  "," ").replace(
                    "=/>","=/> ").strip().replace("[ ","[").replace(" ]","]").replace("{ ","{").replace(" }","}").replace(
                                                  "< ","<").replace(" >",">").replace(" )",")").replace("( ","(").replace("  "," ").strip()
    return inp

if __name__ == "__main__":
    while True:
        inp = SimplisticTermNormalizer(input().rstrip("\n"))
        if inp.startswith("//") and not Print:
            print(inp)
            continue
        if inp.startswith("*memory"):
            print(memory)
            continue
        if not Print:
            print("Input: " + inp)
        if inp.endswith("?"): #query first
            query(inp[:-1])
        ret = NAR.AddInput(inp, Print=Print)
        if not Print and "answers" in ret and ret["answers"]:
            answer = ret["answers"][0]
            if "truth" not in answer:
                print("Answer: None.")
            else:
                occurrenceTimeInfo = "" if answer["occurrenceTime"] == "eternal" else " t="+answer["occurrenceTime"]
                print("Answer: " + answer["term"] + answer["punctuation"] + " {" + str(answer["truth"]["frequency"]) + " " + str(answer["truth"]["confidence"]) + "}" + occurrenceTimeInfo)
        ProcessNAROutput(ret)
        with open(fname, 'w') as f:
            json.dump(memory, f)
        
