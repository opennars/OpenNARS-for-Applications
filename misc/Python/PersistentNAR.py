import NAR
import json
from os.path import exists
import sys

fname = "mem.json"
derivationTrigger = "selections" if "selections" in sys.argv else "derivations"
memory = {}
if exists(fname):
    with open(fname) as json_file:
        memory = json.load(json_file)
retrieved = set([])

def query(term):
    if not term in retrieved and term in memory:
        retrieved.add(term)
        (f, c, _) = memory[term]
        ProcessNAROutput(NAR.AddInput(f"{term}. {{{f} {c}}}"))

def ProcessNAROutput(ret, backups = ["input", "answers", derivationTrigger])
    for backup in backups:
        for derivation in ret[backup]:
            if derivation["punctuation"] == "." and derivation["occurrenceTime"] == "eternal" and derivation["term"] != "None":
                term = derivation["term"]
                if term.startswith("dt="): #we don't need to store time deltas
                    term = term.split(" ")[1]
                query(term)
                f2 = derivation["truth"]["frequency"]
                c2 = derivation["truth"]["confidence"]
                usefulnessAddition = 1000000 if "Priority" not in derivation or derivation["Priority"] == 1.0 else 1
                if term in memory:
                    (f, c, usefulness) = memory[term]
                    if c2 >= c:
                        memory[term] = (f2, c2, usefulness + usefulnessAddition)
                else:
                    memory[term] = (f2, c2, usefulnessAddition)

def SimplisticTermNormalizer(inp): #at least handle space variations in involved parenthesis in input
    for x in ["(", ")", "<", ">"]: #Term reductions more tricky here
        inp = inp.replace(" " + x, x).replace(x + " ",x)
    inp=inp.replace("-->","--> ").replace("<->"," <-> ").replace(
                    "==>","==> ").replace("<=>"," <=> ").replace(
                    "=/>","=/> ").strip().replace("[ ","[").replace(" ]","]").replace("{ ","{").replace(" }","}")
    return inp

if __name__ == "__main__":
    while True:
        inp = SimplisticTermNormalizer(input().rstrip("\n"))
        if inp.startswith("//"):
            continue
        if inp.endswith("?"): #query first
            query(inp.split("?")[0].strip())
        ProcessNAROutput(NAR.AddInput(inp))
        with open(fname, 'w') as f:
            json.dump(memory, f)
        
