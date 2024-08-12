import re
import NAR
import time
import random
import sys

NAR.AddInput("*motorbabbling=false")

def NAR_Cycle(n):
    NAR.AddInput(str(n))

def toMeTTa(term):
    if term.startswith("dt="):
        term = " ".join(term.split(" ")[1:])
    term = re.sub(r"\^([a-zA-Z0-9]*)", r"(^ \1)", term)
    term = re.sub(r"\[([a-zA-Z0-9]*)\]", r"(IntSet \1)", term)
    return term.replace("<", "(").replace(">", ")").replace("--)", "-->").replace("=)", "=>").replace("(=", "<=").replace("/)", "/>").replace("(-)", "-")

def NAR_AddInput(metta):
    print("//" + metta)
    truth = ""
    if metta.startswith("!(AddBeliefEvent "):
        truth = " " + (" ".join(metta.split("!(AddBeliefEvent (")[1].split(" ")[-2:]))[:-2].replace("(", "{").replace(")", "}")
        metta = " ".join(metta.split("!(AddBeliefEvent (")[1].split(" ")[:-2]) + ". :|:"
    if metta.startswith("!(AddGoalEvent "):
        truth = " " + (" ".join(metta.split("!(AddGoalEvent (")[1].split(" ")[-2:]))[:-2].replace("(", "{").replace(")", "}")
        metta = " ".join(metta.split("!(AddGoalEvent (")[1].split(" ")[:-2]) + "! :|:"
    if metta.startswith("!(EventQuestion "):
        metta = metta.split("!(EventQuestion ")[1][:-1] + "? :|:"
    if metta.startswith("!(EternalQuestion "):
        metta = metta.split("!(EternalQuestion ")[1][:-1] + "?"
    metta = metta.replace("IntSet", "'").replace("ExtSet", '"').replace(r"(^ \s*)", r"^")
    metta = re.sub(r"\(\^\s([a-zA-Z0-9]*)\)", r"^\1", metta) #operator format of MeTTa-NARS
    ret = NAR.AddInput(metta + truth)
    results = ret["input"] + ret["derivations"]
    All = []
    Answers = set([])
    for x in results:
        All += [x]
        if x["punctuation"] != "!":
            ret2 = NAR.AddInput(x["term"] + "?")
            All += [ret2["answers"][0]]
            Answers.add(str(ret2["answers"][0]))
    for x in All:
        prefix = "MeTTa-IN" if x in ret["input"] else "MeTTa-OUT"
        punctuation = "." if x["punctuation"] == "." else "!"
        if str(x) in Answers:
            prefix = "MeTTa-OUT"
            punctuation = "@"
        if x["term"] == "None":
            continue
        truthMeTTa = ""
        if "truth" in x:
            truthMeTTa = "(" + x["truth"]["frequency"] + " " + x["truth"]["confidence"] + ")"
        x["metta"] = "(" + toMeTTa(x["term"]) + " " + truthMeTTa + ")"
        print("!(" + prefix + " ("  + "" + punctuation + " " + x["metta"] + " " + x["occurrenceTime"]+ "))")

if "shell" in sys.argv:
    while True:
        inp = input()
        if inp.isnumeric():
            NAR_Cycle(inp)
        else:
            NAR_AddInput(inp)
