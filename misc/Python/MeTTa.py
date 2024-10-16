import re
import NAR
import time
import random
import sys

NAR.AddInput("*motorbabbling=false")
NAR_useNarsese = False
NAR_printInMeTTa = False

def NAR_SetUseNarsese(flag):
    global NAR_useNarsese
    NAR_useNarsese = flag

def NAR_Cycle(n):
    return NAR.AddInput(str(n))

def NAR_PrintInMetta():
    global NAR_printInMeTTa
    NAR_printInMeTTa = True

def NAR_NarseseToMeTTa(term):
    if term.startswith("dt="):
        term = " ".join(term.split(" ")[1:])
    term = re.sub(r"\^([a-zA-Z0-9]*)", r"(^ \1)", term)
    term = re.sub(r"\[([a-zA-Z0-9]*)\]", r"(IntSet \1)", term)
    return term.replace("<", "(").replace(">", ")").replace("--)", "-->").replace("=)", "=>").replace("(=", "<=").replace("/)", "/>").replace("(-)", "-")

def NAR_AddInput(metta):
    global NAR_useNarsese
    print("//" + metta)
    truth = ""
    if not NAR_useNarsese:
        metta = metta.replace(" x ", " * ")
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
    for x in results:
        if x["term"] == "None":
            continue
        truthMeTTa = ""
        if "truth" in x:
            truthMeTTa = "(" + x["truth"]["frequency"] + " " + x["truth"]["confidence"] + ")"
        x["metta"] = "(" + x["punctuation"] + ": (" + NAR_NarseseToMeTTa(x["term"]) + " " + truthMeTTa + "))"
    if NAR_printInMeTTa:
        All = []
        Answers = set([])
        for x in results:
            All += [x]
            if x["punctuation"] != "!" and x["punctuation"] != "?":
                ret2 = NAR.AddInput(x["term"] + "?")
                All += [ret2["answers"][0]]
                Answers.add(str(ret2["answers"][0]))
        for x in All:
            prefix = "MeTTa-IN" if x in ret["input"] else "MeTTa-OUT"
            punctuation = "." if x["punctuation"] == "." else "!" if x["punctuation"] == "!" else "?"
            if str(x) in Answers:
                prefix = "MeTTa-OUT"
                punctuation = "@"
            if x["term"] == "None":
                continue
            truthMeTTa = ""
            if "truth" in x:
                truthMeTTa = "(" + x["truth"]["frequency"] + " " + x["truth"]["confidence"] + ")"
            x["metta"] = "(" + punctuation + ": (" + NAR_NarseseToMeTTa(x["term"]) + " " + truthMeTTa + "))"
            print("!(" + prefix + " " + "(" + x["metta"] + " " + x["occurrenceTime"]+ ")")
    return ret

if "shell" in sys.argv:
    while True:
        inp = input()
        if inp.isnumeric():
            NAR_Cycle(inp)
        else:
            NAR_AddInput(inp)
