import os
import sys
import ast
import signal
import subprocess

def spawnNAR():
    return subprocess.Popen(["./../../NAR", "shell"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, universal_newlines=True)
NARproc = spawnNAR()
def getNAR():
    return NARproc
def setNAR(proc):
    global NARproc
    NARproc = proc
def terminateNAR(usedNAR=NARproc):
    os.killpg(os.getpgid(usedNAR.pid), signal.SIGTERM)

def parseTruth(T):
    return {"frequency": T.split("frequency=")[1].split(" confidence")[0].replace(",",""), "confidence": T.split(" confidence=")[1].split(" dt=")[0].split(" occurrenceTime=")[0]}

def parseTask(s):
    M = {"occurrenceTime" : "eternal"}
    if " :|:" in s:
        M["occurrenceTime"] = "now"
        s = s.replace(" :|:","")
        if "occurrenceTime" in s:
            M["occurrenceTime"] = s.split("occurrenceTime=")[1].split(" ")[0]
    if "Stamp" in s:
        M["Stamp"] = ast.literal_eval(s.split("Stamp=")[1].split("]")[0]+"]")
    sentence = s.split(" occurrenceTime=")[0] if " occurrenceTime=" in s else s.split(" Stamp=")[0].split(" Priority=")[0].split(" creationTime=")[0]
    M["punctuation"] = sentence[-4] if ":|:" in sentence else sentence[-1]
    M["term"] = sentence.split(" creationTime")[0].split(" occurrenceTime")[0].split(" Truth")[0].split(" Stamp=")[0][:-1]
    if "Truth" in s:
        M["truth"] = parseTruth(s.split("Truth: ")[1])
    if "Priority" in s:
        M["Priority"] = s.split("Priority=")[1].split(" ")[0]
    return M

def parseReason(sraw):
    if "implication: " not in sraw:
        return None
    Implication = parseTask(sraw.split("implication: ")[-1].split("precondition: ")[0]) #last reason only (others couldn't be associated currently)
    Precondition = parseTask(sraw.split("precondition: ")[-1].split("\n")[0])
    Implication["occurrenceTime"] = "eternal"
    Precondition["punctuation"] = Implication["punctuation"] = "."
    Reason = {}
    Reason["desire"] = sraw.split("decision expectation=")[-1].split(" ")[0]
    Reason["hypothesis"] = Implication
    Reason["precondition"] = Precondition
    return Reason
    
def parseExecution(e):
    if "args " not in e:
        return {"operator" : e.split(" ")[0], "arguments" : []}
    return {"operator" : e.split(" ")[0], "arguments" : e.split("args ")[1].split("{SELF} * ")[1][:-1]}

def GetRawOutput(usedNAR):
    usedNAR.stdin.write("0\n")
    usedNAR.stdin.flush()
    ret = ""
    before = []
    requestOutputArgs = False
    while "done with 0 additional inference steps." != ret.strip():
        if ret != "":
            before.append(ret.strip())
        if ret.strip() == "//Operation result product expected:":
            requestOutputArgs = True
            break
        ret = usedNAR.stdout.readline()
    return before[:-1], requestOutputArgs

def GetOutput(usedNAR):
    lines, requestOutputArgs = GetRawOutput(usedNAR)
    executions = [parseExecution(l) for l in lines if l.startswith('^')]
    inputs = [parseTask(l.split("Input: ")[1]) for l in lines if l.startswith('Input:')]
    derivations = [parseTask(l.split("Derived: " if l.startswith('Derived:') else "Revised: ")[1]) for l in lines if l.startswith('Derived:') or l.startswith('Revised:')]
    answers = [parseTask(l.split("Answer: ")[1]) for l in lines if l.startswith('Answer:')]
    selections = [parseTask(l.split("Selected: ")[1]) for l in lines if l.startswith('Selected:')]
    reason = parseReason("\n".join(lines))
    return {"input": inputs, "derivations": derivations, "answers": answers, "executions": executions, "reason": reason, "selections": selections, "raw": "\n".join(lines), "requestOutputArgs" : requestOutputArgs}

def GetStats(usedNAR):
    Stats = {}
    lines, _ = GetRawOutput(usedNAR)
    for l in lines:
        if ":" in l:
            leftside = l.split(":")[0].replace(" ", "_").strip()
            rightside = float(l.split(":")[1].strip())
            Stats[leftside] = rightside
    return Stats

def AddInput(narsese, Print=True, usedNAR=NARproc):
    usedNAR.stdin.write(narsese + '\n')
    usedNAR.stdin.flush()
    ReturnStats = narsese == "*stats"
    if ReturnStats:
        if Print:
            print("\n".join(GetRawOutput(usedNAR)[0]))
        return GetStats(usedNAR)
    ret = GetOutput(usedNAR)
    if Print:
        print(ret["raw"])
        sys.stdout.flush()
    return ret

def Exit(usedNAR=NARproc):
    usedNAR.sendline("quit")

def Reset(usedNAR=NARproc):
    AddInput("*reset", usedNAR=usedNAR)

AddInput("*volume=100")

def PrintedTask(task):
    st = task["term"] + task["punctuation"]
    st += (" :|: occurrenceTime="+task["occurrenceTime"] if task["occurrenceTime"].isdigit() else "")
    if "Priority" in task: st += " Priority=" + str(task["Priority"])
    if "truth" in task: st += " Truth: frequency="+task["truth"]["frequency"] + " confidence="+task["truth"]["confidence"]
    return st

def Shell():
    while True:
        try:
            inp = input().rstrip("\n")
        except:
            exit(0)
        AddInput(inp)

if __name__ == "__main__":
    Shell()
