import sys
import subprocess

NAR = subprocess.Popen(["./../../NAR", "shell"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, universal_newlines=True)

def parseTruth(T):
    return {"frequency": T.split("frequency=")[1].split(" confidence")[0], "confidence": T.split(" confidence=")[1].split(" dt=")[0].split(" occurrenceTime=")[0]}

def parseTask(s):
    M = {"occurrenceTime" : "eternal"}
    if " :|:" in s:
        M["occurrenceTime"] = "now"
        s = s.replace(" :|:","")
        if "occurrenceTime" in s:
            M["occurrenceTime"] = s.split("occurrenceTime=")[1].split(" ")[0]
    sentence = s.split(" occurrenceTime=")[0] if " occurrenceTime=" in s else s.split(" Priority=")[0]
    M["punctuation"] = sentence[-4] if ":|:" in sentence else sentence[-1]
    M["term"] = sentence.split(" creationTime")[0].split(" occurrenceTime")[0].split(" Truth")[0][:-1]
    if "Truth" in s:
        M["truth"] = parseTruth(s.split("Truth: ")[1])
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

def GetRawOutput():
    NAR.stdin.write("0\n")
    NAR.stdin.flush()
    ret = ""
    before = []
    requestOutputArgs = False
    while "done with 0 additional inference steps." != ret.strip():
        if ret != "":
            before.append(ret.strip())
        if ret.strip() == "//Operation result product expected:":
            requestOutputArgs = True
            break
        ret = NAR.stdout.readline()
    return before[:-1], requestOutputArgs

def GetOutput():
    lines, requestOutputArgs = GetRawOutput()
    executions = [parseExecution(l) for l in lines if l.startswith('^')]
    inputs = [parseTask(l.split("Input: ")[1]) for l in lines if l.startswith('Input:')]
    derivations = [parseTask(l.split("Derived: " if l.startswith('Derived:') else "Revised:")[1]) for l in lines if l.startswith('Derived:') or l.startswith('Revised:')]
    answers = [parseTask(l.split("Answer: ")[1]) for l in lines if l.startswith('Answer:')]
    reason = parseReason("\n".join(lines))
    return {"input": inputs, "derivations": derivations, "answers": answers, "executions": executions, "reason": reason, "raw": "\n".join(lines), "requestOutputArgs" : requestOutputArgs}

def GetStats():
	Stats = {}
	lines, _ = GetRawOutput()
	for l in lines:
		if ":" in l:
		    leftside = l.split(":")[0].replace(" ", "_").strip()
		    rightside = float(l.split(":")[1].strip())
		    Stats[leftside] = rightside
	return Stats

def AddInput(narsese, Print=True):
    NAR.stdin.write(narsese + '\n')
    NAR.stdin.flush()
    ReturnStats = narsese == "*stats"
    if ReturnStats:
        return GetStats()
    ret = GetOutput()
    if Print:
        print(ret["raw"])
        sys.stdout.flush()
    return ret

def Exit():
    NAR.sendline("quit")

def Reset():
    AddInput("*reset")

AddInput("*volume=100")
