import pexpect
NAR = pexpect.spawn('./../../NAR shell')

def parseTruth(T):
    return {"frequency": T.split("frequency=")[1].split(" confidence")[0], "confidence": T.split(" confidence=")[1]}

def parseTask(s):
    M = {"occurrenceTime" : "eternal"}
    if " :|:" in s:
        M["occurrenceTime"] = "now"
        s = s.replace(" :|:","")
        if "occurrenceTime" in s:
            M["occurrenceTime"] = s.split("occurrenceTime=")[1].split(" ")[0]
    sentence = s.split(" Priority=")[0]
    M["punctuation"] = sentence[-1]
    M["term"] = sentence.split(" creationTime")[0].split(" occurrenceTime")[0][:-1]
    if "Truth" in s:
        M["truth"] = parseTruth(s.split("Truth: ")[1])
    return M
    
def parseExecution(e):
    return {"operator" : e.split(" ")[0], "arguments" : e.split("args ")[1][1:-1].split(" * ")[1]}
    
def GetOutput():
    NAR.sendline("0")
    NAR.expect("done with 0 additional inference steps.")
    lines = [a.strip().decode("utf-8") for a in NAR.before.split(b'\n')][2:-3]
    executions = [parseExecution(l) for l in lines if l.startswith('^')]
    inputs = [parseTask(l.split("Input: ")[1]) for l in lines if l.startswith('Input:')]
    derivations = [parseTask(l.split("Derived: ")[1]) for l in lines if l.startswith('Derived:')]
    answers = [parseTask(l.split("Answer: ")[1]) for l in lines if l.startswith('Answer:')]
    return {"input": inputs, "derivations": derivations, "answers": answers, "executions": executions}

def AddInput(narsese):
    NAR.sendline(narsese)
    return GetOutput()

def Exit():
    NAR.sendline("quit")

def Reset():
    AddInput("*reset")

AddInput("*volume=100")
