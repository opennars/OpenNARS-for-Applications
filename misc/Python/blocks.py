import NAR

NAR.AddInput("*motorbabbling=false")
NAR.AddInput("*volume=0")
NAR.AddInput("<<({SELF} * $1) --> collect> ==> <(<$1 --> [left]> &/ ^left) =/> <$1 --> [centered]>>>.")
NAR.AddInput("<<({SELF} * $1) --> collect> ==> <(<$1 --> [centered]> &/ ^pick) =/> G>>.")
NAR.AddInput("<<({SELF} * $1) --> collect> ==> <(<$1 --> [right]> &/ ^right) =/> <$1 --> [centered]>>>.")
NAR.AddInput("<you <-> {SELF}>.")

selfpos = "centered"
objects = [("box","left"), ("bottle","right")]

test = ""
while not test.startswith("<"):
    test = input()
NAR.AddInput(test)
NAR.AddInput("20")

def sim_step():
    global selfpos
    for (obj, location) in objects:
        relLoc = "centered"
        if (selfpos == "left" and (location == "centered" or location == "right")) or (selfpos == "centered" and location == "right"):
            relLoc = "right"
        if (selfpos == "right" and (location == "centered" or location == "left")) or (selfpos == "centered" and location == "left"):
            relLoc = "left"
        NAR.AddInput("<%s --> [%s]>. :|:" % (obj, relLoc))
    executions = NAR.AddInput("G! :|:")["executions"]
    executions += NAR.AddInput("10")["executions"]
    if len(executions) > 0:
        op = executions[0]["operator"]
        if op == "^left":
            if selfpos == "centered":
                selfpos = "left"
            elif selfpos == "right":
                selfpos = "centered"
        elif op == "^right":
            if selfpos == "centered":
                selfpos = "right"
            elif selfpos == "left":
                selfpos = "centered"
sim_step()
sim_step()

