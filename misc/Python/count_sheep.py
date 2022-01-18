import NAR

NAR.AddInput("*setopname 1 ^count")
NAR.AddInput("*motorbabbling=false")
NAR.AddInput("*volume=0")
NAR.AddInput("10")
NAR.AddInput("<(<(sheep * fence) --> jump> &/ ^count) =/> G>.")

cnt = 0
def count(executions):
    global cnt
    if len(executions) > 0:
        cnt += 1

sheeps = 20
for t in range(sheeps):
    NAR.AddInput("<sheep --> [white]>. :|:")
    NAR.AddInput("<{ex%d} --> [white]>. :|:" % t)
    NAR.AddInput("<({ex%d} * fence) --> jump>. :|:" % t)
    NAR.AddInput("10")
    executions = NAR.AddInput("G! :|:")["executions"]
    for i in range(5):
        executions += NAR.AddInput("1")["executions"]
    NAR.AddInput("500")
    count(executions)

print("sheep counted:", cnt, "of", str(sheeps))
