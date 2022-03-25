import NAR

ops = { "^add":     lambda x, y: x+y  ,
        "^sub":     lambda x, y: x-y  ,
        "^mul":     lambda x, y: x*y  ,
        "^div":     lambda x, y: x/y  ,
        "^greater": lambda x, y: x>y  ,
        "^equal":   lambda x, y: x==y ,
        "^smaller": lambda x, y: x<y  } 

for i, (name, f) in enumerate(ops.items()):
    NAR.AddInput("*setopname %d %s" % (i+1, name))

def NAR_numerics_execute(executions):
    if len(executions) > 0:
        execution = executions[0]
        if execution["operator"] in ops.keys() and len(execution["arguments"]) > 0:
            x, y = execution["arguments"].replace("(","").replace(")","").split(" * ")
            result = ops[execution["operator"]](float(x), float(y))
            NAR.AddInput("<%s --> result>. :|:" % str(result)) 

if __name__ == "__main__":
    while True:
        inp = input().rstrip("\n")
        executions = NAR.AddInput(inp)["executions"]
        NAR_numerics_execute(executions)
