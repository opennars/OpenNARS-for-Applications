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
        opname = execution["operator"]
        if opname in ops.keys() and len(execution["arguments"]) > 0:
            try:
                x, y = execution["arguments"].replace("(","").replace(")","").split(" * ")
                result = ops[opname](float(x), float(y))
                NAR.AddInput("<%s --> [executed]>. :|:" % str(opname.replace("^","")))
                NAR.AddInput("<%s --> result>. :|:" % str(result))
            except:
                None #wrong args, no result

if __name__ == "__main__":
    while True:
        try:
            inp = input().rstrip("\n")
        except:
            exit(0)
        executions = NAR.AddInput(inp)["executions"]
        NAR_numerics_execute(executions)
