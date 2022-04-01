"""
 * The MIT License
 *
 * Copyright 2022 The OpenNARS authors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * """

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
