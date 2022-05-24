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

ops1 = { "^inc":     lambda x: x+1 ,
         "^dec":     lambda x: x-1 }

ops2 = { "^add":     lambda x, y: x+y  ,
         "^sub":     lambda x, y: x-y  ,
         "^mul":     lambda x, y: x*y  ,
         "^div":     lambda x, y: x/y  ,
         "^greater": lambda x, y: x>y  ,
         "^equal":   lambda x, y: x==y ,
         "^smaller": lambda x, y: x<y  }

for i, (name, f) in enumerate(ops1.items()):
    NAR.AddInput("*setopname %d %s" % (i+1, name))
for i, (name, f) in enumerate(ops2.items()):
    NAR.AddInput("*setopname %d %s" % (i+1, name))

def NAR_numerics_execute(executions, requestOutputArgs):
    if len(executions) > 0:
        result = 0.0
        for i, execution in enumerate(executions):
            opname = execution["operator"]
            if opname in ops2.keys() and len(execution["arguments"]) > 0:
                try:
                    if " *" in execution["arguments"]:
                        x, y, _ = execution["arguments"].replace("(","").replace(")","").split(" * ")
                        result = ops2[opname](float(x), float(y))
                        if requestOutputArgs:
                            ret = NAR.AddInput("((%s * %s) * %s)" % (x, y, result))
                            NAR_numerics_execute(ret["executions"], ret["requestOutputArgs"])
                except:
                    None #wrong args, no result
            if opname in ops1.keys() and len(execution["arguments"]) > 0:
                try:
                    if " *" in execution["arguments"]:
                        x, _ = execution["arguments"].replace("(","").replace(")","").split(" * ")
                        result = ops1[opname](float(x))
                        if requestOutputArgs:
                            ret = NAR.AddInput("(%s * %s)" % (x, result))
                            NAR_numerics_execute(ret["executions"], ret["requestOutputArgs"])
                except:
                    None #wrong args, no result

if __name__ == "__main__":
    while True:
        try:
            inp = input().rstrip("\n")
        except:
            exit(0)
        ret = NAR.AddInput(inp)
        NAR_numerics_execute(ret["executions"], ret["requestOutputArgs"])
