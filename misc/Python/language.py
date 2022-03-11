import NAR

NAR.AddInput("*setopname 1 ^say")
NAR.AddInput("*motorbabbling=false")

delay = 50
def ThisIs(thing, name):
    #NAR.AddInput("<ThisIs --> [heard]>. :|:")
    NAR.AddInput("<" + name + " --> [heard]>. :|:")
    NAR.AddInput("<" + thing + " --> [seen]>. :|:")
    NAR.AddInput(str(delay))
    
def WhatIs(thing, name):
    NAR.AddInput("<WhatIs --> [heard]>. :|:")
    NAR.AddInput("<" + thing + " --> [seen]>. :|:")
    NAR.AddInput("<({SELF} * " + name + ") --> ^say>. :|:")
    NAR.AddInput("G. :|:")
    NAR.AddInput(str(delay))
    
    
def WhatIs_NARS(thing, rightAnswer, Asked=True):
    if Asked:
        NAR.AddInput("<WhatIs --> [heard]>. :|:")
    NAR.AddInput("<" + thing + " --> [seen]>. :|:")
    executions = NAR.AddInput("G! :|:")["executions"]
    executions += NAR.AddInput("5")["executions"]
    if len(executions) > 0:
        if not Asked:
            NAR.AddInput("G. :|: {0.0 0.9}")
        elif executions[0]["arguments"][0] == rightAnswer:
            NAR.AddInput("G. :|: {1.0 0.9}")
        else:
            NAR.AddInput("G. :|: {0.0 0.9}")
    NAR.AddInput(str(delay))
    
def NameHeard(name):
    NAR.AddInput("<" + name + " --> [heard]>. :|:")
    
def ThingSeen(name):
    NAR.AddInput("<" + name + " --> [seen]>. :|:")

NAR.AddInput("*volume=0")
for i in range(1):
    ThisIs("Vc", "cat")
    WhatIs("Vc", "cat")
    ThisIs("Vb", "bus")
    ThisIs("Vh", "house")
    for j in range(20):
        WhatIs_NARS("Vb", "bus", Asked=False)
        WhatIs_NARS("Vh", "bus", Asked=False)
        WhatIs_NARS("Vb", "bus")
        WhatIs_NARS("Vh", "house")
