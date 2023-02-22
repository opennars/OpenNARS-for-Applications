import NAR
import random
import time
import sys

NAR.AddInput("*volume=0")
fovea = "U"
t=0
actions = 0

def op_left():
    global fovea
    NAR.AddInput("G. :|: {0.0 0.05}")
    fovea = "L"
        
def op_right():
    global fovea
    NAR.AddInput("G. :|: {0.0 0.05}")
    fovea = "R"

def op_up():
    global fovea
    NAR.AddInput("G. :|: {0.0 0.05}")
    fovea = "U"

def create_new_match_to_sample_scenario():
    global items, fovea
    print("//NEW SCENARIO!!!!")
    items = random.choice([{"U" : "A1", "L" : "B1", "R" : "B2"},
                           {"U" : "A1", "L" : "B2", "R" : "B1"},
                           {"U" : "X1", "L" : "B1", "R" : "B2"},
                           {"U" : "X1", "L" : "B2", "R" : "B1"}])
    fovea = "U" #random.choice(["U", "L", "R"])
    #easy to train extra not to pick a stimulus just for stimulus sake
    NAR.AddInput("dt=1.0 <(LB1 &/ ^pick) =/> G>. {0.0 0.99}")
    NAR.AddInput("dt=1.0 <(RB1 &/ ^pick) =/> G>. {0.0 0.99}")
    NAR.AddInput("dt=1.0 <(LB2 &/ ^pick) =/> G>. {0.0 0.99}")
    NAR.AddInput("dt=1.0 <(RB2 &/ ^pick) =/> G>. {0.0 0.99}")

def rewarded_after_pick():
    match_to_sample_cases = [("X1", "B2"), ("A1", "B1")]
    for (sample, other) in match_to_sample_cases:
        if fovea != "U" and  items["U"] == sample and items[fovea] == other:
            return True
    return False

failures = 0
successes = 0

def stats():
    ratio = 0 if successes+failures == 0 else successes / (failures + successes)
    return f"failures={failures}, successes={successes} ratio={ratio}, t={t}"

def op_pick(failure=False):
    global failures, successes
    if failure == False and rewarded_after_pick():
        NAR.AddInput("G. :|:")
        successes += 1
        print("//SUCCESS +++++++", stats())
        sys.stdout.flush()
        if t % 1000 == 0:
            time.sleep(1.0)
    else:
        NAR.AddInput("G. :|: {0.0 0.999}")
        print("//FAILURE +++++++", stats())
        failures += 1
        sys.stdout.flush()
        if t % 1000 == 0:
            time.sleep(1.0)
    if successes + failures == 1000:
        exit(0)
    create_new_match_to_sample_scenario()
    NAR.AddInput("20")
    
ops = { "^left": op_left, "^right": op_right, "^up": op_up, "^pick" : op_pick}
NAR.AddInput("*babblingops="+str(len(ops)))
for i, (name, f) in enumerate(ops.items()):
    NAR.AddInput("*setopname %d %s" % (i+1, name))
create_new_match_to_sample_scenario()

def step():
    val = items[fovea]
    NAR.AddInput(fovea + val + ". :|:")
    executions = NAR.AddInput("G! :|:")["executions"]
    if executions:
        for i, execution in enumerate(executions):
            opname = execution["operator"]
            ops[opname]()
            break

while True:
    t += 1
    step()

