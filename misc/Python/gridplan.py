import NAR
import time
import sys
import copy
NAR.AddInput("*motorbabbling=false")
NAR.AddInput("*volume=0")

def state(XY):
    return str(XY[0]) + "_" + str(XY[1])

def contingency(pre, op, cons):
    return "<(" + state(pre) + " &/ " + op + ") =/> " + state(cons) + ">." 

SX = 6
SY = 6
unreachables = {(3,3), (3,4), (3,5), (5,3)} # (6,7), (6,8), (6,9)
position = (0,0)
goal = (5,5)

for x in range(SX):
    for y in range(SY):
        cur = (x, y)
        if cur in unreachables:
            continue
        left = (x-1, y)
        right = (x+1, y)
        up = (x, y+1)
        down = (x, y-1)
        left_statement = contingency(left, "^right", cur)
        right_statement = contingency(right, "^left", cur)
        up_statement = contingency(up, "^down", cur)
        down_statement = contingency(down, "^up", cur)
        if x > 0:
            NAR.AddInput(left_statement)
        if x < SX-1:
            NAR.AddInput(right_statement)
        if y > 0:
            NAR.AddInput(down_statement)
        if y < SY-1:
            NAR.AddInput(up_statement)

def execute(executions):
    global position
    lastposition = position
    if executions:
        execution = executions[0]
        if execution["operator"] == "^left":
            position = (max(0, position[0] - 1), position[1])
            print("^left")
        if execution["operator"] == "^right":
            position = (min(SX-1, position[0] + 1), position[1])
            print("^right")
        if execution["operator"] == "^up":
            position = (position[0], min(SY-1, position[1] + 1))
            print("^up")
        if execution["operator"] == "^down":
            position = (position[0], max(0, position[1] - 1))
            print("^down")
    if position in unreachables:
        position = lastposition

field = [['  ' for x in range(SX)] for y in range(SY)]

while True:
    if position == goal:
        break
    NAR.AddInput(state(position) + ". :|:")
    executions = NAR.AddInput(state(goal) + "! :|:")["executions"]
    for i in range(5):
        executions += NAR.AddInput("1", Print=False)["executions"]
    execute(executions)
    print("\033[1;1H\033[2J") #clear screen
    #prepare grid:
    curfield = copy.deepcopy(field)
    curfield[position[0]][position[1]] = "X "
    curfield[goal[0]][goal[1]] = "G "
    for (x, y) in unreachables:
        curfield[x][y] = "##"
    #draw grid:
    for x in range(SX+2):
        print("##", end="")
    print()
    for x in range(SY):
        print("##", end="")
        for y in curfield[x]:
            print(y, end="")
        print("##")
    for x in range(SX+2):
        print("##", end="")
    print()
    sys.stdout.flush()
    time.sleep(0.1)
