import time
import sys
import copy
import random
random.seed(42)
import NAR
NAR.AddInput("*volume=0")

def state(XY):
    return str(XY[0]) + "_" + str(XY[1])

def contingency(pre, op, cons):
    return ("<(" + state(pre) + " &/ ?1) =/> " + state(cons) + ">?", "<(" + state(pre) + " &/ " + op + ") =/> " + state(cons) + ">")

SX = 6
SY = 6
unreachables = {(3,3), (3,4), (3,5), (5,3)} # (6,7), (6,8), (6,9)
position = (0,0)
goal = (1,1)

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
        return execution["operator"]

field = [['  ' for x in range(SX)] for y in range(SY)]
k = 0

def checkAnswer(answers, solution):
    if not answers or answers[0]["term"] == "None":
        return "None"
    if answers[0]["term"] == solution:
        return "Correct " + str(answers[0]["truth"])
    return "Incorrect " + str(answers[0]["truth"])

while True:
    k += 1
    if k % 1000 == 0 or position == goal:
        while True:
            goal = (random.randint(0, SX-1), random.randint(0, SY-1))
            if goal not in unreachables:
                break
    NAR.AddInput(state(position) + ". :|:")
    executions = NAR.AddInput(state(goal) + "! :|:")["executions"]
    for i in range(5):
        executions += NAR.AddInput("1", Print=False)["executions"]
    oldPos = position
    op = execute(executions)
    if op != None:
       contingency(oldPos, op, position)
    print("\033[1;1H\033[2J") #clear screen
    #prepare grid:
    curfield = copy.deepcopy(field)
    curfield[position[0]][position[1]] = "X."
    curfield[goal[0]][goal[1]] = "G!"
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
    #check knowledge:
    x = position[0]
    y = position[1]
    cur = (x, y)
    left = (x-1, y)
    right = (x+1, y)
    up = (x, y+1)
    down = (x, y-1)
    left_statement, left_answer = contingency(left, "^right", cur)
    right_statement, right_answer = contingency(right, "^left", cur)
    up_statement, up_answer = contingency(up, "^down", cur)
    down_statement, down_answer = contingency(down, "^up", cur)
    if x > 0 and not (x-1, y) in unreachables:
        print("left: " + checkAnswer(NAR.AddInput(left_statement, Print=False)["answers"], left_answer))
    if x < SX-1 and not (x+1, y) in unreachables:
        print("right: " + checkAnswer(NAR.AddInput(right_statement, Print=False)["answers"], right_answer))
    if y > 0 and not (x, y-1) in unreachables:
        print("up: " + checkAnswer(NAR.AddInput(down_statement, Print=False)["answers"], down_answer))
    if y < SY-1 and not (x, y+1) in unreachables:
        print("down: " + checkAnswer(NAR.AddInput(up_statement, Print=False)["answers"], up_answer))
    sys.stdout.flush()
    time.sleep(0.05)
    
