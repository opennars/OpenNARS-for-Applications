import gymnasium as gym
import minigrid
import NAR
import sys
import time
import numpy as np

max_steps = -1
try:
    max_steps = int(sys.argv[1])
except:
    None

BOLD = "\x1B[1"
BLUE = BOLD + ";34"
GREEN = BOLD + ";32"
YELLOW = BOLD + ";33"
WHITE = BOLD + ";97"
CYAN = BOLD + ";36"
MAGENTA = BOLD + ";35"
RED = BOLD + ";31"
RESET = "\x1B[0m"

def ASCIIpaint(COLOR, objectchar):
    if objectchar == "W":
        return WHITE + ";47m "
    elif objectchar == "L":
        return COLOR + "mL" #";47mL"
    elif objectchar == "V":
        return RED + "m" + "v"
    elif objectchar == "^":
        return RED + "m" + "^"
    elif objectchar == "<":
        return RED + "m" + "<"
    elif objectchar == ">":
        return RED + "m" + ">"
    else:
        return COLOR + "m" + objectchar

def colorify(x):
    S=""
    even = True
    lasts = ""
    for s in x:
        if even:
            lasts = s
        else:
            COLOR = WHITE
            if s == "G":
                COLOR = GREEN
            if s == "B":
                COLOR = BLUE
            if s == "R":
                COLOR = RED
            if s == "Y":
                COLOR = YELLOW
            S += ASCIIpaint(COLOR, lasts) + RESET
        if s == "\n":
            even = True
            S += "\n"
        else:
            even = not even
    return S

def renderANSI(env):
    print("\033[1;1H\033[2J")
    asciigrid = str(env).split("<PassiveEnvChecker")[1] #WG (wall green)
    print(colorify(asciigrid))
    print(RESET)

#Configure NARS:
NAR.AddInput("*volume=0")
NAR.AddInput("*babblingops=5")
NAR.AddInput("*motorbabbling=0.05")
actions = {"^left" : 0, "^right" : 1, "^forward" : 2, "^pick" : 3, "^toggle" : 5} #,  "^drop" : 4, "^drop" : 5, "^say" : 6} #
for i, x in enumerate(actions):
    NAR.AddInput("*setopname " + str(i+1) + " " + x)
goal = "G"
    
#Setup environment:
env = gym.make('MiniGrid-Empty-6x6-v0').env
env.reset(seed=1337)

def coneForward(viewDistance=6):
    L=[]
    index = 0
    StartIndexX, StartIndexY = (2,5)
    indexX, indexY = (StartIndexX,StartIndexY)
    width = 3 #cone starts with the 3 cells right in front of agent
    for k in range(viewDistance):
        for h in range(width):
            if index != 0 and index != 2: #remove corner items as the system can't toggle switches if diagonal
                L.append((indexX, indexY, k))
            indexX += 1
            index+=1
        StartIndexX = max(0, StartIndexX - 1)
        indexX=StartIndexX
        indexY -=1
        width=min(7,width+2)
    return L

def coneRight(viewDistance=3):
    L=[]
    StartIndexX, StartIndexY = (3,6)
    indexX, indexY = (StartIndexX,StartIndexY)
    for h in range(viewDistance):
        for k in range(viewDistance-h):
            xLeft = indexX+1
            L.append((xLeft, indexY, h))
            indexX+=1
        StartIndexX+=1
        indexX = StartIndexX
        indexY-=1
    L.insert(1, (4,5,1))
    return L

def coneLeft(viewDistance=3):
    L=[]
    StartIndexX, StartIndexY = (3,6)
    indexX, indexY = (StartIndexX,StartIndexY)
    for h in range(viewDistance):
        for k in range(viewDistance-h):
            xLeft = indexX-1
            L.append((xLeft, indexY, h))
            indexX-=1
        StartIndexX-=1
        indexX = StartIndexX
        indexY-=1
    L.insert(1, (2,5,1))
    return L

def scan(cone, cells, colorBlind=True, wall=False):
    if colorBlind:
        cells[3][6][1] = 0
    L = cone()
    for (x,y,distance) in L:
        if colorBlind:
            cells[x][y][1] = 0
        if cells[x][y][0] != 0 and cells[x][y][0] != 1 and (distance==0 or wall or cells[x][y][0] != 2): #a seen object or physical contact with a wall: #a seen object or physical contact with a wall
            return distance, cells[x][y] #return first non-empty cell
    if not wall:
        return scan(cone, cells, colorBlind=colorBlind, wall=True) #nearest wall
    return 9999, np.array([1,0,0])

def stateconcat(state):
    return str(state).replace("\n","").replace(" ","")

def encode(direction, state):
    state = state.replace("[","").replace("]","")
    ret = f"<{{{state}}} --> [{direction}]>"
    return ret

def nearestObject(cells):
    dist_f, forward = scan(coneForward, cells)
    dist_l, left = scan(coneLeft, cells)
    dist_r, right = scan(coneRight, cells)
    if dist_f <= dist_l and dist_f <= dist_r:
        return encode("forward", stateconcat(forward))
    if dist_l <= dist_f and dist_l <= dist_r:
        return encode("left", stateconcat(left))
    if dist_r <= dist_l and dist_r <= dist_f:
        return encode("right", stateconcat(right))

#Local grid vector to Narsese event:
def observationToEvent(cells):
    forward = encode("forward", stateconcat(scan(coneForward,cells)[1]))
    left = encode("left", stateconcat(scan(coneLeft,cells)[1]))
    right = encode("right", stateconcat(scan(coneRight,cells)[1]))
    inventory = encode("holding", stateconcat(cells[3][6]))
    obj = nearestObject(cells)
    narsese = "( " + obj + " &/ " + inventory + " ). :|:"
    return narsese

#Similate for 100000 steps:
successes = 0
timestep = 0
k=1
h=0
DisableToggle=False #strangely locked state isn't returned anymore so this is now necessary
for i in range(0, 10000000):
    default_action = 6 #nop action
    action = default_action
    chosenAction = False
    executions = NAR.AddInput(goal + "! :|:", Print=True)["executions"]
    for i in range(3):
        executions += NAR.AddInput("1", Print=False)["executions"]
    renderANSI(env)
    if executions:
        chosenAction = True
        action = actions[executions[0]["operator"]] if executions[0]["operator"] in actions else default_action
    if not chosenAction:
        action = default_action
    if not DisableToggle and action == 5 and "obs" in globals() and obs is not None and obs["image"][3][6][0] == 5 and obs["image"][3][5][0] == 4:
        DisableToggle = True
    elif action == 5 and DisableToggle:
        action = default_action
    obs, reward, done, info, _ = env.step(action)
    NAR.AddInput(observationToEvent(obs["image"]))
    env.step_count = 0 #avoids episode max_time reset cheat
    if max_steps == -1:
        time.sleep(0.001)
    if done:
        NAR.AddInput(goal + ". :|: {1.0 0.9}")
        if max_steps == -1:
            time.sleep(0.5)
        obs = None
        successes += 1
    if max_steps == -1 or action != default_action: #only record values once for external mode
        print("successes=" + str(successes) + " time="+str(timestep))
    if timestep >= max_steps and max_steps != -1:
        break
    if action != default_action:
        timestep += 1
    if done:
        DisableToggle = False
        NAR.AddInput("20") #don't temporally relate observations across reset
        h+=1
        if h % 10 == 0:
            k += 1
        if k == 3:
            env.close()
            env = gym.make('MiniGrid-Unlock-v0').env
        elif k == 4:
            env.close()
            env = gym.make('MiniGrid-DoorKey-8x8-v0').env
        elif k == 8:
            env.close()
            env = gym.make('MiniGrid-DoorKey-16x16-v0').env
        elif k == 9:
            env.close()
            env = gym.make('MiniGrid-DoorKey-16x16-v0').env
        elif k == 10:
            break
        else:
            None
        env.reset()
while timestep <= max_steps:
    print("successes=" + str(successes) + " time="+str(timestep))
    timestep += 1
env.close()

