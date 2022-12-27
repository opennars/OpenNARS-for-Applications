import gymnasium as gym
import minigrid
import NAR
import sys
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
NAR.AddInput("*motorbabbling=0.1")
actions = {"^left" : 0, "^right" : 1, "^forward" : 2, "^pick" : 3, "^toggle" : 5} #,  "^drop" : 4, "^drop" : 5, "^say" : 6} #
for i, x in enumerate(actions):
    NAR.AddInput("*setopname " + str(i+1) + " " + x)
goal = "G" # "<{200} --> [catched]>"
    
#Setup environment:
env = gym.make('MiniGrid-Empty-6x6-v0').env
env.reset(seed=1337)
viewDistance=3 #how many cells forward the agent can see

def coneForward():
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
        StartIndexX -=1
        indexX=StartIndexX
        indexY -=1
        width+=2
    L[0], L[1] = L[1], L[0] #make 3,5 the first element
    return L

def coneRight():
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
    return L

def coneLeft():
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
    return L

def scan(cone, cells, colorBlind=True):
    if colorBlind:
        cells[3][6][1] = 0
    L = cone()
    k = 0
    for (x,y,distance) in L:
        if colorBlind:
            cells[x][y][1] = 0
            cells[x][y][2] = 0
        if cells[x][y][0] != 1 and (k==0 or cells[x][y][0] != 2): #a seen object or physical contact with a wall
            if cells[x][y][0] == 0:
                cells[x][y][0] = 1 #not sure what 0 means, looks like a bug and it should be 1
            return distance, cells[x][y] #return first non-empty cell
        k += 1
    return 9999, np.array([1,0,0])

def stateconcat(state):
    return str(state).replace("\n","").replace(" ","")


def encode(direction, state):
    state = state.replace("[","").replace("]","")
    #return direction + state
    #ret = f"<{{{state}}} --> ([{direction}] & object)>"
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
    narsese = "(( " + right + " &| " + left + " ) &| (" + forward + " &| " + inventory + ") ). :|:"
    obj = nearestObject(cells)
    narsese = "( " + obj + " &| " + inventory + " ). :|:"
    return narsese

successes = 0
timestep = 0

#Similate for 100000 steps:
k=1
h=0
DisableToggle=False #strangely locked state isn't returned anymore so this is now necessary
for i in range(0, 100000):
    default_action = 6 #nop action
    action = default_action
    chosenAction = False
    executions = NAR.AddInput(goal + "! :|:", Print=True)["executions"]
    #executions += NAR.AddInput("2", Print=False)["executions"]
    if executions:
        chosenAction = True
        action = actions[executions[0]["operator"]] if executions[0]["operator"] in actions else default_action
    if not chosenAction:
        action = default_action
    if not DisableToggle and action == 5 and "obs" in globals() and obs is not None and obs["image"][3][6][0] == 5 and obs["image"][3][5][0] == 4 and obs["image"][3][5][2] == 0:
        DisableToggle = True
    elif action == 5 and DisableToggle:
        action = default_action
    obs, reward, done, info, _ = env.step(action)
    NAR.AddInput(observationToEvent(obs["image"]))
    env.step_count = 0 #avoids episode max_time reset cheat
    if done: #reward > 0:
        input()
        obs = None
        NAR.AddInput(goal + ". :|: {1.0 0.99}")
        successes += 1
    if action != default_action:
        print("successes=" + str(successes) + " time="+str(timestep))
        timestep += 1
    if done or (timestep+2 >= max_steps and max_steps != -1):
        DisableToggle = False
        NAR.AddInput("20") #don't temporally relate observations across reset
        h+=1
        if h % 5 == 0:
            k += 1
        if k == 2:
            env.close()
            env = gym.make('MiniGrid-Unlock-v0').env
            #env.seed(2)
        elif k == 3:
            env.close()
            env = gym.make('MiniGrid-DoorKey-8x8-v0').env
            #env.seed(1337)
        elif k == 5:
            env.close()
            env = gym.make('MiniGrid-DoorKey-16x16-v0').env
            #env.seed(11)
        elif k == 6:
            env.close()
            env = gym.make('MiniGrid-DoorKey-16x16-v0').env
            #env.seed(7)
        elif k == 7:
            break
        else:
            None
            #env.seed(1337+i)
        env.reset()
    #if max_steps == -1:
    #   env.render()
    renderANSI(env)
while timestep <= max_steps:
    print("successes=" + str(successes) + " time="+str(timestep))
    timestep += 1
print(timestep, max_steps)
env.close()

