import gym
import gym_minigrid
import NAR
import sys

max_steps = 999999999999999
try:
    max_steps = int(sys.argv[1])
except:
    None

#Configure NARS:
NAR.AddInput("*volume=0")
NAR.AddInput("*babblingops=5")
actions = {"^left" : 0, "^right" : 1, "^forward" : 2, "^pick" : 3, "^toggle" : 5} #"^drop" : 4
for i, x in enumerate(actions):
    NAR.AddInput("*setopname " + str(i+1) + " " + x)
goal = "G"
    
#Setup environment:
env = gym.make('MiniGrid-Empty-6x6-v0').env
env.reset()

#Local grid vector to Narsese event:
def observationToEvent(cells, colorBlind=True):
    if colorBlind:
        cells[3][5][1] = 0
        cells[4][6][1] = 0
        cells[2][6][1] = 0
    stateconcat = lambda state: str(state).replace("\n","").replace("[","").replace("]","").replace(" ","")
    forward = stateconcat(cells[3][5])
    left = stateconcat(cells[4][6])
    right = stateconcat(cells[2][6])
    return "(( " + right + "r &| " + left + "l ) &| " + forward + "f ). :|:"

successes = 0
timestep = 0
cur_steps = 0

#Similate for 100000 steps:
k=1
for i in range(0, 100000):
    default_action = 6 #nop action
    action = default_action
    chosenAction = False
    executions = NAR.AddInput(goal + "! :|:", Print=True)["executions"]
    executions += NAR.AddInput("5", Print=False)["executions"]
    print("successes=" + str(successes) + " time="+str(timestep))
    if executions:
        chosenAction = True
        action = actions[executions[0]["operator"]] if executions[0]["operator"] in actions else default_action
    if not chosenAction:
        action = default_action
    if action == 5 and "obs" in globals() and obs["image"][3][5][2] == 0:
        action = default_action
    if action != default_action:
        cur_steps += 1
        timestep += 1
    obs, reward, done, info = env.step(action)
    NAR.AddInput(observationToEvent(obs["image"]))
    env.step_count = 0 #avoids episode max_time reset cheat
    if reward > 0:
        NAR.AddInput(goal + ". :|:")
        successes += 1
    elif cur_steps > max_steps:
        successes += 1 #lift to next level
    if done or cur_steps > max_steps:
        NAR.AddInput("20") #don't temporally relate observations across reset
        k += 1
        if k == 2:
            env.close()
            env = gym.make('MiniGrid-Unlock-v0').env
            env.seed(2)
        elif k == 3:
            env.close()
            env = gym.make('MiniGrid-DoorKey-5x5-v0').env
            env.seed(1337)
        elif k == 5:
            env.close()
            env = gym.make('MiniGrid-DoorKey-6x6-v0').env
            env.seed(11)
        elif k == 6:
            env.close()
            env = gym.make('MiniGrid-DoorKey-8x8-v0').env
            env.seed(7)
        elif k == 7:
            break
        else:
            env.seed(1337+i)
        env.reset()
        cur_steps = 0
    env.render()
env.close()
