import NAR
import multiprocessing
from joblib import Parallel, delayed

#Node graph:
master = "narMaster"
nodes = [master, "narSlave1", "narSlave2"]
edges = [("narSlave1", master), ("narSlave2", master)]
priorityThresholdGoals = 0.1 #priority needed for communication of a task
priorityThresholdBeliefs = 0.1 #priority needed for communication of a task
num_cores = multiprocessing.cpu_count()

#Spawn processes:
processes = {master: NAR.getNAR()}
for x in nodes:
    if x != master:
        processes[x] = NAR.spawnNAR()

#Build efficient graph structure with ingoing/outgoing nodes for each node
incoming = {} #dict of node -> set associations
outgoing = {} #dict of node -> set associations
for x in nodes: #initialize empty sets for each node's incoming/outgoing nodes
    incoming[x] = set([])
    outgoing[x] = set([])
for (From, To) in edges:
    #Add incoming node From to To:
    incoming[To].add(From)
    #Add outgoing node To to From:
    outgoing[From].add(To)

#Filter for belief and goal selections:
def selectOfPunctuation(selections, punctuation):
    return [x for x in selections if x["punctuation"] == punctuation]

#Translate selection dict to input Narsese
def selectionToNarsese(selection):
    return selection["term"] + selection["punctuation"] + " :|: {" + str(selection["truth"]["frequency"]) + " " + str(selection["truth"]["confidence"]) + "}"

#Transport informations (beliefs up, goals down)
def Trickle(node, selections, Down=True):
    print("//^ in node:", node)
    for selection in selections:
        if Down and float(selection["Priority"]) < priorityThresholdGoals:
            continue
        if not Down and float(selection["Priority"]) < priorityThresholdBeliefs:
            continue
        narsese = selectionToNarsese(selection)
        if Down:
            for x in incoming[node]:
                Trickle(x, selectOfPunctuation(NAR.AddInput(narsese, Print=True, usedNAR=processes[x])["selections"], "!"), Down=True)
        else:
            for x in outgoing[node]:
                Trickle(x, selectOfPunctuation(NAR.AddInput(narsese, Print=True, usedNAR=processes[x])["selections"], "."), Down=False)

def PerformIndependentSteps(ticks=1):
    Parallel(n_jobs=num_cores, require='sharedmem')(delayed(NAR.AddInput)(str(ticks), Print=True, usedNAR=x) for x in processes.values())

#Add input to the NARS graph
def AddInput(narsese, node = None):
    if narsese.isdigit():
        PerformIndependentSteps(int(narsese))
    elif narsese.endswith("! :|:"): #is goal
        Trickle(master, selectOfPunctuation(NAR.AddInput(narsese, Print=True, usedNAR = processes[master])["selections"], "!"), Down=True)
    else:
        Trickle(node, selectOfPunctuation(NAR.AddInput(narsese, Print=True, usedNAR = processes[node])["selections"], "."), Down=False)

#Example:
if __name__ == "__main__":
    AddInput("G! :|:")
    AddInput("a. :|:", node="narSlave1")
    AddInput("b. :|:", node="narSlave1")
    AddInput("50")
