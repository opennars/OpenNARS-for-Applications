import NAR
import multiprocessing
from joblib import Parallel, delayed

#Node graph:
master = "narMaster"
nodes = [master, "narSlave1", "narSlave2"]
edges = [("narSlave1", master), ("narSlave2", master)]
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
        narsese = selectionToNarsese(selection)
        if Down: #parallelelization only safe down so far assuming child has just one parent, else locking is necessary in addition
            #Parallel(n_jobs=num_cores, require='sharedmem')(delayed(Trickle)(x, selectOfPunctuation(NAR.AddInput(narsese, Print=True, usedNAR=processes[x])["selections"], "!"), Down=True) for x in incoming[node])
            for x in incoming[node]:
                Trickle(x, selectOfPunctuation(NAR.AddInput(narsese, Print=False, usedNAR=processes[x])["selections"], "!"), Down=True)
        else:
            for x in outgoing[node]:
                Trickle(x, selectOfPunctuation(NAR.AddInput(narsese, Print=False, usedNAR=processes[x])["selections"], "."), Down=False)

#Add input to the NARS graph
def AddInput(narsese, node = None):
    if narsese.endswith("! :|:"): #is goal
        Trickle(master, selectOfPunctuation(NAR.AddInput(narsese, Print=False, usedNAR = processes[master])["selections"], "!"), Down=True)
    else:
        Trickle(node, selectOfPunctuation(NAR.AddInput(narsese, Print=False, usedNAR = processes[node])["selections"], "."), Down=False)

#Example:
if __name__ == "__main__":
    AddInput("G! :|:")
    AddInput("b. :|:", "narSlave1")
    AddInput("c. :|:", "narSlave2")
