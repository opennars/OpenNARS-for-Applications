import NAR

#Node graph:
master = "narMaster"
nodes = [master, "narSlave1", "narSlave2"]
edges = [("narSlave1", master), ("narSlave2", master)]

#Spawn processes:
processes = {master: NAR.getNAR()}
for x in nodes:
    if x != master:
        processes[x] = NAR.spawnNAR()

#Filter for belief and goal selections:
def selectOfPunctuation(selections, punctuation):
    return [x for x in selections if x["punctuation"] == punctuation]

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

#Translate selection dict to input Narsese
def selectionToNarsese(selection):
    return selection["term"] + selection["punctuation"] + " :|: {" + str(selection["truth"]["frequency"]) + " " + str(selection["truth"]["confidence"]) + "}"

def Trickle(node, selections, Down=True):
    for selection in selections:
        narsese = selectionToNarsese(selection)
        if Down:
            for x in incoming[node]:
                print("//Arriving in node:", x)
                NAR.setNAR(processes[x])
                Trickle(x, selectOfPunctuation(NAR.AddInput(narsese)["selections"], "!"), Down=True)
        else:
            for x in outgoing[node]:
                print("//Arriving in node:", x)
                NAR.setNAR(processes[x])
                Trickle(x, selectOfPunctuation(NAR.AddInput(narsese, Print=True)["selections"], "."), Down=False)

def AddInput(narsese, node = None):
    if narsese.endswith("! :|:"): #is goal
        print("//Arriving in node:", master)
        NAR.setNAR(processes[master])
        Trickle(master, selectOfPunctuation(NAR.AddInput(narsese)["selections"], "!"), Down=True)
    else:
        print("//Arriving in node:", node)
        NAR.setNAR(processes[node])
        Trickle(node, selectOfPunctuation(NAR.AddInput(narsese)["selections"], "."), Down=False)

AddInput("G! :|:")
AddInput("b. :|:", "narSlave1")
