"""
 * The MIT License
 *
 * Copyright 2020 The OpenNARS authors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * """

import re
import sys
import ast
import networkx as nx
import matplotlib.pyplot as plt

#Get input and arguments
NoTermlinks = "NoTermlinks" in sys.argv
NoEdgeLabels = "NoEdgeLabels" in sys.argv
lines = []
curline = None
G = nx.MultiDiGraph()
inlines = []
for line in sys.stdin:
    inlines.append(line)

#Utility functions:
def parse_concept(line):
    if line.startswith("//{i="):
        concept = " ".join(line.split(" ")[1:]).split(":")[0]
        dictionary = ast.literal_eval("{" + line.split(": {")[1])
        return (concept, dictionary)
    return (None, None)
    
def parse_truth(line):
    TV = line.split(". {")[1].split("}")[0].split(" ")
    return (float(TV[0]), float(TV[1]))

def truth_expectation(truth):
    (f,c) = truth
    return c * (f - 0.5) + 0.5
    
def truth_to_color(truth):
    truthcol = int(255*truth_expectation(truth))
    return '#%02x%02x%02x' % (truthcol,0, 255 - truthcol)
    
implicationEdges={}
def addImplicationEdge(a, b, operator, truth=[0, 0]):
    if (a,b) in implicationEdges:
        (operator2, truth2) = implicationEdges[(a,b)]
        if truth_expectation(truth) > truth_expectation(truth2) or (operator2 == "" and operator != ""):
            implicationEdges[(a,b)] = (operator, truth)
    else:
        implicationEdges[(a,b)] = (operator, truth)
    
def addImplicationEdges():
    for (a,b) in implicationEdges:
        (operator, truth) = implicationEdges[(a,b)]
        color = truth_to_color(truth)
        label = ("Op: " + operator + "\n" if operator != "" else "") + "{" + str(truth[0])+ " " + str(truth[1]) + "}"
        if NoEdgeLabels:
            label = ""
        max_rad = 0.0
        if (a, b) in G.edges:
            max_rad = max(x[2].get('rad', 0.0) for x in G.edges(data=True) if sorted(x[:2]) == sorted([a,b]))
        G.add_edge(a, b, rad=max_rad + 0.1, color=color, weight=4, label=label, arrowsize=20)

#Add statement concept nodes:
for line in inlines:
    (concept, dictionary) = parse_concept(line)
    if concept != None:
        truth = (dictionary["frequency"], dictionary["confidence"])
        del dictionary['termlinks']
        dictionary["color"] = truth_to_color(truth)
        dictionary["size"] = 1.0
        dictionary["label"] = concept
        if dictionary["confidence"] > 0:
            dictionary["label"] += "\n{" + str(truth[0]) + " " + str(truth[1]) + "}"
        if concept not in G:
            G.add_nodes_from([(concept, dictionary)])

#Add subterm nodes:
for line in inlines:
    (concept, dictionary) = parse_concept(line)
    if concept != None:
        if not NoTermlinks:
            for x in dictionary["termlinks"]:
                if x not in G and x != "":
                    variable = False
                    for vartype in ["\#", "\$", "\?"]:
                        if re.match(vartype + "[0-9]", x) != None or re.match("\{" + vartype + "[0-9]\}", x) != None or re.match("\[" + vartype + "[0-9]\]", x) != None:
                           variable = True
                    if not variable:
                        tldict = {"color": "gray"}
                        tldict["size"] = 1.0
                        tldict["label"] = x
                        G.add_nodes_from([(x, tldict)])
        
#Add subterm links:
def AddTermlink(source, target):
    if source in G and target in G and source != "" and target != "":
        G.add_edge(source, target, color='green', weight=1, label="", arrowsize=1)
        G.add_edge(target, source, color='green', weight=1, label="", arrowsize=1)
if not NoTermlinks:
    for line in inlines:
        (concept, dictionary) = parse_concept(line)
        if concept != None:
            tldict = dictionary["termlinks"]
            AddTermlink(concept, tldict[0])
            AddTermlink(concept, tldict[1])
            AddTermlink(tldict[0], tldict[2])
            AddTermlink(tldict[0], tldict[3])
            AddTermlink(tldict[1], tldict[4])
            AddTermlink(tldict[1], tldict[5])

#Add implication links:
for line in inlines:
    print(line)
    if (line.startswith("<") or line.startswith("dt=")) and " =/> " in line:
        term = line.split(". {")[0]
        if term.startswith("dt="):
            timing = float(term.split("dt=")[1].split(" ")[0])
            term = " ".join(term.split(" ")[1:])
        left = term.split(" =/> ")[0][1:]
        right = term.split(" =/> ")[1][:-1]
        operator = ""
        if " &/ " in left:
            operator = left.split(" &/ ")[-1][:-1]
            precondition = " &/ ".join(left.split(" &/ ")[:-1])[1:]
            if '^' in operator:
                left = precondition
            else:
                operator = ""
        truth = parse_truth(line)
        if left in G and right in G:
            addImplicationEdge(left, right, operator, truth)
addImplicationEdges()

#Draw the graph:
nodecolors = nx.get_node_attributes(G,'color').values()
nodesizes = nx.get_node_attributes(G,'size').values()
labels = nx.get_node_attributes(G, 'label') 
degrees = nx.degree(G)
nodesizes = [c * 100 for (node,c) in degrees]
pos = nx.kamada_kawai_layout(G)
nx.draw_networkx_nodes(G, pos, node_color = nodecolors, node_size = nodesizes)
nx.draw_networkx_labels(G, pos, labels)
edgelabels = {}
for edge in G.edges(data=True):
    edgelabels[(edge[0],edge[1])] = edge[2].get("label", "")
    nx.draw_networkx_edges(G, pos, edgelist=[(edge[0],edge[1])], edge_color=[edge[2]["color"]], width=[edge[2].get("weight", 1.0)], connectionstyle=f'arc3, rad = {edge[2].get("rad", 0.0)}', arrowsize=edge[2].get("arrowsize", 10.0))
nx.draw_networkx_edge_labels(G, pos, edge_labels = edgelabels, label_pos=0.8, bbox=dict(alpha=0))
nx.write_graphml(G, "memory.graphml")
plt.show()

