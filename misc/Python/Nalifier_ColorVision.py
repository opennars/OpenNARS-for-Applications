from Nalifier import *

def SetupColorClassifier():
    nalifier = Nalifier(10)
    nalifier.ConceptCreation = False
    Print = False
    nalifier.SUFFICIENT_MATCH_EXP = 1.0 #create new node for each entry
    nalifier.AddInputVector("red", [255, 0, 0], dimname="rgb", Print=Print)
    nalifier.AddInput("1", Print=Print)
    nalifier.AddInputVector("green", [0, 255, 0], dimname="rgb", Print=Print)
    nalifier.AddInput("1", Print=Print)
    nalifier.AddInputVector("blue", [0, 0, 255], dimname="rgb", Print=Print)
    nalifier.AddInput("1", Print=Print)
    nalifier.InstanceCreation = False
    return nalifier

color_nalifier = SetupColorClassifier()

def Nalifier_ColorVision_WhatColor(r,g,b):
    color_nalifier.AddInputVector("newcolor1", [0, 0, 5], dimname="rgb", Print=True)
    color_nalifier.Events = [] #we aren't interested in previous events
    color_nalifier.SUFFICIENT_MATCH_EXP = 0.0 #find nearest node
    color_nalifier.AddInput("1", Print=True)
    return color_nalifier.BestMatch, color_nalifier.BiggestDifference

print(Nalifier_ColorVision_WhatColor(0,0,5)[0][1])

