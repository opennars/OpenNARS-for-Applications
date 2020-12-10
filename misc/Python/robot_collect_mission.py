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

#Robot completing a mission:
#bringing a bottle to the other bottles, while avoiding obstacles
#needs nxt-python and either pyusb or pybluez

import sys
import random
import nxt.locator
import NAR
import subprocess
from nxt.sensor import *
from nxt.motor import *
from time import sleep

b = nxt.locator.find_one_brick()
m_gripper = Motor(b, PORT_A)
m_left = Motor(b, PORT_B)
m_right = Motor(b, PORT_C)

def close():
    m_gripper.run(power=-60)
    sleep(0.5)
    m_gripper.idle()

def open():
    m_gripper.run(power=60)
    sleep(0.5)
    m_gripper.idle()

closed_gripper = False #already picked something

def interrupt():
    if Touch(b, PORT_2).get_sample():
        m_left.idle()
        m_right.idle()
        if closed_gripper:
            open()
        exit(0)

def forward(mul=1):
    interrupt()
    m_left.run(power=64*mul)
    m_right.run(power=64*mul)
    sleep(1.3)
    m_left.idle()
    m_right.idle()
    return "forward"
    
def left(doScan=False):
    interrupt()
    m_left.run(power=64)
    m_right.run(power=-64)
    sleep(0.45 if doScan==True else 1.0)
    m_left.idle()
    m_right.idle()
    return "left"
    
def right(doScan=False):
    interrupt()
    m_left.run(power=-64)
    m_right.run(power=64)
    sleep(0.45 if doScan==True else 1.0)
    m_left.idle()
    m_right.idle()
    return "right"

def gripper_pick():
    global closed_gripper
    interrupt()
    if closed_gripper:
        return
    forward()
    forward()
    close()
    closed_gripper = True
    return "gripper_pick"

def gripper_drop():
    global closed_gripper
    if not closed_gripper:
        return
    forward()
    forward()
    forward()
    open()
    forward(mul=-1)
    forward(mul=-1)
    closed_gripper = False
    return "gripper_drop"

def scan():
    Proximity = 1000
    Proximity = min(Proximity, Ultrasonic(b, PORT_3).get_sample())
    left(doScan=True)
    Proximity = min(Proximity, Ultrasonic(b, PORT_3).get_sample())
    right(doScan=True)
    Proximity = min(Proximity, Ultrasonic(b, PORT_3).get_sample())
    right(doScan=True)
    Proximity = min(Proximity, Ultrasonic(b, PORT_3).get_sample())
    left(doScan=True)
    Proximity = min(Proximity, Ultrasonic(b, PORT_3).get_sample())
    return Proximity < 20

def ExecMotorCommands(executions, DisableForward=False):
    action = None  #DisableForward as an optional reflex to preserve hardware integrity if already too close
    for execution in executions:
        print(execution)
        if execution["operator"] == "^left":
            action = left()
        elif execution["operator"] == "^right":
            action = right()
        elif execution["operator"] == "^forward" and not DisableForward: #forward or gripper-pick
            action = forward()
        elif execution["operator"] == "^pick" and not DisableForward:
            action = gripper_pick()
        elif execution["operator"] == "^drop" and not DisableForward: #currently used for gripper-drop
            action = gripper_drop()
    return action

def NARAddInput(narsese):
    print(narsese)
    return NAR.AddInput(narsese)

VisibleObjects = ["bottle"]

def NAR_Invoke(Proximity, VisualEvents):
    SeenSomethingMissionRelevant = False
    if Proximity:
        NARAddInput("<obstacle --> [observed]>. :|:") #TODO also encode color
    else:
        if closed_gripper:
            NARAddInput("<gripper --> [closed]>. :|:")
        else:
            NARAddInput("<gripper --> [open]>. :|:")
        if len(VisualEvents) > 0:
            Observed = False
            for obj in VisibleObjects:
                for v in VisualEvents:
                    print(v)
                    if obj in v:
                        NARAddInput(v)
                        SeenSomethingMissionRelevant = True
        if not SeenSomethingMissionRelevant:
            NARAddInput("(! <obstacle --> [observed]>). :|:") #TODO also encode color
    action = None
    if Proximity and not SeenSomethingMissionRelevant: #Don't allow forward as a reflex to not damage hardware
        executions = NARAddInput("(! <obstacle --> [observed]>)! :|:")["executions"]
        action = ExecMotorCommands(executions, DisableForward=True)
    else:
        executions = NARAddInput("<mission --> [progressed]>! :|:" if SeenSomethingMissionRelevant else "<{SELF} --> [moved]>! :|:")["executions"]
        executions += NARAddInput("5")["executions"]
        action = ExecMotorCommands(executions)
    if action == "forward":
        NARAddInput("<{SELF} --> [moved]>. :|:")

BackgroundKnowledge = """
//What's expected by the robot to learn:
//move forward if nothing is seen (due to innate boredom/move goal)
//<((! <obstacle --> [observed]>) &/ ^forward) =/> <{SELF} --> [moved]>>.
//move left when an obstacle is in front (due to innate collision pain to avoid)
//<(<obstacle --> [observed]> &/ ^left) =/> (! <obstacle --> [observed]>)>.
//How to focus on a bottle (comment out if it should also be learned!)
<(<bottle --> [smallerX]> &/ ^left) =/> <bottle --> [equalX]>>.
<(<bottle --> [largerX]> &/ ^right) =/> <bottle --> [equalX]>>.
//Mission description:
//1. Pick a bottle if it's in front
<((<gripper --> [open]> &/ <bottle --> [equalX]>) &/ ^pick) =/> <mission --> [progressed]>>.
//2. Drop grabbed to other bottles
<((<gripper --> [closed]> &/ <bottle --> [equalX]>) &/ ^drop) =/> <mission --> [progressed]>>.
"""

k=0
for bg in BackgroundKnowledge.split("\n"):
    bgstr = bg.strip()
    if len(bgstr) > 0:
        NAR.AddInput(bgstr)
NARAddInput("*babblingops=3")
NARAddInput("*motorbabbling=0.3")
NARAddInput("*setopname 1 ^left")
NARAddInput("*setopname 2 ^right")
NARAddInput("*setopname 3 ^forward")
NARAddInput("*setopname 4 ^pick")
NARAddInput("*setopname 5 ^drop")
while True:
    #1. Actively retrieve sensor input
    Proximity = scan()
    VisualEvents = subprocess.check_output("python3 vision_to_narsese.py once", shell=True, stderr=subprocess.STDOUT).decode("utf-8").split('\n')
    #2. Let NARS decide what to do
    NAR_Invoke(Proximity, VisualEvents)
    k+=1
