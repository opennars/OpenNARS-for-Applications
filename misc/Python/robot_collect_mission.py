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
import time
from nxt.sensor import *
from nxt.motor import *
from nxt.bluesock import *
from time import sleep

b = nxt.bluesock.BlueSock('00:16:53:0D:6B:42').connect() #use if Bluetooth and already paired, use right address!
#b = nxt.locator.find_one_brick() #use instead if USB is desired
m_gripper = Motor(b, PORT_A)
m_left = Motor(b, PORT_B)
m_right = Motor(b, PORT_C)

def close():
    m_gripper.run(power=-70)
    sleep(4.0)
    #m_gripper.idle() #no idle since grab&lift demands power to keep the object lifted up

def open():
    m_gripper.run(power=70)
    sleep(2.0)
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
    P = 70 if closed_gripper else 64
    m_left.run(power=P*mul)
    m_right.run(power=P*mul)
    sleep(1.3)
    m_left.idle()
    m_right.idle()
    return "forward"
    
def left(doScan=False):
    interrupt()
    P = 70 if closed_gripper else 64
    m_left.run(power=P)
    m_right.run(power=-P)
    sleep(0.45 if doScan==True else 1.0)
    m_left.idle()
    m_right.idle()
    return "left"
    
def right(doScan=False):
    interrupt()
    P = 70 if closed_gripper else 64
    m_left.run(power=-P)
    m_right.run(power=P)
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

VisibleObjects = ["bottle"]

def NAR_Invoke(Proximity, VisualEvents):
    SeenSomethingMissionRelevant = False
    if Proximity:
        NAR.AddInput("<obstacle --> [observed]>. :|:") #TODO also encode color
    else:
        if closed_gripper:
            NAR.AddInput("<gripper --> [closed]>. :|:")
        else:
            NAR.AddInput("<gripper --> [open]>. :|:")
        if len(VisualEvents) > 0:
            Observed = False
            for obj in VisibleObjects:
                for v in VisualEvents:
                    print(v)
                    if obj in v:
                        NAR.AddInput(v)
                        SeenSomethingMissionRelevant = True
        if not SeenSomethingMissionRelevant:
            NAR.AddInput("(! <obstacle --> [observed]>). :|:") #TODO also encode color
    action = None
    if Proximity and not SeenSomethingMissionRelevant: #Don't allow forward as a reflex to not damage hardware
        executions = NAR.AddInput("(! <obstacle --> [observed]>)! :|:")["executions"]
        action = ExecMotorCommands(executions, DisableForward=True)
    else:
        executions = NAR.AddInput("<mission --> [progressed]>! :|:" if SeenSomethingMissionRelevant else "<{SELF} --> [moved]>! :|:")["executions"]
        executions += NAR.AddInput("5")["executions"]
        action = ExecMotorCommands(executions)
    if action == "forward":
        NAR.AddInput("<{SELF} --> [moved]>. :|:")

BackgroundKnowledge = """
//What's expected by the robot to learn:
//move forward if nothing is seen (due to innate boredom/move goal)
//<((! <obstacle --> [observed]>) &/ ^forward) =/> <{SELF} --> [moved]>>.
//move left when an obstacle is in front (due to innate collision pain to avoid)
//<(<obstacle --> [observed]> &/ ^left) =/> (! <obstacle --> [observed]>)>.
//How to focus on objects (comment out if it should also be learned!)
<(<$1 --> [smallerX]> &/ ^left) =/> <$1 --> [equalX]>>.
<(<$1 --> [largerX]> &/ ^right) =/> <$1 --> [equalX]>>.
//Mission description:
//1. Pick a bottle if it's in front
<((<gripper --> [open]> &/ <bottle --> [equalX]>) &/ ^pick) =/> <mission --> [progressed]>>.
//2. Drop grabbed to other bottles
<((<gripper --> [closed]> &/ <bottle --> [equalX]>) &/ ^drop) =/> <mission --> [progressed]>>.
"""

NAR.AddInput("*babblingops=3")
NAR.AddInput("*motorbabbling=0.3")
NAR.AddInput("*setopname 1 ^left")
NAR.AddInput("*setopname 2 ^right")
NAR.AddInput("*setopname 3 ^forward")
NAR.AddInput("*setopname 4 ^pick")
NAR.AddInput("*setopname 5 ^drop")

k=0
for bg in BackgroundKnowledge.split("\n"):
    bgstr = bg.strip()
    if len(bgstr) > 0:
        NAR.AddInput(bgstr)

while True:
    #1. Actively retrieve sensor input
    Proximity = scan()
    VisualEvents = subprocess.check_output("python3 vision_to_narsese.py once", shell=True, stderr=subprocess.STDOUT).decode("utf-8").split('\n')
    #2. Let NARS decide what to do
    NAR_Invoke(Proximity, VisualEvents)
    k+=1
