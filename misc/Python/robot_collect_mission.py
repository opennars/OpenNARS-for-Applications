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

def forward(power=64, mul=1):
    interrupt()
    m_left.run(power=power*mul)
    m_right.run(power=power*mul)
    sleep(1.1)
    m_left.idle()
    m_right.idle()
    return "forward"

SCANSPEED = 0.35
TURNSPEED = 0.55
def left(doScan=False):
    interrupt()
    m_left.run(power=64)
    m_right.run(power=-64)
    sleep(SCANSPEED if doScan==True else TURNSPEED)
    m_left.idle()
    m_right.idle()
    return "left"
    
def right(doScan=False):
    interrupt()
    m_left.run(power=-64)
    m_right.run(power=64)
    sleep(SCANSPEED if doScan==True else TURNSPEED)
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
    action = None
    for execution in executions:
        print(execution)
        if execution["operator"] == "^right":
            action = right()
        elif execution["operator"] == "^left":
            action = left()
        elif execution["operator"] == "^up" and not DisableForward: #forward or gripper-pick
            if execution["operator"] == "^up":
                action = forward() #if the hardware is strong enough, this line is fine however
        elif execution["operator"] == "^down":
                action = gripper_pick()
        elif execution["operator"] == "^say": #currently used for gripper-drop
            action = gripper_drop()
    return action

def NARAddInput(narsese):
    print(narsese)
    return NAR.AddInput(narsese)

VisibleObjects = ["bottle"]

hadProximity = False
def NAR_Invoke(Proximity, VisualEvents):
    global hadProximity
    SeenSomethingMissionRelevant = False
    if Proximity:
        NARAddInput("<obstacle --> [observed]>. :|:") #TODO also encode color
    else:
        if closed_gripper:
            NARAddInput("closed. :|:")
        else:
            NARAddInput("open. :|:")
        if len(VisualEvents) > 0:
            Observed = False
            for obj in VisibleObjects:
                for v in VisualEvents:
                    print(v)
                    if obj in v:
                        NARAddInput(v)
                        SeenSomethingMissionRelevant = True
        if not SeenSomethingMissionRelevant:
            NARAddInput("<nothing --> [observed]>. :|:") #TODO also encode color
    action = None
    if Proximity and not SeenSomethingMissionRelevant: #Don't allow forward as a reflex to not damage hardware
        executions = NARAddInput("(! collision)! :|:")["executions"]
        action = ExecMotorCommands(executions, DisableForward=True)
    else:
        executions = NARAddInput("<mission --> [progressed]>! :|:" if SeenSomethingMissionRelevant else "forward! :|:")["executions"]
        action = ExecMotorCommands(executions)
        executions.append(NARAddInput("5")["executions"])
    if not Proximity and hadProximity and action == "forward":
        NARAddInput("(! collision). :|:") #moved away from collision state
    if action == "forward":
        NARAddInput("forward. :|:")
    hadProximity = Proximity

BackgroundKnowledge = """
//Let's say robot already learned to navigate from previous experiment:
//meaning to move forward if nothing is seen (due to innate boredom/forward goal)
<(<nothing --> [observed]> &/ ^up) =/> forward>.
//and to move left when an obstacle is in front (due to innate collision pain to avoid)
<(<obstacle --> [observed]> &/ ^left) =/> (! collision)>.
//Also easily learnable from observations with bottles:
<(<bottle --> [smallerX]> &/ ^left) =/> <bottle --> [equalX]>>.
<(<bottle --> [largerX]> &/ ^right) =/> <bottle --> [equalX]>>.
//Mission description:
//2. Grab a bottle if it's in front
<((open &/ <bottle --> [equalX]>) &/ ^down) =/> <mission --> [progressed]>>.
//3. Put grabbed to other bottles
<((closed &/ <bottle --> [equalX]>) &/ ^say) =/> <mission --> [progressed]>>.
"""

k=0
for bg in BackgroundKnowledge.split("\n"):
    bgstr = bg.strip()
    if len(bgstr) > 0:
        NAR.AddInput(bgstr)
NARAddInput("*motorbabbling=false")

while True:
    #1. Actively retrieve sensor input
    Proximity = scan()
    VisualEvents = subprocess.check_output("python3 vision_to_narsese.py once", shell=True, stderr=subprocess.STDOUT).decode("utf-8").split('\n')
    #2. Let NARS decide what to do
    NAR_Invoke(Proximity, VisualEvents)
    k+=1
