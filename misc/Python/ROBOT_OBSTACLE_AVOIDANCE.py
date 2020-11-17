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

#Robot which quickly learns to avoid obstacles
#recommended Python2, not Python3, its nxt-python is known to be incomplete/unstable
#needs nxt-python and either pyusb or pybluez

import sys
import random
import nxt.locator
import NAR
from nxt.sensor import *
from nxt.motor import *
from time import sleep

b = nxt.locator.find_one_brick()
m_left = Motor(b, PORT_A)
m_right = Motor(b, PORT_C)

def interrupt():
    if Touch(b, PORT_1).get_sample():
        m_left.idle()
        m_right.idle()
        exit(0)

def forward():
    interrupt()
    m_left.run(power=64)
    m_right.run(power=64)
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

def scan():
    Proximity = 1000
    Proximity = min(Proximity, Ultrasonic(b, PORT_2).get_sample())
    left(doScan=True)
    Proximity = min(Proximity, Ultrasonic(b, PORT_2).get_sample())
    right(doScan=True)
    Proximity = min(Proximity, Ultrasonic(b, PORT_2).get_sample())
    right(doScan=True)
    Proximity = min(Proximity, Ultrasonic(b, PORT_2).get_sample())
    left(doScan=True)
    Proximity = min(Proximity, Ultrasonic(b, PORT_2).get_sample())
    return Proximity < 18


def ExecMotorCommands(executions, DisableForward=False):
    action = None
    for execution in executions:
        print(execution)
        if execution["operator"] == "^right":
            action = right()
        elif execution["operator"] == "^left":
            action = left()
        elif execution["operator"] == "^up" and not DisableForward:
            print("FORWARD ACTION WTF")
            action = forward() #if the hardware is strong enough, this line is fine however
    return action

def NARAddInput(narsese):
    print(narsese)
    return NAR.AddInput(narsese)

hadProximity = False
def NAR_Invoke(Proximity):
    global hadProximity
    if Proximity:
        NARAddInput("<obstacle --> [observed]>. :|:") #TODO also encode color
    else:
        NARAddInput("<nothing --> [observed]>. :|:") #TODO also encode color
    action = None
    if Proximity: #Don't allow forward as a reflex to not damage hardware
        executions = NARAddInput("(! collision)! :|:")["executions"]
        action = ExecMotorCommands(executions, DisableForward=True)
    else:
        executions = NARAddInput("forward! :|:")["executions"]
        action = ExecMotorCommands(executions)
    if not Proximity and hadProximity and action == "forward":
        NARAddInput("(! collision). :|:") #moved away from collision state
    if action == "forward":
        NARAddInput("forward. :|:")
    hadProximity = Proximity

while True:
    #1. Actively retrieve sensor input
    Proximity = scan()
    #2. Let NARS decide what to do
    NAR_Invoke(Proximity)
