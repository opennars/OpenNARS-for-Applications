import sys
import os
os.system("pkill NAR")
sys.path.append('../Python/')
import NAR
from time import sleep
import random

def detect_objects():
    return ([("bottle", random.randint(0,640), 479, 10, 10, 0.9)], "")

def getLocation():
    return [(0,0,0),(0,0,0,0)]

picked = False
frame = ""

def OpStop():
    None

def forward():
    None

def left():
    None

def right():
    None

def backward():
    None

def pick():
    None

def arm_down():
    None

def arm_up():
    None

def close_gripper():
    global picked
    picked = True
    return True

def open_gripper():
    None

def OpGo(x, y, z=0, w=1, frame_id = 'map'):
    None

def drop():
    global picked
    picked = False
    None

class FakeCV:
    def waitKey(self, wtf):
        return 0
    def imshow(self, frame, k):
        None
cv = FakeCV()

with open('transbot.py') as f:
    lines = f.readlines()

maxindex = 0
for i in range(len(lines)):
    if "import" in lines[i]:
        maxindex = i

code = "\n".join(lines[maxindex+1:])
exec(code)
