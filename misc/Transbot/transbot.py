#run in terminal: 
#jupyter notebook stop
#jupyter notebook
#roslaunch transbot_nav laser_bringup.launch
#roslaunch transbot_nav rrt_exploration.launch open_rviz:=true
import sys
import os
os.system("pkill NAR")
sys.path.append('/home/jetson/OpenNARS-for-Applications/misc/Python/')
import NAR
from time import sleep
from transbot_nav import *
from transbot_gripper import *
#from transbot_vision import *
#from geometry_msgs.msg import Twist

locationToTermOffset = 100.0
def TransbotExecute(executions):
    global Right_warning, Left_warning, Front_warning
    for execution in executions:
        op = execution["operator"]
        arguments = execution["arguments"]
        try:
            if op == "^forward":
                OpStop()
                OpGo(0.5, 0.0, 0.0, 1.0, frame_id = "base_link")
                sleep(2.0)
            elif op == "^left":
                OpStop()
                OpGo(0.0, 0.0, 0.5, 1.0, frame_id = "base_link")
                sleep(1.0)
            elif op == "^right":
                OpStop()
                OpGo(0.0, 0.0, -0.5, 1.0, frame_id = "base_link")
                sleep(1.0)
            elif op == "^pick":
                OpStop()
                pick()
            elif op == "^drop":
                OpStop()
                drop()
            elif op == "^activate": #for later
                None
            elif op == "^deactivate": #for later
                None
            elif op == "^remember":
                if arguments == "fridge":
                    locationQueryAnswer = NAR.AddInput("<(%s * ?where) --> at>? :|:" % arguments)["answers"][0]
                    if locationQueryAnswer["term"] != "None":
                        NAR.AddInput("<%s --> [localized]>. :|:" % arguments)
                        NAR.AddInput("%s. :|:" % (locationQueryAnswer["term"]))
            elif op == "^goto":
                (x,y) = arguments.split("_")
                print("GOTO: " + str((x, y)))
                (xf, yf) = (float(x)-locationToTermOffset, float(y)-locationToTermOffset)
                OpGo(xf, yf)
            elif op == "^say":
                print("SAY: " + arguments)
        except:
            print("execution of wrong format " + str(execution))

def TransbotPerceiveAt(obj, x, y):
    NAR.AddInput("<(%s * %f_%f) --> at>. :|:" % (obj, x+locationToTermOffset, y+locationToTermOffset))

def TransbotPerceiveVisual(obj, screenX, screenY):
    TransbotPerceiveAt(obj, location[0], location[1])
    direction = "center"
    if screenX < 500:
        direction = "left"
    elif screenX > 600:
        direction = "right"
    NAR.AddInput("<%s --> [%s]>. :|:" % (obj, direction))

def reset_ona():
    NAR.AddInput("*reset")
    NAR.AddInput("*setopname 1 ^forward")
    NAR.AddInput("*setopname 2 ^left")
    NAR.AddInput("*setopname 3 ^right")
    NAR.AddInput("*setopname 4 ^pick")
    NAR.AddInput("*setopname 5 ^drop")
    NAR.AddInput("*setopname 6 ^activate")
    NAR.AddInput("*setopname 7 ^deactivate")
    NAR.AddInput("*babblingops=3")
    NAR.AddInput("*setopname 8 ^remember")
    NAR.AddInput("*setopname 9 ^goto")
    NAR.AddInput("*setopname 10 ^say")
    NAR.AddInput("*motorbabbling=true")
    NAR.AddInput("*volume=0")
    #you need to ask the map in order to localize an object
    NAR.AddInput("<(tick &/ <({SELF} * $obj) --> ^remember>) =/> <$obj --> [localized]>>.")
    #once it's localized, and the location at the map is known, go to the location in order to see the object
    NAR.AddInput("<((<$obj --> [localized]> &/ <($obj * #location) --> at>) &/ <({SELF} * #location) --> ^goto>) =/> <$obj --> [see]>>.")
    #NAR.AddInput("<(a &/ ^forward) =/> G>.")
    #NAR.AddInput("a. :|:")
    for i in range(50):
        TransbotExecute(NAR.AddInput("G! :|:")["executions"])
    #CELL2: tell NARS about locations of objects:
    #TransbotPerceiveAt("fridge", -0.25, 1.4)

def process(line):
    if line != "":
        if line.endswith("! :|:"):
            executions = NAR.AddInput("<fridge --> [see]>! :|:")["executions"]
            executions += NAR.AddInput("10")["executions"]
            TransbotExecute(executions)

lastLine = ""
def transbot_shell():
    lastLine = ""
    while True:
        #Get input line and forward potential command
        try:
            line = input().rstrip("\n") #"the green cat quickly eats the yellow mouse in the old house"
        except:
            exit(0)
        if len(line.strip()) == 0:
            line = lastLine;
        else:
            lastLine = line
        process(line)

#CELL2:
#NAR.AddInput("tick. :|:")
#executions = NAR.AddInput("<fridge --> [see]>! :|:")["executions"]
#executions += NAR.AddInput("10")["executions"]
#TransbotExecute(executions)

#CELL3:
#NAR.AddInput("tick. :|:")
#executions = NAR.AddInput("<fridge --> [see]>! :|:")["executions"]
#executions += NAR.AddInput("10")["executions"]
#TransbotExecute(executions)
