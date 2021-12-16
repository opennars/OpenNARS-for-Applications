#run in terminal: 
#jupyter notebook stop
#jupyter notebook
#roslaunch transbot_nav laser_bringup.launch
#roslaunch transbot_nav rrt_exploration.launch open_rviz:=false
#roslaunch astra_camera astrapro.launch

import sys
import os
os.system("pkill NAR")
sys.path.append('/home/jetson/OpenNARS-for-Applications/misc/Python/')
import NAR
import cv2 as cv
from time import sleep
from transbot_nav import *
from transbot_gripper import *
from transbot_vision import *

centerSize = 15
def pick_with_feedback(pickobj=None):
    arm_down()
    sleep(1)
    while True:
        action = cv.waitKey(10) & 0xFF
        detections, frame = detect_objects()
        y_real_temp = -1
        x_real_temp = -1
        for detection in detections:
            (obj, x, y, w, h, c) = detection
            x_real = x+w/2
            y_real = y+h #down side of bb
            if y_real > y_real_temp and (pickobj == None or pickobj == obj):
                y_real_temp = y_real
                x_real_temp = x_real
        if y_real_temp != -1:
            mid = 360 #it's a bit to the right
            if y_real_temp < 340:
                arm_up()
                break
            if x_real_temp >= mid-centerSize and x_real_temp <= mid+centerSize:
                print("//CENTER------------")
                closer_to_gripper = 475
                if y_real_temp < closer_to_gripper: #visual feedback
                    forward()
                elif y_real_temp > closer_to_gripper:
                    forward()
                    forward()
                    forward()
                    success = close_gripper() #gripper feedback
                    if success:
                        print("//pick succeeded")
                        arm_up()
                        drop() #TODO remove
                        break #focused
                    else:
                        print("//pick failed")
                        break
            elif x_real_temp > mid+centerSize:
                print("//RIGHT<<<<<<<<<<<<<<<<")
                right()
            elif x_real_temp < mid-centerSize:
                print("//LEFT>>>>>>>>>>>>>>>")
                left()
        else:
            arm_up()
            break
        #print(detections)
        cv.imshow('frame', frame)
        sleep(1.0)

valueToTermOffset = 500.0
def TransbotExecute(executions):
    global Right_warning, Left_warning, Front_warning
    for execution in executions:
        op = execution["operator"]
        arguments = execution["arguments"]
        try:
            if op == "^forward":
                OpStop()
                OpGo(0.5, 0.0, 0.0, 1.0, frame_id = "base_link") #Lidar-safe
                sleep(2.0)
            elif op == "^left":
                OpStop()
                #left()
                OpGo(0.0, 0.0, 0.5, 1.0, frame_id = "base_link")
                sleep(1.0)
            elif op == "^right":
                OpStop()
                #right()
                OpGo(0.0, 0.0, -0.5, 1.0, frame_id = "base_link")
                sleep(1.0)
            elif op == "^pick":
                OpStop()
                pick_with_feedback(None if len(arguments) == 0 else arguments)
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
                (x,y,z,w) = arguments.split("_")
                print("GOTO: " + str((x, y)))
                (xf, yf, zf, wf) = (float(x)-valueToTermOffset, float(y)-valueToTermOffset, float(z)-valueToTermOffset, float(w)-valueToTermOffset)
                OpGo(xf, yf, zf, wf)
            elif op == "^say":
                print("SAY: " + arguments)
        except:
            print("execution of wrong format " + str(execution))

def valueToTerm(x):
    return str(x+valueToTermOffset)[:5]

def TransbotPerceiveAt(obj, trans, rot):
    transXYrotZW = "_".join([valueToTerm(x) for x in trans[:2]]) + "_".join([valueToTerm(x) for x in rot[2:]])
    NAR.AddInput("<(%s * %s) --> at>. :|:" % (obj, transXYrotZW))

def TransbotPerceiveVisual(obj, screenX, screenY, trans, rot):
    TransbotPerceiveAt(obj, trans, rot)
    direction = "center" #640  -> 320 center
    if screenX < 320-centerSize: 
        direction = "left"
    elif screenX > 320+centerSize:
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
    #for i in range(50):
    #    TransbotExecute(NAR.AddInput("G! :|:")["executions"])
    #CELL2: tell NARS about locations of objects:
    #TransbotPerceiveAt("fridge", -0.25, 1.4)
    print("//transbot.py (ONA) go!")

def process(line):
    if line != "":
        if line == "*testmission":
            NAR.AddInput("<(<bottle --> [left]> &/ ^pick) =/> G>.")
            line = "G! :|:"
        if line.endswith("! :|:") or line == "*see":
            (trans, rot) = getLocation()
            action = cv.waitKey(10) & 0xFF
            detections, frame = detect_objects()
            for detection in detections:
                (obj, x, y, w, h, c) = detection
                x_real = x+w/2
                y_real = y+h
                TransbotPerceiveVisual(obj, x_real, y_real, trans, rot)
            print(detections)
            cv.imshow('frame', frame)
        if line.endswith("! :|:"):
            executions = NAR.AddInput(line)["executions"] #account for mental op
            executions += NAR.AddInput("10")["executions"]
            TransbotExecute(executions)
            executions = NAR.AddInput(line)["executions"]
            executions += NAR.AddInput("10")["executions"]
            TransbotExecute(executions)
        if line.endswith(".") or line.endswith(". :|:"):
            NAR.AddInput(line)
        elif line == "*pick_with_feedback":
            pick_with_feedback()
        elif line == "*left":
            left()
        elif line == "*right":
            right()
        elif line == "*forward":
            forward()
        elif line == "*backward":
            backward()
        elif line == "*arm_down":
            arm_down()
        elif line == "*arm_up":
            arm_up()
        elif line == "*pick":
            OpStop()
            pick()
        elif line == "*drop":
            OpStop()
            drop()
        elif line == "*reset":
            OpStop()
            init_pose()
            reset_ona()
        elif line == "*explore":
            NAR.AddInput("<(a &/ ^forward) =/> G>.")
            NAR.AddInput("a. :|:")
            for i in range(50):
                TransbotExecute(NAR.AddInput("G! :|:")["executions"])

def shell_step(lastLine = ""):
    #Get input line and forward potential command
    try:
        line = input().rstrip("\n") #"the green cat quickly eats the yellow mouse in the old house"
    except:
        exit(0)
    if len(line.strip()) == 0:
        line = lastLine;
    print("PROCESSED LINE: " + line)
    if line == "*loop": #endless sense-act cycle if desired
        line = "G! :|:"
        while True:
            process(line)
    if line.startswith("*steps "): #k steps
        steps = int(line.split("*steps ")[1])
        line = "G! :|:"
        for i in range(steps):
            process(line)
    process(line)
    return line

def transbot_shell():
    lastLine = ""
    while True:
        lastLine = shell_step(lastLine)

if __name__ == '__main__':
    reset_ona()
    print("//Welcome to ONA-Transbot shell!")
    transbot_shell()
        
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



