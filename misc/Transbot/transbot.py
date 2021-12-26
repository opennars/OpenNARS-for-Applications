#Before running this script, run basenodes.sh to make sure the relevant ros nodes are running
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

centerSize = 8
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
            mid = 375 #it's a bit to the right
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
                    forward(0.2)
                    forward(0.2)
                    success = close_gripper() #gripper feedback
                    if success:
                        print("//pick succeeded")
                        arm_up()
                        break
                    else:
                        print("//pick failed")
                        open_gripper()
                        arm_up()
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
    ActionInvoked = False
    for execution in executions:
        op = execution["operator"]
        arguments = execution["arguments"]
        ActionInvoked = True
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
            right()
            right()
            right()
            right()
        elif op == "^activate": #for later
            None
        elif op == "^deactivate": #for later
            None
        elif op == "^remember":
            locationQueryAnswer = NAR.AddInput("<(%s * ?where) --> at>? :|:" % arguments)["answers"][0]
            if locationQueryAnswer["term"] != "None":
                NAR.AddInput("<%s --> [localized]>. :|:" % arguments)
                NAR.AddInput("%s. :|:" % (locationQueryAnswer["term"]))
        elif op == "^goto":
            (x,y,z,w) = arguments.split("_")
            print("//GOTO: " + str((x, y, z, w)))
            (xf, yf, zf, wf) = (float(x)-valueToTermOffset, float(y)-valueToTermOffset, float(z)-valueToTermOffset, float(w)-valueToTermOffset)
            OpGo(xf, yf, zf, wf)
        elif op == "^say":
            print("//SAY: " + arguments)
    return ActionInvoked

def valueToTerm(x):
    return str(x+valueToTermOffset)[:5]

def TransbotPerceiveAt(obj, trans, rot):
    transXYrotZW = "_".join([valueToTerm(x) for x in trans[:2]]) + "_" + "_".join([valueToTerm(x) for x in rot[2:]])
    NAR.AddInput("<(%s * %s) --> at>. :|:" % (obj, transXYrotZW))

def TransbotPerceiveVisual(obj, screenX, screenY, trans, rot):
    direction = "center" #640  -> 320 center
    TransbotPerceiveAt(obj, trans, rot) #TODO improve
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
    NAR.AddInput("*motorbabbling=false")
    NAR.AddInput("*volume=0")
    #you need to ask the map in order to localize an object
    NAR.AddInput("<(<gripper --> [#sth]> &/ <({SELF} * $obj) --> ^remember>) =/> <$obj --> [localized]>>.")
    #once it's localized, and the location at the map is known, go to the location in order to see the object
    NAR.AddInput("<((<$obj --> [localized]> &/ <($obj * #location) --> at>) &/ <({SELF} * #location) --> ^goto>) =/> <$obj --> [see]>>.")
    #CELL2: tell NARS about locations of objects:
    #NAR.AddInput("<fridge * 500.0_500.0_500.0_500.0) --> at>. :|:"
    print("//transbot.py (ONA) go!")

def process(line):
    if line != "":
        if line.endswith("! :|:") or line == "*internal":
            if picked:
                NAR.AddInput("<gripper --> [closed]>. :|:")
            else:
                NAR.AddInput("<gripper --> [open]>. :|:")
        if line.endswith("! :|:") or line == "*see":
            (trans, rot) = getLocation()
            action = cv.waitKey(10) & 0xFF
            detections, frame = detect_objects()
            (obj_temp, x_real_temp, y_real_temp, w_temp, h_temp, c_temp) = ("", -1, -1, -1, -1, 0)
            for detection in detections:
                (obj, x, y, w, h, c) = detection
                x_real = x+w/2
                y_real = y+h #down side of bb
                if y_real > y_real_temp:
                    (obj_temp, x_real_temp, y_real_temp, w_temp, h_temp, c_temp) = (obj, x_real, y_real, w, h, c)
            if y_real_temp != -1:
                TransbotPerceiveVisual(obj, x_real_temp, y_real_temp, trans, rot)
            cv.imshow('frame', frame)
        if line.endswith("! :|:"):
            executions = NAR.AddInput(line)["executions"] #account for mental op
            for i in range(10): #time limit to act
                executions += NAR.AddInput("1")["executions"]
                ActionInvoked = TransbotExecute(executions)
                if ActionInvoked:
                    break #acted, done
                executions = []
        if line.endswith(".") or line.endswith(". :|:") or line.endswith("?") or line.endswith("? :|:"):
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
        elif line.isdigit() or line.startswith("*volume") or line.startswith("*motorbabbling") or line.endswith("}"):
            NAR.AddInput(line)

lastGoal = "G! :|:"
def shell_step(lastLine = ""):
    global lastGoal
    #Get input line and forward potential command
    try:
        line = input().rstrip("\n") #"the green cat quickly eats the yellow mouse in the old house"
    except:
        exit(0)
    if len(line.strip()) == 0:
        line = lastLine;
    print("//PROCESSED LINE: " + line)
    if line.endswith("! :|:"):
        lastGoal = line
    if line == "*loop": #endless sense-act cycle if desired
        while True:
            process(lastGoal)
    if line == "*testmission":
        NAR.AddInput("<((<gripper --> [open]> &/ <bottle --> [left]>) &/ ^pick) =/> <gripper --> [closed]>>.")
        NAR.AddInput("<(<gripper --> [closed]> &/ ^drop) =/> G>.")
        line = "*steps 2"
    if line.startswith("*steps "): #k steps
        steps = int(line.split("*steps ")[1])
        for i in range(steps):
            process(lastGoal)
        print("//*steps DONE")
    process(line)
    return line

def transbot_shell():
    lastLine = "G! :|:"
    while True:
        lastLine = shell_step(lastLine)

if __name__ == '__main__':
    reset_ona()
    print("//Welcome to ONA-Transbot shell!")
    transbot_shell()


