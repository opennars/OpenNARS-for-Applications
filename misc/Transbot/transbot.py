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
from transbot_lidar import *

#Parameters:
motorsleep = 5 if sys.argv[0] != "transbot_simulation.py" else 0
gotosleep = 20 if sys.argv[0] != "transbot_simulation.py" else 0
cyclesleep = 5 if sys.argv[0] != "transbot_simulation.py" else 0
centerSize = 10
y_too_far_to_grab = 340
robotVisualMiddle = 375 #middle of the robot

def pick_failed():
    arm_up()
    backward()
    backward()
    backward()
    right()
    right()
    open_gripper()

def pick_with_feedback(pickobj=None):
    if getPicked():
        return
    arm_down()
    sleep(1)
    max_ops = 30
    ops = 0
    max_swaps = 3
    max_ops = 30
    swaps = 0 #left/right focus attempts
    swap_Left = False
    swap_Right = False
    while True:
        sys.stdout.flush()
        ops+=1
        if ops > max_ops:
            open_gripper()
            pick_failed()
            break
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
            if y_real_temp < y_too_far_to_grab:
                print("//pick failed, too far away to grab")
                pick_failed()
                break
            if (x_real_temp >= robotVisualMiddle-centerSize and x_real_temp <= robotVisualMiddle+centerSize) or swaps > 3:
                print("//CENTER------------")
                closer_to_gripper = 475
                swaps = 0
                if y_real_temp < closer_to_gripper: #visual feedback
                    forward()
                elif y_real_temp > closer_to_gripper:
                    left()
                    forward()
                    forward()
                    forward()
                    forward()
                    success = close_gripper() #gripper feedback
                    if success:
                        print("//pick succeeded")
                        setPicked(True)
                        arm_up()
                        break
                    else:
                        print("//pick failed, sensed nothing to grab")
                        pick_failed()
                        arm_up()
                        break
            elif x_real_temp > robotVisualMiddle+centerSize:
                print("//RIGHT<<<<<<<<<<<<<<<<")
                right()
                swap_Right = True
            elif x_real_temp < robotVisualMiddle-centerSize:
                print("//LEFT>>>>>>>>>>>>>>>")
                left()
                swap_Left = True
            if swap_Left and swap_Right:
                swap_Left = False
                swap_Right = False
                swaps += 1
        else:
            print("//pick failed, object disappeared visually")
            pick_failed()
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
            #OpGo(0.5, 0.0, 0.0, 1.0, frame_id = "base_link") #Lidar-safe
            forward()
            forward()
            forward()
            #sleep(motorsleep)
            OpStop()
        elif op == "^left":
            OpStop()
            left()
            left()
            #OpGo(0.0, 0.0, 0.5, 1.0, frame_id = "base_link")
            #sleep(motorsleep)
            OpStop()
        elif op == "^right":
            OpStop()
            right()
            right()
            #OpGo(0.0, 0.0, -0.5, 1.0, frame_id = "base_link")
            #sleep(motorsleep)
            OpStop()
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
        elif op == "^goto":
            locationQueryAnswer = NAR.AddInput("<(%s * ?where) --> at>? :|:" % arguments)["answers"][0]
            if locationQueryAnswer["term"] != "None":
                (x,y,z,w) = locationQueryAnswer["term"].split(" * ")[1].split(") --> at>")[0].split("_")
                (xf, yf, zf, wf) = (float(x)-valueToTermOffset, float(y)-valueToTermOffset, float(z)-valueToTermOffset, float(w)-valueToTermOffset)
                OpGo(xf, yf, zf, wf)
                for i in range(gotosleep):
                    sleep(1)
                    print("//^goto wait %d / %d" % (i+1, gotosleep))
                    sys.stdout.flush()
                OpStop()
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
    if screenX < robotVisualMiddle-centerSize:
        direction = "left"
    elif screenX > robotVisualMiddle+centerSize:
        direction = "right"
    NAR.AddInput("<%s --> [%s]>. :|:" % (obj, direction))

Configuration = """
*reset
*setopname 1 ^forward
*setopname 2 ^left
*setopname 3 ^right
*babblingops=3
*setopname 4 ^pick
*setopname 5 ^drop
*setopname 6 ^activate
*setopname 7 ^deactivate
*setopname 8 ^say
*setopname 9 ^goto
"""
def reset_ona():
    with open("knowledge.nal", 'r') as f:
        BackgroundKnowledge = f.read()
    for bg in (Configuration + BackgroundKnowledge).split("\n"):
        bgstr = bg.strip()
        if len(bgstr) > 0:
            NAR.AddInput(bgstr)
    print("//transbot.py (ONA) go!")

def process(line):
    if line != "":
        if line.endswith("? :|:") and "{SELF}" in line:
            (trans, rot) = getLocation()
            TransbotPerceiveAt("{SELF}", trans, rot)
        if line.endswith("! :|:") or line == "*internal":
            if getPicked():
                NAR.AddInput("<gripper --> [hold]>. :|:")
            else:
                NAR.AddInput("<gripper --> [open]>. :|:")
        if line.endswith("! :|:") or line == "*see":
            collision = getCollision()
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
            if y_real_temp == -1 or y_real_temp < y_too_far_to_grab or x_real_temp > robotVisualMiddle-centerSize or collision != "free": #right side blocked by arm
                NAR.AddInput("<obstacle --> [" + collision + "]>. :|:")
            elif y_real_temp != -1 and y_real_temp >= y_too_far_to_grab:
                TransbotPerceiveVisual(obj, x_real_temp, y_real_temp, trans, rot)
            cv.imshow('frame', frame)
        if line.endswith("! :|:"):
            executions = NAR.AddInput(line)["executions"] #account for mental op
            reasoningtime = 10
            for i in range(reasoningtime): #time limit to act
                executions += NAR.AddInput("1")["executions"]
                ActionInvoked = TransbotExecute(executions)
                if ActionInvoked:
                    NAR.AddInput(str(reasoningtime-(i+1))) #still same think time even when already reacted
                    break #acted, done
                executions = []
        if line.endswith(".") or line.endswith(". :|:") or line.endswith("?") or line.endswith("? :|:"):
            NAR.AddInput(line)
        elif line == "*pick_with_feedback":
            pick_with_feedback()
        elif line == "*left":
            left(angular=0.6)
        elif line == "*right":
            right(angular=0.6)
        elif line == "*forward":
            forward(linear=0.6)
        elif line == "*backward":
            backward(linear=0.6)
        elif line == "*arm_down":
            arm_down()
        elif line == "*arm_up":
            arm_up()
        elif line == "*pick":
            OpStop()
            pick(force=True)
        elif line == "*drop":
            OpStop()
            drop(force=True)
        elif line == "*reset":
            setPicked(False)
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
        line = input().rstrip("\n").replace("leave","left") #"the green cat quickly eats the yellow mouse in the old house"
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
            sleep(cyclesleep)
    if line.startswith("*steps "): #k steps
        steps = int(line.split("*steps ")[1])
        for i in range(steps):
            process(lastGoal)
            sleep(cyclesleep)
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



