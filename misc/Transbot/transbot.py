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
from Nalifier import *
import time
import json

#Parameters:
center_offset = 20
y_too_far_to_grab = 340
robotVisualMiddle = 375 #middle of the robot
checkpointdecisions = 1

def pick_failed():
    arm_up()
    backward()
    backward()
    right()
    right()
    right()
    open_gripper()

def pick_with_feedback(pickobj=None, location=None, ForwardSleep=0.5):
    if " * " in pickobj:
        pickobj = pickobj.replace("(","").replace(")","")
        location = pickobj.split(" * ")[0]
        pickobj = pickobj.split(" * ")[1]
    if getPicked():
        return
    arm_down()
    sleep(1)
    max_ops = 30
    move_steps = 0
    max_swaps = 3
    max_ops = 30
    swaps = 0 #left/right focus attempts
    swap_Left = False
    swap_Right = False
    while True:
        sys.stdout.flush()
        move_steps+=1
        if move_steps > max_ops:
            open_gripper()
            pick_failed()
            break
        action = cv.waitKey(10) & 0xFF
        detections, frame = detect_objects()
        y_real_temp = -1
        x_real_temp = -1 if location != "left" else 9999999
        for i, detection in enumerate(detections):
            if i >= 2:
                continue
            (obj, x, y, w, h, c, color) = detection
            x_real = x+w/2
            y_real = y+h #down side of bb
            if pickobj == None or pickobj == obj:
                if location is None and y_real > y_real_temp:
                    y_real_temp = y_real
                    x_real_temp = x_real
                if location == "left" and x_real < x_real_temp:
                    y_real_temp = y_real
                    x_real_temp = x_real
                if location == "right" and x_real > x_real_temp:
                    y_real_temp = y_real
                    x_real_temp = x_real
        if y_real_temp != -1:
            if y_real_temp < y_too_far_to_grab:
                print("//pick failed, too far away to grab")
                pick_failed()
                break
            kp = 50
            turn_steps_required = int(abs(x_real_temp - robotVisualMiddle) / kp)
            if (x_real_temp >= robotVisualMiddle-center_offset and x_real_temp <= robotVisualMiddle+center_offset) or swaps > 3: # Object in the center case
                print("//CENTER------------")
                closer_to_gripper = 475
                swaps = 0
                if y_real_temp < closer_to_gripper: #visual feedback
                    forward()
                elif y_real_temp > closer_to_gripper:
                    #left()
                    forward(linear=0.25)
                    if ForwardSleep is not None:
                        time.sleep(ForwardSleep)
                    forward(linear=0.25)
                    if ForwardSleep is not None:
                        time.sleep(ForwardSleep)
                    forward(linear=0.2)
                    if ForwardSleep is not None:
                        time.sleep(ForwardSleep)
                    forward(linear=0.2)
                    success, grip_angle = close_gripper() # Grabbing object
                    if success:
                        arm_up()
                        success2, _ = close_gripper(grip_angle) # Check if gripper is still holds an object after lifting
                        if success2:
                            setPicked(True)
                            print("pick succeeded")
                            if get_hastrailer():
                                drop_trailer()
                        else:
                            open_gripper()
                            print("//pick failed, object slipped")
                            backward()
                            backward()
                            right()
                            right()
                            right()
                        break
                    else:
                        print("//pick failed, sensed nothing to grab")
                        pick_failed()
                        arm_up()
                        break
            elif x_real_temp > robotVisualMiddle+center_offset: # Object is on the RIGHT of the gripper
                print("//RIGHT<<<<<<<<<<<<<<<<")
                turn_steps_required = max(1, turn_steps_required)
                for i in range(abs(turn_steps_required)): right()
                swap_Right = True
            else: # Object is on the LEFT of the gripper
                print("//LEFT>>>>>>>>>>>>>>>")
                turn_steps_required = max(1, turn_steps_required)
                for i in range(abs(turn_steps_required)): left()
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
            forward()
            forward()
            OpStop()
        elif op == "^left":
            OpStop()
            left()
            left()
            left()
            left()
            #OpGo(0.0, 0.0, 0.5, 1.0, frame_id = "base_link")
            OpStop()
        elif op == "^right":
            OpStop()
            right()
            right()
            right()
            right()
            #OpGo(0.0, 0.0, -0.5, 1.0, frame_id = "base_link")
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
    direction = "front" #640  -> 320 center
    TransbotPerceiveAt(obj, trans, rot) #TODO improve
    if screenX < robotVisualMiddle-center_offset:
        direction = "left"
    elif screenX > robotVisualMiddle+center_offset:
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
    ActionInvoked = False
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
            (obj_temp, x_real_temp, y_real_temp, w_temp, h_temp, c_temp, color_temp) = ("", -1, -1, -1, -1, 0, (0,0,0))
            detections.sort(key=lambda d: -(d[2]+d[4]))
            if len(detections) > 0:
                (obj, x, y, w, h, c, color) = detections[0] #nearest
                x_real = x+w/2
                y_real = y+h #down side of bb
                (obj_temp, x_real_temp, y_real_temp, w_temp, h_temp, c_temp, color_temp) = (obj, x_real, y_real, w, h, c, color)
            if y_real_temp != -1 and y_real_temp >= y_too_far_to_grab:
                if len(detections) >=2 and detections[0][0] == detections[1][0]: #same object, compare using Nalifier!
                    detections_xsorted = [detections[0], detections[1]]
                    detections_xsorted.sort(key=lambda x: x[1])
                    color_insensitivity = 2.0
                    size_insensitivity = 1.0
                    maxSizeX = max(detections[0][3], detections[1][3])
                    maxSizeY = max(detections[0][4], detections[1][4])
                    extractVectors = lambda detection: ([detection[6][0]/(255.0 * color_insensitivity), detection[6][1]/(255.0 * color_insensitivity), detection[6][2]/(255.0 * color_insensitivity)],
                                                        [detection[3]/(maxSizeX * size_insensitivity), detection[4]/(maxSizeY*size_insensitivity)]) #color and size thus far
                    dleft = detections_xsorted[0]
                    dright = detections_xsorted[1]
                    color_left, size_left = extractVectors(dleft)
                    color_right, size_right = extractVectors(dright)
                    nalifier = Nalifier(1)
                    nalifier.AddInputVector("left", color_left, dimname="color", UseHistogram=False)
                    nalifier.AddInputVector("left", size_left, dimname="size", UseHistogram=False)
                    nalifier.AddInput("1", Print=False)
                    nalifier.InstanceCreation = False
                    nalifier.AddInputVector("right", color_right, dimname="color", UseHistogram=False)
                    nalifier.AddInputVector("right", size_right, dimname="size", UseHistogram=False)
                    nalifier.SUFFICIENT_MATCH_EXP = 0.0 #find nearest node
                    nalifier.AddInput("1", Print=True)
                    #print("//color_left:", color_left, "color_right:", color_right)
                    #print("//size_left:", size_left, "size_right:", size_right)
                    biggestDifference = nalifier.BiggestDifference
                    if biggestDifference[0] == "+":
                        statement1 = f"<({dright[0]} * [left]) --> (+ {biggestDifference[1]})>"
                        statement2 = f"<([right] * {dleft[0]}) --> (+ {biggestDifference[1]})>"
                        NAR.AddInput(f"({statement1} && {statement2}). :|:")
                    else:
                        statement1 = f"<([left] * {dright[0]}) --> (+ {biggestDifference[1]})>"
                        statement2 = f"<({dleft[0]} * [right]) --> (+ {biggestDifference[1]})>"
                        NAR.AddInput(f"({statement1} && {statement2}). :|:")
                else:
                    for detection in detections[0:2]:
                        (obj, x, y, w, h, c, color) = detection
                        x_real = x+w/2
                        y_real = y+h #down side of bb
                        TransbotPerceiveVisual(obj, x_real, y_real, trans, rot)
            if y_real_temp == -1 or y_real_temp < y_too_far_to_grab or x_real_temp > robotVisualMiddle-center_offset or collision != "free": #right side blocked by arm
                NAR.AddInput("<obstacle --> [" + collision + "]>. :|:")
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
        elif line == "*droptrailer":
            drop_trailer(force=True)
        elif line == "*reset":
            setPicked(False)
            OpStop()
            init_pose()
            reset_ona()
        elif line.isdigit() or line.startswith("*volume") or line.startswith("*motorbabbling") or line.endswith("}"):
            NAR.AddInput(line)
        return ActionInvoked

lastGoal = "G! :|:"
points = []
def shell_step(lastLine = ""):
    global lastGoal, points
    #Get input line and forward potential command
    try:
        line = input().rstrip("\n").replace("leave","Pleft") #"the green cat quickly eats the yellow mouse in the old house"
    except:
        exit(0)
    if len(line.strip()) == 0:
        line = lastLine;
    print("//PROCESSED LINE: " + line)
    if line.endswith("! :|:"):
        lastGoal = line
    if line.startswith("*clearpoints"):
        points = []
        return line
    if line.startswith("*point"): #define checkpoint
        P = getLocation()
        points.append(P)
        return line
    if line.startswith("*patrol "): #how often to patrol the points that have been defined
        repetitions = int(line.split("*patrol ")[1])
        for i in range(repetitions):
            for j, (trans, rot) in enumerate(points):
                OpGo(trans[0], trans[1], rot[2], rot[3])
                TransbotPerceiveAt("{P" + str(j) + "}", trans, rot)
                TransbotPerceiveAt("{SELF}", trans, rot)
                for i in range(checkpointdecisions):
                    if process(lastGoal):
                        break
        return line
    if line.startswith("*savepoints"):
        with open("points.json", "w") as json_file:
            json.dump(points, json_file)
        return line
    if line.startswith("*loadpoints"):
        with open("points.json") as json_file:
            points = json.load(json_file)
        return line
    if line.startswith("*hastrailer "):
        set_hastrailer(line.split("*hastrailer ")[1] in ["true", "True"])
        return line
    if line == "*loop": #endless sense-act cycle if desired
        while True:
            process(lastGoal)
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


