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

import sys
import subprocess
import os
import os.path

ImgSizeX, ImgSizeY = (288, 352)
DarknetFolder = "/home/tc/Dateien/Visionchannel/AlexeyAB_darknet/darknet/"

InputPass = 1 #How many Stdin lines will be passed on to Stdout for each line produced by this sensory channel
for arg in sys.argv:
    if arg.startswith("InputPass="):
        InputPass = int(arg.split("InputPass=")[1])

def BoundingBoxFromBBStr(BBStr):
    M = {}
    BBlist = ' '.join(BBStr.strip().replace('(','').replace(')','').replace(':','').split()).split(' ')
    for i in range(0,len(BBlist),2):
        label = BBlist[i]
        value = float(BBlist[i+1])
        M[label] = value
    return M
    
def LocationFromDetection(BB):
    Location = {}
    Location['x'] = BB["left_x"]+BB["width"]/2
    Location['y'] = BB["top_y"]+BB["height"]/2
    return Location
    
def DiscretizeValue(Value, MaxValue):
    if Value < MaxValue/3.0:
        return "smaller"
    if Value < MaxValue*(2.0/3.0):
        return "equal"
    return "larger"
    
def DiscretizedLocationFromLocation(Location):
    DiscretizedLocation = {}
    DiscretizedLocation['x'] = DiscretizeValue(Location['x'], ImgSizeX)
    DiscretizedLocation['y'] = DiscretizeValue(Location['y'], ImgSizeY)
    return DiscretizedLocation
    
EncodeAxes = ['x'] #'x', 'y'
def Narsesefy(label, percent, DiscretizedLocation):
    Loc = ""
    for axis in EncodeAxes:
        Loc = Loc + " " + DiscretizedLocation[axis] + axis.upper()
    return "<" + label + " --> [" + Loc.strip() + "]>. :|: {1.0 " + str(min(99.0,percent)/100.0) + "}"

def LineToNarsese(line):
    labelPercent_BB = line.split("%")
    label = labelPercent_BB[0].split(":")[0]
    percent = float(labelPercent_BB[0].split(":")[1].strip())
    DiscretizedLocation = DiscretizedLocationFromLocation(LocationFromDetection(BoundingBoxFromBBStr(labelPercent_BB[1])))
    print(Narsesefy(label, percent, DiscretizedLocation))

BaseFolder = os.getcwd()
while True:
    os.chdir(DarknetFolder)
    if os.path.isfile("./frame.jpg"):
        os.remove("./frame.jpg")
    os.system("ffmpeg -i http://192.168.0.185:8080/video -ss 0:0:1 -frames 1 ./frame.jpg")
    output = subprocess.check_output("./darknet detector test ./cfg/coco.data ./cfg/yolov4-tiny.cfg yolov4-tiny.weights -thresh 0.25 -ext_output ./frame.jpg", shell=True, stderr=subprocess.STDOUT)
    os.chdir(BaseFolder)
    for line in output.decode("utf-8").split('\n'):
        if "%" in line: #
            LineToNarsese(line)
            #also for each line pass on a line of stdin to allow for multiple input streams:
            for i in range(InputPass):
                s = sys.stdin.readline().strip()
                if s != "":
                    print(s)
    sys.stdout.flush()
    if len(sys.argv) > 1 and sys.argv[1] == "once":
        break
