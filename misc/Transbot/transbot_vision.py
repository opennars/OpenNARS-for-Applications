import time
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from threading import Lock
from time import sleep
import sys
import os
import pycuda.autoinit
from Transbot_Lib import Transbot

bot = Transbot()
path = os.getcwd()
sys.path.append("/home/jetson/tensorrt_demos/")
os.chdir("/home/jetson/tensorrt_demos/")
from utils.yolo_with_plugins import TrtYOLO
from utils.yolo_classes import COCO_CLASSES_LIST
yolo = TrtYOLO("yolov4-416")
os.chdir(path)
COLORS = np.random.uniform(0, 255, size=(len(COCO_CLASSES_LIST), 3))
transbot_vision_WIDTH = 640
transbot_vision_HEIGHT = 480
warn_person_car = True
warn_distance = 300 #upside down

framelock = Lock()
frame = ""
def rgbtopic(msg):
    global frame, framelock
    if not isinstance(msg, CompressedImage):
        return
    framelock.acquire()
    np_arr = np.fromstring(msg.data, np.uint8)
    frame = cv.imdecode(np_arr, cv.IMREAD_ANYCOLOR)
    frame = cv.resize(frame, (transbot_vision_WIDTH, transbot_vision_HEIGHT))
    framelock.release()
    
#depthframelock = Lock()
#depthframe = ""
#def depthtopic(msg):
#    global depthframe, depthframelock
#    if not isinstance(msg, CompressedImage):
#        return
#    depthframelock.acquire()
#    np_arr = np.fromstring(msg.data, np.uint8)
#    depthframe = cv.imdecode(np_arr, cv.IMREAD_ANYCOLOR) #IMREAD_GRAYSCALE
#    depthframe = cv.resize(depthframe, (640, 480))
#    depthframelock.release()

sub = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, rgbtopic)
#sub2 = rospy.Subscriber("/camera/depth/image_raw/compressed", CompressedImage, depthtopic)

def cropImage(img, BB): #crop image according to boundinx box
    (x,y,w,h) = BB
    crop = img[y:y+h, x:x+w]
    return crop

def dominantColorsWithPixelCounts(img):
    im_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    pixels = np.float32(im_rgb.reshape(-1, 3))
    n_color_clusters = 5
    _, labels, palette = cv.kmeans(pixels, n_color_clusters, None, (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 200, .1), 10, cv.KMEANS_RANDOM_CENTERS)
    _, counts = np.unique(labels, return_counts=True)
    DominantColorsWithCounts = list(zip(palette, counts))
    DominantColorsWithCounts.sort(key=lambda x: -x[1]) #not necessary
    return DominantColorsWithCounts

def applyYOLO(img):
    detection_confidence_threshold = 0.3
    boxes, confs, clss = yolo.detect(img, detection_confidence_threshold)
    detections = []
    for i in range(len(clss)):
        class_id = int(clss[i])
        box = boxes[i]
        class_name = COCO_CLASSES_LIST[class_id]
        if warn_person_car and class_name in ["person", "car"]:
            y = box[1] + (box[3] - box[1])
            if y >= warn_distance:
                bot.set_beep(1000)
        imagecropped = cropImage(img, (box[0], box[1], box[2]-box[0], box[3]-box[1]))
        dominantColor = dominantColorsWithPixelCounts(imagecropped)[0][0]
        detections.append([class_name, box[0], box[1], box[2]-box[0], box[3]-box[1], confs[i], (dominantColor[0], dominantColor[1], dominantColor[2])])
        color = COLORS[class_id]
        cv.rectangle(img, (box[0], box[1]), (box[2], box[3]), color, thickness=2)
        cv.putText(img, class_name +":"+str(box[0]) + "," + str(box[1]), (box[0], box[1] - 5), cv.FONT_HERSHEY_SIMPLEX, 1, color, 2)
    detections.sort(key = lambda x: -(x[2] + x[4])) #sort by distance (Y as proxy for distance for now since camera is tilted slightly towards floor)
    return img, detections

def detect_objects():
    global frame, framelock
    framecopy = None
    framelock.acquire()
    framecopy = frame.copy()
    framelock.release()
    #depthframecopy = None
    #depthframelock.acquire()
    #depthframecopy = depthframe.copy()
    #depthframelock.release()
    if frame != "":
        start = time.time()
        (framecopy, detections) = applyYOLO(framecopy)
        end = time.time()
        fps = 1 / (end - start)
        text = "FPS : " + str(int(fps))
        cv.putText(framecopy, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (100, 200, 200), 1)
        cv.imshow('frame', framecopy)
        #cv.imshow('depthframe', depthframecopy)
        return detections, framecopy
    return None, None

#1. Wait for frame:
while True:
    framelock.acquire()
    if frame != "":
        framelock.release()
        break
    framelock.release()
    sleep(0.1)
print("//transbot_vision.py go!")

if __name__ == '__main__':
    rospy.init_node('NARTest')
    #2. Debug GUI:
    while True:
        action = cv.waitKey(10) & 0xFF
        detections, frame = detect_objects()
        if detections != None:
            print(detections)
        sleep(0.1)
