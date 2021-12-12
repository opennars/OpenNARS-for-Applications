import time
import cv2 as cv
import numpy as np

#detection path
detectpath = "/home/jetson/transbot_ws/src/transbot_visual/detection/"
# load the COCO class names
with open(detectpath + 'object_detection_coco.txt', 'r') as f: class_names = f.read().split('\n')
# get a different color array for each of the classes
COLORS = np.random.uniform(0, 255, size=(len(class_names), 3))
# load the DNN modelimage
model = cv.dnn.readNet(model = detectpath + 'frozen_inference_graph.pb', config = detectpath + 'ssd_mobilenet_v2_coco.txt', framework = 'TensorFlow')

def Target_Detection(image):
    detections = []
    image_height, image_width, _ = image.shape
    # create blob from image
    blob = cv.dnn.blobFromImage(image=image, size=(300, 300), mean=(104, 117, 123), swapRB=True)
    model.setInput(blob)
    output = model.forward()
    # loop over each of the detections
    for detection in output[0, 0, :, :]:
        # extract the confidence of the detection
        confidence = detection[2]
        # draw bounding boxes only if the detection confidence is above...
        # ... a certain threshold, else skip
        if confidence > .4:
            # get the class id
            class_id = detection[1]
            # map the class id to the class
            class_name = class_names[int(class_id) - 1]
            color = COLORS[int(class_id)]
            # get the bounding box coordinates
            box_x = detection[3] * image_width
            box_y = detection[4] * image_height
            # get the bounding box width and height
            box_width = detection[5] * image_width
            box_height = detection[6] * image_height
            ret += [(class_name, box_x, box_y, box_width, box_height)]
            # draw a rectangle around each detected object
            cv.rectangle(image, (int(box_x), int(box_y)), (int(box_width), int(box_height)), color, thickness=2)
            # put the class name text on the detected object
            cv.putText(image, class_name, (int(box_x), int(box_y - 5)), cv.FONT_HERSHEY_SIMPLEX, 1, color, 2)
    return (image, detections)

capture = cv.VideoCapture(0)
cv_edition = cv.__version__
if cv_edition[0] == '3': capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
else: capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

def get_frame():
    if capture.isOpened():
        start = time.time()
        ret, frame = capture.read()
        (frame, detections) = Target_Detection(frame)
        end = time.time()
        fps = 1 / (end - start)
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (100, 200, 200), 1)
        cv.imshow('frame', frame)
        return detections
    return []
