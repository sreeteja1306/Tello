from djitellopy import Tello
import mediapipe as mp
import time
import cv2
import numpy as np

mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

# drone = Tello()
# drone.connect()
# print(drone.get_battery())

def intializeTello():
    myDrone = Tello()
    myDrone.connect()
    myDrone.for_back_velocity = 0
    myDrone.left_right_velocity = 0
    myDrone.up_down_velocity = 0
    myDrone.yaw_velocity = 0
    myDrone.speed = 0
    print(myDrone.get_battery())
    myDrone.streamoff()
    time.sleep(1)
    myDrone.streamon()
    time.sleep(1)
    return myDrone

def findFace(results):
    myFacesListC = []
    myFaceListArea = []
    if results.detections:
        for detection in results.detections:
            print(detection)
            # mp_drawing.draw_detection(image, detection)
            location_data = detection.location_data
            if location_data.format == location_data.RELATIVE_BOUNDING_BOX:
                bb = location_data.relative_bounding_box
                bb_box = [
                    bb.xmin, bb.ymin,
                    bb.width, bb.height,
                ]

                x1, y1 = int((bb_box[0]) * w), int((bb_box[1]) * h)
                x2, y2 = int((bb_box[0] + bb_box[2]) * w), int((bb_box[1] + bb_box[3]) * h)

                print(x1, y1, x2, y2)

                cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)

                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                area = bb_box[2] * bb_box[3]
                myFacesListC.append([cx, cy])
                myFaceListArea.append(area)

    if len(myFaceListArea) != 0:
        i = myFaceListArea.index(max(myFaceListArea))
        return [myFacesListC[i], myFaceListArea[i]]
    else:
        return [[0, 0], 0]

pid = [0.3, 0.5, 0]
fbRange = [0.025, 0.04]


def trackFace(tello, center, area, w, pid, pError):
    fb = 0
    x,y = center[0], center[1]
    error = x - w/2
    yaw = pid[0] * error + pid[1] * (error - pError)
    yaw = int(np.clip(yaw, -100,100))
    #print(area)
    if area > fbRange[0] and area < fbRange[1]:
        fb = 0
    elif area > fbRange[1]:
        fb = -10
    elif area < fbRange[0] and area != 0:
        fb = 10
    if x == 0:
        yaw = 0
        error = 0
    #print(area,fb)
    tello.send_rc_control(0, fb, 0, yaw)
    return error

def telloGetFrame(myDrone,w,h):
    myFrame = myDrone.get_frame_read()
    myFrame = myFrame.frame
    img = cv2.resize(myFrame, (w, h))
    return img

#myDrone = intializeTello()
#cap = cv2.VideoCapture(0)
w=640
h=480
pError = 0
#myDrone.takeoff()
time.sleep(1)
#myDrone.send_rc_control(0, 0, 22, 0)
#time.sleep(1)

vidcap = cv2.VideoCapture("C:\\Users\\DST\\Desktop\\Autonomous_Tello_Drone-master\\pose_controlled_tello\\my_poses\\test7.mp4")
success,image = vidcap.read()
while success:
    with mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.6) as face_detection:
        #image = telloGetFrame(myDrone, w, h)
        #ok, image = cap.read()
        success, image = vidcap.read()
        h, w, c = image.shape
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = face_detection.process(image)
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        info=findFace(results)
        #print(info[0], info[1])
        #pError = trackFace(myDrone, info[0], info[1], w, pid, pError)
        if info[1] > 0.06:
            # continue
            print(info[1])
            cv2.putText(image, 'WARNING!!!', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            #myDrone.send_rc_control(0, -30, 0, 0)
            # cv2.putText(image, 'Back', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
        else:
            print(info[0], info[1])
            #pError = trackFace(myDrone, info[0], info[1], w, pid, pError)
        cv2.imshow('MediaPipe Face Detection', image)
        if cv2.waitKey(5) & 0xFF == 27:
            cv2.destroyAllWindows()
            #myDrone.land()
            break

