import cv2
import mediapipe as mp
from djitellopy import Tello
import numpy as np


mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

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
    myDrone.streamon()
    return myDrone

def telloGetFrame(myDrone, w, h):
    myFrame = myDrone.get_frame_read()
    myFrame = myFrame.frame
    img = cv2.resize(myFrame, (w, h))
    return img

pid = [0.3, 0.5, 0]
fbRange = [6200, 6800]

if __name__=="__main__":
    #myDrone = intializeTello()
    cap = cv2.VideoCapture(0)
    w = 640
    h = 480
    pError = 0
    #myDrone.takeoff()
