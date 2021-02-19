#importing packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import cv2.aruco as aruco
import numpy as np

import smbus
import board
import busio

# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# This is the address we setup in the Arduino Program
address = 0x04

def writeNumber(value):
    #bus.write_byte(address, value)
    if value == None:
        return -2
    try:
        bus.write_byte_data(address, 0, value)
    except:
        return -1
    
    return 0

#continuously takes pictures and checks for aruco markers
def FindQuadrant(corners, ids, WIDTH, HEIGHT):
    #Check for Quadrent
    if ids != None:
        for marker in range(0,ids.size):
            mark = np.array(corners[marker][0])

            #x and y coordinate of center of marker
            x = ((mark[1][0] - mark[0][0])/2) + mark[0][0]
            y = ((mark[3][1] - mark[0][1])/2) + mark[0][1]
            #determining the quadrant
            if(x < (WIDTH/2) and y < (HEIGHT/2)):
                #upper left quadrant
                return 0
            elif(x >= (WIDTH/2) and y < (HEIGHT/2)):
                #upper right quadrant
                return 1
            elif(x >= (WIDTH/2) and y >= (HEIGHT/2)):
                #lower right quadrant
                return 2
            else:
                #lower left quadrant
                return 3

#MAIN PROGRAM
#initializing camera
camera = PiCamera()
rawCapture = PiRGBArray(camera)
time.sleep(0.1)
camera.capture(rawCapture, format="bgr")

#Setting up camera
HEIGHT = 768
WIDTH = 1024
quadrant = None
camera.resolution = (WIDTH, HEIGHT)
rawCapture.truncate(0)

#Creating aruco dictionary and parameters
arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
arucoParam = aruco.DetectorParameters_create()
    
loop = True
#taking pictures and checking for markers
while loop:
    try:
        camera.capture(rawCapture,format='bgr')
        img = rawCapture.array
            
        #converting to grayscale
        grayImg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    
        #detecting markers
        (corners, ids, rejected) = aruco.detectMarkers(grayImg, arucoDict, parameters=arucoParam)

        #Find and return quadrant
        quadrant = FindQuadrant(corners, ids, WIDTH, HEIGHT)

        print(quadrant)

        print(writeNumber(quadrant))

        rawCapture.truncate(0)
    #exits loop when ctl+c is pressed
    except KeyboardInterrupt:
        loop = False
