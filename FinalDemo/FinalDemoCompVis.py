Python 3.8.3 (v3.8.3:6f8c8320e9, May 13 2020, 16:29:34) 
[Clang 6.0 (clang-600.0.57)] on darwin
Type "help", "copyright", "credits" or "license()" for more information.
>>> #importing packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2.aruco as aruco
import time
import cv2
import numpy as np
import CameraFunctions as cf
import smbus
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import struct
import ctypes

b=0

# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

lcd_columns = 16
lcd_rows = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

# This is the address we setup in the Arduino Program
address = 0x04

def writeNumber(value, description):
    
    
    if value == None:
        return 'HAHA'
    elif (isinstance(value, float) or (value > 128 or value < -129)):
        #print("Float")
        #try:
            data=[]
            data = [0 for i in range(4)]
            #For the I2C we communicate the dataType and the read based on the address offset, 
            if(isinstance(value, float)):
                #These next two lines take the bits of the float and interpret them as an int
                s = ctypes.c_float(value) #print(s)
                b = ctypes.c_int.from_address(ctypes.addressof(s)).value#print(b)

                #We can then bitslice the int to isolate the 4 bytes that make up the float and store them in an array of bytes
                data[0] = (b & 0x000000FF) 
                data[1] = (b & 0x0000FF00) >> 8
                data[2] = (b & 0x00FF0000) >> 16
                data[3] = (b & 0xFF000000) >> 24

                #Finally, we can send the byte array over I2C
                #print("SEND FLOAT")
                #print(*data)
                if(description == 'distance'):
                    bus.write_i2c_block_data(address, 2, data)
                else:
                    bus.write_i2c_block_data(address, 3, data)
            else:
                #If the value passed through the function is already a long, then no python data type conversion is needed and we can just bitslice
          
                data[0] = (value & 0x000000FF) 
                data[1] = (value & 0x0000FF00) >> 8
                data[2] = (value & 0x00FF0000) >> 16
                data[3] = (value & 0xFF000000) >> 24
          
                bus.write_i2c_block_data(address, 1, data)
            #print(*data)
          
            return 2
        #except:
            return -2
    elif (value > -129 and value < 128):
       # try:
            bus.write_byte_data(address, 0, value)
            return 1
       # except:
            return -1
    
    return 0

def LCDDisplayAngle(angle):
    if angle == None:
        lcd.clear
        lcd.message = "NO MRKR!"
        return -1
    lcd.color = [100, 100, 100]
    #time.sleep(1)
    lcd.message = "MRKR Found " + "\nAngle: " + str(round(float(angle),2))
    #lcd.color = [0, 0, 0]
    #time.sleep(1)
    return 1

#initializing camera
HEIGHT =1232
WIDTH = 1664
camera = PiCamera()
rawCapture = PiRGBArray(camera)
time.sleep(0.1)
camera.resolution = (WIDTH, HEIGHT)
rawCapture.truncate(0)

#creating aruco dictonary and parameters
arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
arucoParam = aruco.DetectorParameters_create()

#calibrating Camera
cal = False
if cal:
    #taking 12 calibration images
    for x in range(12):
        imageName = 'cal'+str(x)+'.jpg'
        camera.capture(rawCapture, format='bgr')
        img = rawCapture.array
        cv2.imwrite(imageName, img)
        cv2.imshow(imageName, img)
        cv2.waitKey(1000)
        cv2.destroyAllWindows()
        rawCapture.truncate(0)
    #sends pictures through calibration function
    ret, mtx, dist, rvecs, tvecs = cf.calibrateCam()

    #saving calibration results in files for later use
    saveRet = open("RET","wb")
    np.save(saveRet, ret)
    saveRet.close

    saveMtx = open("MTX","wb")
    np.save(saveMtx, mtx)
    saveMtx.close

    saveDist = open("DIST","wb")
    np.save(saveDist, dist)
    saveDist.close

    saveRvecs = open("RVECS","wb")
    np.save(saveRvecs, rvecs)
    saveRvecs.close
    
    saveTvecs = open("TVECS","wb")
    np.save(saveTvecs, tvecs)
    saveTvecs.close

#Loading calibration settings from files
readRet = open("RET", "rb")
ret = np.load(readRet)
readRet.close

readMtx = open("MTX","rb")
mtx = np.load(readMtx)
readMtx.close

readDist = open("DIST","rb")
dist = np.load(readDist)
readDist.close

readRvecs = open("RVECS","rb")
rvecs = np.load(readRvecs)
readRvecs.close

readTvecs = open("TVECS","rb")
tvecs = np.load(readTvecs)
readTvecs.close

loop = True
currentMarker = 0;
#taking pictures continuously  MAIN LOOP
while loop:
    try:
        #take image and undistort
        camera.capture(rawCapture,format='bgr')
        img = rawCapture.array
        h, w = img.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
        dst = cv2.undistort(img, mtx,dist, None, newcameramtx)
        x,y,w,h = roi
        h, w = dst.shape[:2]

         #converting to grayscale
        grayImg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        #detecting markers
        (corners, ids, rejected) = aruco.detectMarkers(grayImg, arucoDict, parameters=arucoParam)


        #show image
        #cv2.imshow('undistorted image',dst)
        #cv2.waitKey(1)

        #get angle and distance
        angle = cf.getAngle(dst, h, w, corners, ids, currentMarker)
        distance = cf.getDistance(dst, h, w, corners, ids, currentMarker)
        #LCDDisplayAngle(angle)

        if angle != None:
            writeNumber(float(angle),'')
        if distance != None:
            writeNumber(float(distance),'distance')
        
        
        rawCapture.truncate(0)
    #exits loop when ctl+c is pressed
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        loop = False