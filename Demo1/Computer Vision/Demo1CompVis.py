#importing packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import CameraFunctions as cf
import smbus
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

#This is setup for the LCD Display
bus = smbus.SMBus(1)
i2c = busio.I2C(board.SCL, board.SDA)
lcd_columns = 16
lcd_rows = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

#This is the I2C address of the arduino
address = 0x04

def LCDDisplayAngle(angle):
    lcd.clear()
    lcd.color = [100, 100, 100]
    #time.sleep(1)
    lcd.message = "MRKR Found " + "\nAngle: " + round(angle,2)
    lcd.color = [0, 0, 0]
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

    ret, mtx, dist, rvecs, tvecs = cf.calibrateCam()

    #saving calibration results for later use
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

#Loading calibration settings
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
#taking pictures continuously
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

        #show image and get angle of marker
        cv2.imshow('undistorted image',dst)
        cv2.waitKey(1)
        angle = cf.getAngle(dst, h, w)
        #LCDDisplayAngle(angle)
        
        
        rawCapture.truncate(0)
    #exits loop when ctl+c is pressed
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        loop = False
