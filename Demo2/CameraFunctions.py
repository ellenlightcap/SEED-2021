import cv2
import cv2.aruco as aruco
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import math
import glob

#detects aruco markers in an image and the angle
def getAngle(img, height, width, corners, ids):

    #finding angle
    FIELDOFVIEW = 54
    corners = np.array(corners)

    if (ids != None):
        #finds angle
        for marker in range(0,ids.size):
            mark = np.array(corners[marker][0])

            #getting x coordinate of center of marker
            x = ((mark[1][0] - mark[0][0])/2) + mark[0][0]
            print('x: ',x)
            #determining if the angle is to the left or right
            distLeft = x
            distRight = width - x
            if(distLeft < distRight):
                side = 'left'
                sideSign = 1
                dist = distLeft
            elif(distRight < distLeft):
                 side = 'right'
                 sideSign = -1
                 dist = distRight
            else:
                 side = 'Center'
                 dist = distLeft
                
            #calculating angle
            angle = (FIELDOFVIEW/2)*(((width/2)-dist)/(width/2))*sideSign
            print("Marker Angle: ",angle)
            return angle

    #get warning about comparing to None... should probably fix that
    else:
        print("No markers detected")

#get distance of marker from camera    
def getDistance(img, height, width, corners, ids):

    #THESE CONSTANTS ARE NOT UPDATED FOR NEW BEACONS
    MARKERSIDELENGTH = 4.875
    REFDISTANCE = 24
    PIXELREF = 127
    
    if (ids != None):
        #finds distance for all markers
        for marker in range(0,ids.size):
            mark = np.array(corners[marker][0])

            #distance Might become a problem if it sees more than 1 marker
            markerHeight = mark[2][1] - mark[0][1]
            inchPerPixel = MARKERSIDELENGTH/markerHeight
            focalLength = (PIXELREF * REFDISTANCE) / MARKERSIDELENGTH
            zdistance = MARKERSIDELENGTH * focalLength / markerHeight
            print(" id: ",ids[marker],'\n',"distance: ",zdistance)
            return zdistance

    #get warning about comparing to None... should probably fix that
    else:
        print("No markers detected")

#calibrate camera with pretaken images
def calibrateCam():
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
    
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    #array of calibration image names
    images = ['cal0.jpg','cal1.jpg','cal2.jpg','cal3.jpg','cal4.jpg','cal5.jpg','cal6.jpg','cal7.jpg','cal8.jpg','cal9.jpg','cal10.jpg','cal11.jpg']
    
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        print(fname)
        
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (7,6), None)
        
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)
            
            # Draw and display the corners
            cv2.drawChessboardCorners(img, (7,6), corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)
    cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    print ('___Calibration Values___')
    print ('ret: ',ret)
    print ('mtx: ',mtx)
    print ('dist: ',dist)
    print ('rvecs: ',rvecs)
    print ('tvecs: ',tvecs)

    #checking reprojection error
    meanError = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        meanError += error
    print('total error: {}'.format(meanError/len(objpoints)))
    return ret, mtx, dist, rvecs, tvecs

