# -*- coding: utf-8 -*-
"""
Created on Mon Dec  2 00:53:06 2024

@author: t4mar
"""

# -*- coding: utf-8 -*-
"""
Created on Fri Nov 29 00:02:19 2024

@author: t4mar
"""

# -*- coding: utf-8 -*-
"""
Created on Tue Nov 19 10:20:26 2024

@author: t4mar
"""
# https://learnopencv.com/camera-calibration-using-opencv/
# https://stackoverflow.com/questions/42119899/opencv-cv2-python-findchessboardcorners-failing-on-seemingly-simple-chessboard
#!/usr/bin/env python
 
import cv2
import numpy as np
import os
import glob
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation
import math
import socket
import random
import serial 
import time 
arduino = serial.Serial(port='COM6', baudrate=115200, timeout=.1) 
	   

isVideo = 0


# For troubleshooting https://stackoverflow.com/questions/7749341/basic-python-client-socket-example
HOST = 'localhost'                 # Symbolic name meaning all available interfaces
PORT = 12000              # Arbitrary non-privileged port
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

PI = 3.1415926535
'''
while True:
    #s.send("test\n".encode());
    data = random.random()        #simulated data
    data = (str(data) + "\n").encode()
    s.send(data)
'''
 
# Defining the dimensions of checkerboard
CHECKERBOARD = (10,7)
num_corners = 10*7
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# Extracting path of individual image stored in a given directory
images = glob.glob('./images/*.jpg')
num_images = 4;


# Open the default camera
cam = cv2.VideoCapture(0)
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))


# Capture Images
trials = 999
trial = 0
while trial < trials:
    # Creating vector to store vectors of 3D points for each checkerboard image
    objpoints = []
    # Creating vector to store vectors of 2D points for each checkerboard image
    imgpoints = [] 
    
    #cam = cv2.VideoCapture(0)
    # Defining the world coordinates for 3D points
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)*2.5
    prev_img_shape = None
    frames = np.zeros((num_images, frame_height, frame_width, 3)).astype('float32')
    if (not isVideo):
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (frame_width, frame_height), True)
        for i in range(0,num_images):
            ret, frame = cam.read()
            # Write the frame to the output file
            out.write(frame)
            # cv2.imshow('Camera', frame)
            frames[i,:,:,:] = frame
            
        # Release the capture and writer objects
        #cam.release()
        out.release()
    
    
    # Read Images from Video Instead
    if(isVideo):
        i= 0
        # Open the video file
        video = cv2.VideoCapture("output.mp4")
        
        # Check if the video opened successfully
        if not video.isOpened():
            print("Error opening video file")
        
        # Read frames until the video ends
        for i in range(0,num_images):
            ret, frames[i,:,:,:] = video.read()
    
    
    for i in range(0,num_images):
        img = frames[i,:,:,:]
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # https://stackoverflow.com/questions/25485886/how-to-convert-a-16-bit-to-an-8-bit-image-in-opencv
        # Needs to be 8 bit image
        ratio = np.amax(gray) / 256
        gray8 = (gray / ratio).astype('uint8')
        #gray8[np.where(gray8 > 125)] = 255
        cv2.imshow('Image', gray8)
        # Find the chess board corners
        # If desired number of corners are found in the image then ret = true
        # https://docs.opencv.org/4.x/d4/d8c/tutorial_py_shi_tomasi.html
        #corners = cv2.goodFeaturesToTrack(gray,num_corners,0.01,5)
        ret, corners = cv2.findChessboardCorners(gray8, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ret:
            corners = np.int0(corners)
    
             
            for i in corners:
                x,y = i.ravel()
                cv2.circle(img,(x,y),3,255,-1)
             
            plt.imshow(img),plt.show()
            print("Found corners")
            """
            If desired number of corner are detected,
            we refine the pixel coordinates and display 
            them on the images of checker board
            """
        
            if np.size(corners,0):
                print("Found valid corners")
                objpoints.append(objp)
                # refining pixel coordinates for given 2d points.
                corners = np.ascontiguousarray(corners, dtype=np.float32)
        
                corners2 = cv2.cornerSubPix(gray8, corners, (11,11),(-1,-1), criteria)
                 
                imgpoints.append(corners2)
         
                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners, ret)
             
            plt.imshow(img),plt.savefig(r"C:\Users\t4mar\Documents\ENME351\Processing\Lab9\data\Dots.png")
        
     
     
    h,w = img.shape[:2]
     
    """
    Performing camera calibration by 
    passing the value of known 3D points (objpoints)
    and corresponding pixel coordinates of the 
    detected corners (imgpoints)
    """
    if len(imgpoints)> 0:
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
     
        # print("Camera matrix : \n")
        # print(mtx)
        # print("dist : \n")
        # print(dist)
        # print("rvecs : \n")
        # print(rvecs)
        # print("tvecs : \n")
        # print(tvecs)
        
        ret, rvec, tvec = cv2.solvePnP(objpoints[-1], imgpoints[-1], mtx, dist)
        # d = math.sqrt(tvec[0]*tvec[0] + tvec[1]*tvec[1] + tvec[2]*tvec[2])
        
        # https://stackoverflow.com/questions/61360556/opencv-camera-calibration-rotation-vector-to-matrix
        rotation_object = Rotation.from_rotvec(np.squeeze(rvec))
        rotation_matrix = rotation_object.as_matrix()
        
        # https://stackoverflow.com/questions/11514063/extract-yaw-pitch-and-roll-from-a-rotationmatrix
        yaw=math.atan2(rotation_matrix[1,0],rotation_matrix[0,0]);
        pitch=math.atan2(-rotation_matrix[2,0],math.sqrt(rotation_matrix[2,1]**2+rotation_matrix[2,2]**2));
        roll=math.atan2(rotation_matrix[2,1],rotation_matrix[2,2]);
        
        # https://stackoverflow.com/questions/73976281/correct-camera-position-from-vectors-from-solvepnp-pythonopencv
        camera_position = -np.matrix(rotation_matrix).T @ np.matrix(tvec)
        tx = camera_position[0][0]
        ty = camera_position[1][0]
        tz = camera_position[2][0]
        d = math.sqrt(tx*tx + ty*ty + tz*tz)
        '''
        if yaw > PI/2:
            yaw = yaw - PI
        if pitch > PI/2:
            pitch = pitch - PI
        if roll > PI/2:
            roll = roll - PI
        '''
        
        data = (str(yaw) + "\t"+ str(pitch) + "\t"+ str(roll) + "\t"+ str(d) + "\n").encode()
        s.send(data)
        print(rotation_matrix)
        arduino.write(bytes(data, 'utf-8')) 
        time.sleep(0.05) 
        print(arduino.readline().decode("utf-8").strip('\n').strip('\r'))
    else:
        data = (str(0) + "\t"+ str(0) + "\t"+ str(0) + "\t"+ str(-1) + "\n").encode()
        s.send(data)
        arduino.write(data) 
        time.sleep(0.05)
        # https://stackoverflow.com/questions/42769611/arduino-to-python-is-serial-input-from-readline-an-integer-or-an-actual-stri
        print(arduino.readline().decode("utf-8").strip('\n').strip('\r'))
    trial += 1