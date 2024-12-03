# -*- coding: utf-8 -*-
"""
Created on Tue Nov 19 10:20:26 2024

@author: Terrence Pierce
"""
# Helpful resources
# https://learnopencv.com/camera-calibration-using-opencv/
# https://stackoverflow.com/questions/42119899/opencv-cv2-python-findchessboardcorners-failing-on-seemingly-simple-chessboard
 
# Import important libraries
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

# Initialize Serial Port connection to Arduino
arduino = serial.Serial(port='COM6', baudrate=115200, timeout=.1) 

# User-set parameter regarding saving vs loading data
isVideo = 0


# For troubleshooting https://stackoverflow.com/questions/7749341/basic-python-client-socket-example
# Initialize connection to Processing server on the local host
HOST = 'localhost'                 
PORT = 12000    # Any Port works
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

# Define PI constant
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
 
# User-set parameter determining how many images are used for calibration
num_images = 4;


# Open the default camera
cam = cv2.VideoCapture(0)
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))


# Capture Images
trials = 99999
trial = 0
# Can set how many times to run data fetching 
while trial < trials:
    # Check if the arduino is ready to receive data
    if arduino.readline().decode("utf-8").strip('\n').strip('\r') == "Arduino Ready":
        start_time = time.perf_counter()
        print("Arduino Ready")
        
        # Clear any backlog of Arduino messages
        while arduino.in_waiting:
            arduino.read()
            # print("In Waiting")
        
        
        # Send 3 times in case of dropped packets
        arduino.write("Python Ready".encode())
        time.sleep(0.05)
        arduino.write("Python Ready".encode())
        time.sleep(0.05)
        arduino.write("Python Ready".encode())
        time.sleep(0.05)
        # Inspired by resources above
        # Creating vector to store vectors of 3D points for each checkerboard image
        objpoints = []
        # Creating vector to store vectors of 2D points for each checkerboard image
        imgpoints = [] 
                
        print("After Python Ready " + str(time.perf_counter() - start_time));
        
        # Initialize data vectors (partially comes from different resources above)
        objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)*2.5
        prev_img_shape = None
        frames = np.zeros((num_images, frame_height, frame_width, 3)).astype('float32')
        
        ''' Optiional code if saving a video or reading from a video is useful
        # I wrote this mainly for troubleshooting
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
        '''
        # Efficiency improvements in getting frames
        for i in range(0,num_images):
            ret, frame = cam.read()
            # cv2.imshow('Camera', frame)
            frames[i,:,:,:] = frame
        
        print("After Get Frames " + str(time.perf_counter() - start_time));
        
        # Get the corners of the chessboard to determine the rotation matrix
        for i in range(0,num_images):
            img = frames[i,:,:,:]
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            # https://stackoverflow.com/questions/25485886/how-to-convert-a-16-bit-to-an-8-bit-image-in-opencv
            # Needs to be 8 bit image
            ratio = np.amax(gray) / 256
            gray8 = (gray / ratio).astype('uint8')
            
            # Find the chess board corners
            # If desired number of corners are found in the image then ret = true
            # https://docs.opencv.org/4.x/d4/d8c/tutorial_py_shi_tomasi.html
            #corners = cv2.goodFeaturesToTrack(gray,num_corners,0.01,5)
            ret, corners = cv2.findChessboardCorners(gray8, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
            
            # If findChessboardCorners found corners, proceed
            if ret:
                corners = np.int0(corners)
        
                # Helps with plotting the dots later on
                for i in corners:
                    x,y = i.ravel()
                    cv2.circle(img,(x,y),3,255,-1)
                 
                # plt.imshow(img),plt.show()
                # print("Found corners")

                # If the size is nonzero, add the results to the lists 
                if np.size(corners,0):
                    # print("Found valid corners")
                    objpoints.append(objp)
                    # refining pixel coordinates for given 2d points.
                    corners = np.ascontiguousarray(corners, dtype=np.float32)
            
                    corners2 = cv2.cornerSubPix(gray8, corners, (11,11),(-1,-1), criteria)
                     
                    imgpoints.append(corners2)
             
                    # Draw corners
                    img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners, ret)
        
        # Display the last image, which is used for the processing code
        plt.imshow(img),plt.savefig(r"C:\Users\t4mar\Documents\ENME351\Processing\Lab9\data\Dots.png")
        print("After Get Corners " + str(time.perf_counter() - start_time));
         
         
        h,w = img.shape[:2]
         
        """
        Performing camera calibration by 
        passing the value of known 3D points (objpoints)
        and corresponding pixel coordinates of the 
        detected corners (imgpoints)
        """
        if len(imgpoints)> 0:
            # Get camera and distortion matrices
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
         
            # print("Camera matrix : \n")
            # print(mtx)
            # print("dist : \n")
            # print(dist)
            # print("rvecs : \n")
            # print(rvecs)
            # print("tvecs : \n")
            # print(tvecs)
            
            # Get rotation and translation vectors
            ret, rvec, tvec = cv2.solvePnP(objpoints[-1], imgpoints[-1], mtx, dist)
            print("After SolvePnP " + str(time.perf_counter() - start_time));
            # d = math.sqrt(tvec[0]*tvec[0] + tvec[1]*tvec[1] + tvec[2]*tvec[2])
            
            # https://stackoverflow.com/questions/61360556/opencv-camera-calibration-rotation-vector-to-matrix
            # Obtains the rotation matrix which I'm more familar with
            rotation_object = Rotation.from_rotvec(np.squeeze(rvec))
            rotation_matrix = rotation_object.as_matrix()
            
            # https://stackoverflow.com/questions/11514063/extract-yaw-pitch-and-roll-from-a-rotationmatrix
            # Obtains the yaw, pitch, and roll
            yaw=math.atan2(rotation_matrix[1,0],rotation_matrix[0,0]);
            pitch=math.atan2(-rotation_matrix[2,0],math.sqrt(rotation_matrix[2,1]**2+rotation_matrix[2,2]**2));
            roll=math.atan2(rotation_matrix[2,1],rotation_matrix[2,2]);
            
            # https://stackoverflow.com/questions/73976281/correct-camera-position-from-vectors-from-solvepnp-pythonopencv
            camera_position = -np.matrix(rotation_matrix).T @ np.matrix(tvec)
            tx = camera_position[0][0]
            ty = camera_position[1][0]
            tz = camera_position[2][0]
            # Calculate the distance to the target using the camera position
            # This equation comes from the fact that a transpose is equal to 
            # an inverse for rotation matrices and then I mulitply that by
            # the matrix form of tvec to obtain camera position and then
            # distance
            d = math.sqrt(tx*tx + ty*ty + tz*tz)
            
            # Create data to send to Processing and Arduino
            data = f'{yaw:.3f}\t{pitch:.3f}\t{roll:.3f}\t{d:.3f}\n'.encode()
            # Send data to Processing
            s.send(data)
            #Clear the arduino queue to get an accurate reading
            while arduino.in_waiting:
                arduino.read()
                print("In Waiting")
                
            # Write data to Arduino
            arduino.write(data)
            time.sleep(0.1)
            # Check the returned string of Arduino
            returned_str = arduino.readline().decode("utf-8").strip()
            
            # If arduino did not return an expected value, then write again
            while not returned_str.startswith("I received:"):
                arduino.write(data) 
                time.sleep(0.1)
                returned_str = arduino.readline().decode("utf-8").strip('\n').strip('\r')
                
            print(returned_str)
            print("After Return " + str(time.perf_counter() - start_time));
        
        else:
            data = (str(0) + "\t"+ str(0) + "\t"+ str(0) + "\t"+ str(-1) + "\n").encode()
            # Send data to Processing
            s.send(data)
            #Clear the arduino queue to get an accurate reading
            while(arduino.inWaiting()):
                arduino.read()
                print("In Waiting")
                
            # Write data to Arduino
            arduino.write(data) 
            time.sleep(0.5) # Higher delay in case there was an issue related to delay when obtaining data
            print("Entering read line")
            # https://stackoverflow.com/questions/42769611/arduino-to-python-is-serial-input-from-readline-an-integer-or-an-actual-stri
            # Check the returned string of Arduino
            returned_str = arduino.readline().decode("utf-8").strip('\n').strip('\r')
            print("Exiting read line")
            
            # If arduino did not return an expected value, then write again
            while(not returned_str.startswith("I received:")):
                arduino.write(data)
                time.sleep(0.5)
                returned_str = arduino.readline().decode("utf-8").strip('\n').strip('\r')
                
            print(returned_str)
            print("After Return " + str(time.perf_counter() - start_time));
        # Increase Trial Number
        trial += 1