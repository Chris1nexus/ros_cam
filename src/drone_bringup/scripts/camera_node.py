#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

import cv2
import depthai as dai
import numpy as np

from pathlib import Path


import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import argparse
import math
import os


# Marker id infos. Global to access everywhere. It is unnecessary to change it to local.
firstMarkerID = None
secondMarkerID = None

#cap = cv2.VideoCapture(0)
image_width = 0
image_height = 0

#hyper parameters
distanceBetweenTwoMarkers = 0.0245  # in meters, 2.45 cm
oneSideOfTheMarker = 0.023 # in meters, 2.3 cm

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


class Namespace:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

def calibrate(dirpath):
    """ Apply camera calibration operation for images in the given directory path. """
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
    CHESS_SQUARE_LEN = 0.027
 
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*9,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)*CHESS_SQUARE_LEN
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    images = glob.glob(os.path.join(dirpath,"*.jpg"))
    gray = None
    for fname in images:
        print(fname)
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (9,6), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)
            # Draw and display the corners
            #cv2.drawChessboardCorners(img, (9,6), corners2, ret)
            #cv2.imshow('img', img)
            #cv2.waitKey(500)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    return [ret, mtx, dist, rvecs, tvecs]


def saveCoefficients(mtx, dist, path):
    """ Save the camera matrix and the distortion coefficients to given path/file. """
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    cv_file.write("camera_matrix", mtx)
    cv_file.write("dist_coeff", dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()


def loadCoefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("camera_matrix").mat()
    dist_matrix = cv_file.getNode("dist_coeff").mat()

    # Debug: print the values
    # print("camera_matrix : ", camera_matrix.tolist())
    # print("dist_matrix : ", dist_matrix.tolist())

    cv_file.release()
    return [camera_matrix, dist_matrix]


def inversePerspective(rvec, tvec):
    """ Applies perspective transform for given rvec and tvec. """
    rvec, tvec = rvec.reshape((3, 1)), tvec.reshape((3, 1))
    R, _ = cv2.Rodrigues(rvec)
    R = np.matrix(R).T
    invTvec = np.dot(R, np.matrix(-tvec))
    invRvec, _ = cv2.Rodrigues(R)

    invTvec = invTvec.reshape((3, 1))
    invTvec = invTvec.reshape((3, 1))
    return invRvec, invTvec


def make_1080p():
    global image_width
    global image_height
    image_width = 1920
    image_height = 1080
    #change_res(image_width, image_height)


def make_720p():
    global image_width
    global image_height
    image_width = 1280
    image_height = 720
    #change_res(image_width, image_height)


def make_480p():
    global image_width
    global image_height
    image_width = 640
    image_height = 480
    #change_res(image_width, image_height)


#def change_res(width, height):
#    cap.set(3, width)
#    cap.set(4, height)


def relativePosition(rvec1, tvec1, rvec2, tvec2):
    """ Get relative position for rvec2 & tvec2. Compose the returned rvec & tvec to use composeRT with rvec2 & tvec2 """
    rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape((3, 1))
    rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))

    # Inverse the second marker, the right one in the image
    invRvec, invTvec = inversePerspective(rvec2, tvec2)

    info = cv2.composeRT(rvec1, tvec1, invRvec, invTvec)
    composedRvec, composedTvec = info[0], info[1]

    composedRvec = composedRvec.reshape((3, 1))
    composedTvec = composedTvec.reshape((3, 1))
    return composedRvec, composedTvec


def euclideanDistanceOfTvecs(tvec1, tvec2):
    return math.sqrt(math.pow(tvec1[0]-tvec2[0], 2) + math.pow(tvec1[1]-tvec2[1], 2) + math.pow(tvec1[2]-tvec2[2], 2))

def euclideanDistanceOfTvec(tvec):
    return euclideanDistanceOfTvecs(tvec, [0, 0, 0])

def draw(img, imgpts, color):
    """ draw a line between given two points. """
    imgpts = np.int32(imgpts).reshape(-1, 2)
    for pointf in range(len(imgpts)):
        for points in range(len(imgpts)):
            img = cv2.line(img, tuple(imgpts[pointf]), tuple(imgpts[points]), color, 3)
    return img


def set_camera_input_fn(dai_input_camera_node, compute_node):
    dai_input_camera_node.preview.link(compute_node.input)
    

class DAIComputeNode(object):
    def __init__(self, dai_pipeline, dai_input_node, nn_blob_abs_path, dai_set_input_node_function=set_camera_input_fn, compute_node_name="compute"):

        
        compute_node = dai_pipeline.createNeuralNetwork()
        compute_node.setBlobPath(nn_blob_abs_path)
        compute_node.input.setBlocking(False)


        # abstraction to any possible input source
        dai_set_input_node_function(dai_input_node, compute_node)
        # an example of a specialized way to attach an input camera node is cam_rgb.preview.link(compute_node.input)

        # Send NN out to the host via XLink
        compute_out = dai_pipeline.createXLinkOut()
        compute_out.setStreamName(compute_node_name)
        compute_node.out.link(compute_out.input)

        self.compute_in = dai_input_node
        self.compute_node = compute_node
        self.compute_out = compute_out



def talker():

    # Start defining a pipeline
    pipeline = dai.Pipeline()

    # Define a source - color camera
    cam_rgb = pipeline.createColorCamera()


    global image_width
    global image_height

    make_480p()


    ret, matrix_coefficients, distortion_coefficients, rvecs, tvecs = calibrate("../chessboard")

    if not os.path.exists( "../params"):
        os.makedirs( "../params")
    saveCoefficients(matrix_coefficients, distortion_coefficients, "../params/calibration_coefficients.yaml")




    cam_rgb.setPreviewSize(image_width, image_height)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

    # Create output
    xout_rgb = pipeline.createXLinkOut()
    xout_rgb.setStreamName("rgb")
    cam_rgb.preview.link(xout_rgb.input)



    #xIn = pipeline.createXLinkIn()
    #xIn.setStreamName("image_inputer")
    
    bbBlobPath = '../models/simple_compute.blob'
    blob_path = str((Path(__file__).parent / Path(bbBlobPath)).resolve().absolute())    
    computeNode = DAIComputeNode(pipeline, cam_rgb, blob_path, dai_set_input_node_function=set_camera_input_fn, compute_node_name="compute")




    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)  # Use 5x5 dictionary to find markers
    parameters = aruco.DetectorParameters_create()  # Marker detection parameters


    with dai.Device(pipeline) as device:
        # Start pipeline
        device.startPipeline()
        # Output queue will be used to get the rgb frames from the output defined above
        q_rgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
        q_compute = device.getOutputQueue(name="compute", maxSize=1, blocking=False)
        #q_inputer = device.getInputQueue("image_inputer")#, maxSize=4, blocking=False)

        pub = rospy.Publisher('img_topic', String, queue_size=10)
        rospy.init_node('camera_publisher', anonymous=True)
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            in_rgb = q_rgb.tryGet()  # blocking call, will wait until a new data has arrived
            in_compute = q_compute.tryGet()

            if in_rgb is not None and in_compute is not None:
                frame = in_rgb.getCvFrame()
                
                vector = in_compute.getData()
                ## RGB CONVENTION FOLLOWED BOTH IN THE INPUT TO THE NET AND IN THE OUTPUT (R,G,B)
                # computation graph result
                channel_means = in_compute.getLayerFp16('Mean')
                print(channel_means)







                # from here, aruco detection through the camera 
                isFirstMarkerDetected = False
                isSecondMarkerDetected = False
                #ret, frame = cap.read()
                # operations on the frame come here
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
                h,  w = gray.shape[:2]
                newcameramtx, roi = cv2.getOptimalNewCameraMatrix(matrix_coefficients, distortion_coefficients, (w,h), 1, (w,h))
                # undistort
                dst = cv2.undistort(gray, matrix_coefficients, distortion_coefficients, None, newcameramtx)
                # crop the image
                """
                x, y, w, h = roi
                dst = dst[y:y+h, x:x+w]
                #gray = dst
                #print()
                """
                gray = dst
                


                # lists of ids and the corners beloning to each id
                corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
                                                                        parameters=parameters,
                                                                        cameraMatrix=matrix_coefficients,
                                                                        distCoeff=distortion_coefficients)


                if ids is not None: #np.all(ids is not None):  # If there are markers found by detector
                        zipped = zip(ids, corners)
                        ids, corners = zip(*(sorted(zipped,key=lambda x: x[0][0]))) #sorted(zipped,key=lambda x: x[0][0])

                        # print(ids)
                        for i in range(0, len(ids)):  # Iterate in markers
                            # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                            rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], oneSideOfTheMarker, matrix_coefficients,
                                                                                       distortion_coefficients)

                            #if ids[i] == firstMarkerID:
                            firstRvec = rvec
                            firstTvec = tvec
                            isFirstMarkerDetected = True
                            firstMarkerCorners = corners[i]
         

                            (rvec - tvec).any()  # get rid of that nasty numpy value array error
                            # aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  # Draw Axis
                            aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers

                            
                            # left hand coordinates
                            rel_y, rel_z, rel_x = firstTvec.squeeze()

                            print(f"x:{rel_x}, y:{rel_y}, z:{rel_z}")
                
                            realDistanceInTvec = euclideanDistanceOfTvec(firstTvec.squeeze())
                            print(cv2.norm(firstTvec.squeeze() ))

                        
                #'''

                # Display the resulting frame
                cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('frame', image_width, image_height)
                cv2.imshow('frame', frame)
                # Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.
                key = cv2.waitKey(3) & 0xFF
                if key == ord('q'):  # Quit
                    break
                elif key == ord('p'):  # print necessary information here
                    pass  # Insert necessary print here


                hello_str = "hello world {}".format(frame.shape)
                rospy.loginfo(hello_str)


                pub.publish(hello_str)
                rate.sleep()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
