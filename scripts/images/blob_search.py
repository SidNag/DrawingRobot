#!/usr/bin/env python

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0.0
beta = 0.0
tx = 0.0
ty = 0.0

# Function that converts image coord to world coord
def IMG2W(col, row):
    Or = 320
    Oc = 240
    beta = (377-302)/(10/100)
    xcr = (col - Or)
    ycc = (row - Oc)
    #204, 73
    Tx = xcr + Or - 204
    Ty = ycc + Oc - 73
    xw = (Tx)/beta
    yw = (Ty)/beta
    # print(xw, yw)
    return [yw, xw]

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    # params.filterByColor = True
    # params.blobColor = 255

    # # Filter by Area.
    # params.filterByArea = True
    # # params.minArea = 50
    # params.maxArea = 810

    # # Filter by Circularity
    # params.filterByCircularity = True
    # params.minCircularity = 0.3
    # # params.maxCircularity = 0.6
    
    # # Filter by Inerita
    # params.filterByInertia = False

    # # Filter by Convexity
    # params.filterByConvexity = True
    # params.minConvexity = 0.5
    
    params.filterByConvexity = True
    params.minConvexity = 0.5
    # params.filterByArea = True
    # params.minArea = 200
    params.filterByColor = True
    params.blobColor = 255
    # params.filterByCircularity = True
    # params.minCircularity = 0.1
    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    # green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
    # print(green)
    # lower = (110,50,50)     # blue lower
    # upper = (130,255,255)   # blue upper
    
    #GreenRGB = (55,137,60)
    #RedRGB = (110,0,0)
    if color == "green":
        lower = (50,50,50)
        upper = (70,255,255)
    else:
        lower = (0,150,150)
        upper = (10,255,255)
        # lowerC = (0,50,50)
        # upperC = (20,255,255)

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)
    
    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))
        # print((keypoints[i].pt[0],keypoints[i].pt[1]))
    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, image_raw)

    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)
    # print(np.shape(xw_yw))
    return xw_yw
