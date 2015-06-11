import numpy as np
import cv2
import serial
import stereoboard_tools
import array
import sys

BAUDRATE=1000000
W = 128
H=96
saveImages=True
DISPARITY_OFFSET_LEFT=0
DISPARITY_OFFSET_RIGHT=0
DISPARITY_BORDER=W/2
previousLeftImage=None
ser = serial.Serial('/dev/ttyUSB0',BAUDRATE)
size_of_one_image=25348 # 128*96*2+4*96+4*96+4

frameNumber = 0



cv2.namedWindow('img', cv2.WINDOW_NORMAL)
cv2.namedWindow('leftimg', cv2.WINDOW_NORMAL)
cv2.namedWindow('rightimg', cv2.WINDOW_NORMAL)

currentBuffer=[]
while True:
    try:
        # Read the image
        currentBuffer, location = stereoboard_tools.readPartOfImage(ser, currentBuffer)

        if location > 0:
            oneImage = currentBuffer[0:location]
            currentBuffer=currentBuffer[location::]

            # Search the startbyte
            sync1, length,lineLength, lineCount=stereoboard_tools.determine_image_and_line_length(oneImage)

            if sync1<0:    # We did not find the startbit... try again
                continue


            # Fill the image arrays
            img, leftImage, rightImage = stereoboard_tools.fill_image_arrays(
                oneImage, sync1,size_of_one_image, W, H, DISPARITY_OFFSET_LEFT,DISPARITY_OFFSET_RIGHT,DISPARITY_BORDER)

            # Go from values between 0-255 to intensities between 0.0-1.0
            img /= 255
            leftImage /= 255
            rightImage /= 255
            # Show the images
            cv2.imshow('img',img)
            cv2.imshow('leftimg',leftImage)
            cv2.imshow('rightimg',rightImage)
            key=cv2.waitKey(1)

            if saveImages:
                stereoboard_tools.saveImages(img, leftImage, rightImage, frameNumber, 'images')
                frameNumber+=1
    except:
        print 'error!'
