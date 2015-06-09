import numpy as np
import cv2
import serial
import matplotlib.pyplot as plt
import stereoboard_tools
ser = serial.Serial('/dev/ttyUSB0',1000000,timeout=None)
frameNumber = 0
saveImages= False

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


            img = stereoboard_tools.fill_disparity_array(sync1,oneImage, lineLength, lineCount)
            img =img[:,::-1]
            img /= 100

            cv2.namedWindow('img',cv2.WINDOW_NORMAL)
            cv2.imshow('img',img)


            key=cv2.waitKey(1)

            if saveImages:
                import scipy
                fileNameBoth = 'imageBoth'+str(frameNumber)+'.png'
                scipy.misc.imsave(fileNameBoth, img)
                frameNumber+=1

    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep