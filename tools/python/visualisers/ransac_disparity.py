#!/usr/bin/env python
import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')
import cv2
import serial
import stereoboard_tools
import Tkinter as tk
import numpy as np
import time
import argparse
import matplotlib.pyplot as plt
from matplotlib import pylab
from mpl_toolkits.mplot3d import Axes3D


from ransac import *
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


frameNumber = 0
saveImages= True

raw_img = []
currentBuffer=[]
now = time.clock()
cv2.namedWindow('stereoimg',cv2.WINDOW_NORMAL)

parser = argparse.ArgumentParser(description='This will display the disparity image from the stereo camera. Press q to quit and s to save a frame.')
parser.add_argument("-p", "--port", type=str, default='/dev/ttyUSB0', help="The port name of the camera (/dev/ttyUSB0)")
parser.add_argument("-b", "--baud", type=int, default=921600, help="The baud rate of the camera (921600)")
#parser.add_argument("-o", "--other", action='store_true', help="Print other messages from camera")
parser.add_argument("-s", action='store_true', help="Save all incoming images")

args = parser.parse_args()
    
ser = serial.Serial(args.port,args.baud,timeout=None)
saveImages = args.s
def augment(xyzs):
	axyz = np.ones((len(xyzs), 4))
	axyz[:, :3] = xyzs
	return axyz
def is_inlier(coeffs, xyz, threshold):
	return np.abs(coeffs.dot(augment([xyz]).T)) < threshold
def estimate(xyzs):
	axyz = augment(xyzs[:3])
	return np.linalg.svd(axyz)[-1][-1, :]

# main loop:
while True:
    try:
        # Read the image
        currentBuffer, location, endOfImagesFound = stereoboard_tools.readPartOfImage(ser, currentBuffer)
        startPosition=location[0]
        endPosition=location[1]
        
        if endOfImagesFound > 0:
            oneImage = currentBuffer[startPosition:endPosition]
            currentBuffer = currentBuffer[endPosition::]
            
            # Search the startbyte
            sync1, length, lineLength, lineCount=stereoboard_tools.determine_image_and_line_length(oneImage)
            #print 'length: ', length, ' count: ', lineCount, ' lineLength: ', lineLength, ' end of images found: ', endOfImagesFound
            if sync1 < 0:    # We did not find the startbit... try again
                continue
                
            img = stereoboard_tools.fill_image_array(sync1, oneImage, lineLength, lineCount)
            if img.size <= 32:
              print img
              count = 0
            elif img.size >= 12288:
              raw_img = img
              if saveImages:
                cv2.imwrite('images/disp{0}.png'.format(frameNumber), raw_img)
                frameNumber += 1
                
              img *= 2
              
              #print 1 / (time.clock() - now)
              now = time.clock()

              
	      x,y = img.nonzero()
	      print img[x,y]
	      plt.cla()
              ax.scatter3D(x, y, img[x,y], zdir='z', c= img[x,y])
	      ax.set_xlim([0, 96])
	      ax.set_ylim([0, 128])
	      ax.set_zlim([0, 200])
	      plt.draw()
    	      plt.pause(0.05)


	      xyzs = np.zeros((len(x), 3))
	      xyzs.T[0] = x
	      xyzs.T[1] = y
	      xyzs.T[2] =  img[x,y]
	      def plot_plane(a, b, c, d):
		xx, yy = np.mgrid[:128, :96]
		return xx, yy, (-d - a * xx - b * yy) / c
	      
	      print("length",len(x))

              goal_inliers =200
	      max_iterations = 100
	      m, b = run_ransac(xyzs, estimate, lambda l, i: is_inlier(l, i, 1), 3, goal_inliers, max_iterations)
	      a, b, c, d = m
	      xx, yy, zz = plot_plane(a, b, c, d)
	      ax.plot_surface(xx, yy, zz, color=(0, 1, 0, 0.5))	

	      plt.draw()
    	      plt.pause(0.05)

 	      img = cv2.resize(img,(0,0),fx=3,fy=3 ,interpolation=cv2.INTER_NEAREST)
              cv2.imshow('stereoimg',img)
	                   # Create a color image
              img = stereoboard_tools.createRedBlueImage(img,lineCount,lineLength)
              #img = img[::2]
              # if (not '3.0.0'==cv2.__version__) and (not '3.0.0-dev'==cv2.__version__):
              #     print 'resizing stuff!'

              key=cv2.waitKey(1)
	      



            if key == ord('q'):
              break
            elif key == ord('s') and ~saveImages:
              cv2.imwrite('images/disp{0}.png'.format(frameNumber), raw_img)
              frameNumber += 1
	

    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep
