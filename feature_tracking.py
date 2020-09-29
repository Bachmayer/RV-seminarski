
import numpy as np
import cv2 as cv
import argparse
import time


# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

# Create some random colors
color = np.random.randint(0,255,(100,3))

# Take first frame and find corners in it
#ret, old_frame = cap.read()
old_gray = cv.imread('CameraRGB/00020.png',0)
p0 = cv.goodFeaturesToTrack(old_gray, mask = None, **feature_params)

# Create a mask image for drawing purposes
mask = np.zeros_like(old_gray)

count=20 #kako bi se preskocilo prve frame-ove dok se auto respawn-a

path_format='CameraRGB/{:0>5d}.png'

while(1):
    count = count + 1
    
    #path1=path1.replace(path1[6],str(z))
    path = path_format.format(count)
    frame_gray = cv.imread(path,0)
    # calculate optical flow
    p1, st, err = cv.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
    
    # Select good points
    good_new = p1[st==1]
    good_old = p0[st==1]

    # draw the tracks
    for i,(new,old) in enumerate(zip(good_new, good_old)):
        a,b = new.ravel()
        c,d = old.ravel()
        mask = cv.line(mask, (a,b),(c,d), color[i].tolist(), 2)
        frame_gray = cv.circle(frame_gray,(a,b),5,color[i].tolist(),-1)
    img = cv.add(frame_gray,mask)

    cv.imshow('frame',img)
    k = cv.waitKey(30) & 0xff
    if k == 27:
        break

    if count == 79:
        break  
    # Now update the previous frame and previous points
    old_gray = frame_gray.copy()
    p0 = good_new.reshape(-1,1,2)
    time.sleep(0.1)

