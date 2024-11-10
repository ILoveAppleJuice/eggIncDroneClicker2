import numpy as np
import cv2
from mss import mss
from PIL import Image
import pyautogui
import math
import keyboard

#wha
pyautogui.PAUSE = 0

bounding_box = {'top': 270, 'left': 680, 'width': 500,'height': 550}
sct = mss()

brownLower = np.array([193,152,79])
brownUpper = np.array([193,152,79])

imgScale = 1

params = cv2.SimpleBlobDetector.Params()
params.filterByColor = True
params.blobColor = 255
params.filterByArea = False
params.minArea = 5
params.filterByCircularity = False
params.filterByConvexity = False
params.filterByInertia = False

detector = cv2.SimpleBlobDetector.create(params)


def filterImg(img):
    #in RGB color space

    #filter brown
    res = cv2.inRange(img, brownLower, brownUpper)
    return res

def detectBlobs(img):
    keypoints = detector.detect(img)
    return keypoints

def drawBlobs(img,keypoints):
    img_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    return img_with_keypoints


prevBlobs = []
while True:
    if keyboard.is_pressed("b"):
        break

    sct_img = sct.grab(bounding_box)
    img = np.array(sct_img)
    img = cv2.cvtColor(img,cv2.COLOR_BGRA2RGB) #do image processing in RGB for simplicity sake
    
    img = cv2.resize(img,None,fx=imgScale,fy=imgScale)

    #first filter out image for the brown box color on the drone
    img = filterImg(img)
    #then run blob detection to find clumps of the white pixel from the binary mask generated from the filter
    keypoints = detectBlobs(img)
    img = drawBlobs(img,keypoints)

    if prevBlobs:
        #init a list for blobs that changed in position
        changeBlobs = []
        for p in keypoints:
            change = True #initialize change to true
            for p_ in prevBlobs:
                #if there are previous blobs within a threshold of the curr position: means that it is not moving -> not a drone
                if math.fabs(((p.pt[0] - p_.pt[0]) + (p.pt[1]-p_.pt[1]))/2) <= 1:
                    change = False
                    break
            if change:
                changeBlobs.append(p.pt)

        #go through change blobs and do stuff
        for pt in changeBlobs:
            #make point int
            pt = (int(pt[0]),int(pt[1]))

            #draw circle
            cv2.circle(img,pt,10,[0,255,0],2)

            #do mouse actions
            pyautogui.moveTo(bounding_box["left"] + int(pt[0]/imgScale), bounding_box["top"] + int(pt[1]/imgScale),_pause=False)
            pyautogui.leftClick()

    prevBlobs = keypoints

    #cvt back to bgr
    img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
    cv2.imshow("poop",img)
    
    if (cv2.waitKey(1) & 0xFF) == ord('q'):
        break

cv2.destroyAllWindows()