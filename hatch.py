import numpy as np
import cv2
from mss import mss
from PIL import Image
import pyautogui
import math
import time

from pynput.keyboard import Key, Listener
from pynput import keyboard
from pynput.mouse import Button, Controller

mouse = Controller()


#wha
pyautogui.PAUSE = 0

bounding_box = {'top': 125, 'left': 935, 'width': 39,'height': 1}


sct = mss()

greenColor = np.array([25,172,0])
percentMaintain = 0.5


running = True
prevMousePos = mouse.position
lastStop = time.time()
while running:

    

    sct_img = sct.grab(bounding_box)
    img = np.array(sct_img)
    img = cv2.cvtColor(img,cv2.COLOR_BGRA2RGB) #do image processing in RGB for simplicity sake


    mask = cv2.inRange(img, greenColor, greenColor)
    
    # Count the number of non-zero pixels in the mask
    pixel_count = cv2.countNonZero(mask)
    down = False
    if pixel_count/bounding_box["width"] > percentMaintain:
        #pyautogui.move(943,1017)

        if math.fabs(((mouse.position[0]-prevMousePos[0]) + (mouse.position[1]-prevMousePos[1]))/2) <= 0:
            if time.time()-lastStop >= 1:
                down = True
        else:
            lastStop = time.time()
            ...
    else:
        down = False

    if down:
        pyautogui.mouseDown()
    else:
        pyautogui.mouseUp()
        
    prevMousePos = mouse.position
    #print(pixel_count,pixel_count/bounding_box["width"])

    #cvt back to bgr
    
    img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
    img = cv2.resize(img,None,fx=3,fy=20)
    cv2.imshow("poop",img)
    
    if (cv2.waitKey(1) & 0xFF) == ord('q'):
        break

pyautogui.mouseUp()

cv2.destroyAllWindows()