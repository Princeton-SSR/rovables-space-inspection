import os
import cv2
import serial
import numpy as np
frameCount = 0
cap = cv2.VideoCapture(1) #Get webcam
w = int(cap.get(3))
h = int(cap.get(4))
size = (w, h)
print(size)
crop_offset = 30
#GREEN
lower_green = np.array([50, 10, 100])
upper_green = np.array([95, 255, 255])
#BLUE
lower_blue = np.array([105,50,110])
upper_blue = np.array([120,255,255])
#RED
lower_red = np.array([30,150,50])
upper_red = np.array([255,255,180])


#Get through the intial gopro screen
for i in range(20):
    ret, frame = cap.read()

#Capture and save the position of the arena
ret, frame = cap.read()
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
contours_blue, hierarchy = cv2.findContours(mask_blue, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
#Detect and crop the image to the arena
arena_center = 0
arena_w = 0
arena_h = 0
for c in contours_blue:
    area = cv2.contourArea(c)
    if (area > 100):
        arena_x,arena_y,arena_w,arena_h = cv2.boundingRect(c)
        arena_center = (arena_x, arena_y)
        # cv2.circle(frame, (int(x), int(y)), 10, (255, 0, 0), -1)
        # cv2.putText(frame, "Arena Center", (int(x) - 20, int(y) - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # #cv2.drawContours(frame, c, -1, (0,255,0), 3)
        #cv2.rectangle(frame,(brx,bry),(brx+brw,bry+brh),(0,255,0),2)
print(arena_center)
print(arena_w, arena_h)
while(True):
    ret, frame = cap.read()


    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    contours_green, hierarchy = cv2.findContours(mask_green, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    
    # lower mask (0-10)
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(hsv, lower_red, upper_red)

    # upper mask (170-180)
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    mask_red = mask0+mask1
    contours_red, hierarchy = cv2.findContours(mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    for c in contours_green:
        area = cv2.contourArea(c)
        if (area > 5):
            brx,bry,brw,brh = cv2.boundingRect(c)
            x = brx+brw/2
            y = bry+brh/2
            cv2.circle(frame, (int(x), int(y)), 10, (0, 255, 0), -1)
            cv2.putText(frame, "Rovable", (int(x) - 20, int(y) - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            #cv2.drawContours(frame, c, -1, (0,255,0), 3)

    for c in contours_red:
        area = cv2.contourArea(c)
        if (area > 50):
            brx,bry,brw,brh = cv2.boundingRect(c)
            x = brx+brw/2
            y = bry+brh/2
            arena_center = (x, y)
            cv2.circle(frame, (int(x), int(y)), 10, (255, 0, 0), -1)
            cv2.putText(frame, "Vibration Source", (int(x) - 20, int(y) - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            #cv2.drawContours(frame, c, -1, (0,255,0), 3)
            cv2.rectangle(frame,(brx,bry),(brx+brw,bry+brh),(0,255,0),2)
    
    cv2.imshow('Frame', frame)
    #cv2.imshow('Frame', frame[bry:bry+brh,brx:brx+brw])

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break