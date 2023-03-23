import serial
import time
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os
import cv2
import csv
## HSV MASKS

lower_green = np.array([50, 100, 100])
upper_green = np.array([95, 255, 255])
lower_blue = np.array([105,50,110])
upper_blue = np.array([120,255,255])

# lower mask (0-10)
lower_red_l = np.array([0,50,50])
upper_red_l = np.array([10,255,255])


# upper mask (170-180)
lower_red_u = np.array([170,50,50])
upper_red_u = np.array([180,255,255])


n_robot = 2
frameCount = 0
posData = []
arena_location = 0
vibration_location = 0
b0 = []
b1 = []
b2 = []
b3 = []
runName = "90s_spread_nodamp"
port = '/dev/cu.usbmodem1201' 
lines = ""
i = 0
cap = cv2.VideoCapture(1) #Get webcam
w = int(cap.get(3))
h = int(cap.get(4))
size = (w, h)
print("Camera Dimensions: ", size)

#Get through the intial gopro screen
for i in range(20):
    ret, frame = cap.read()

while(True):    
    ret, frame = cap.read()

    #Filter initial image
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask0_red = cv2.inRange(hsv, lower_red_l, upper_red_l)
    mask1_red = cv2.inRange(hsv, lower_red_u, upper_red_u)

    mask_red = mask0_red + mask1_red
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    contours_blue, hierarchy = cv2.findContours(mask_blue, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contours_red, hierarchy = cv2.findContours(mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    #Find vibration source
    for c in contours_red:
            area = cv2.contourArea(c)
            if (area > 50):
                vibration_location = cv2.boundingRect(c)
                center = (int(vibration_location[0]) + int(vibration_location[2])/2, int(vibration_location[1]) + int(vibration_location[3])/2)
                print(center)
                cv2.circle(frame, (int(center[0]), int(center[1])), 10, (255, 0, 0), -1)
                cv2.putText(frame, "Vibration Source", (int(vibration_location[0]) - 20, int(vibration_location[1]) - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                #cv2.drawContours(frame, c, -1, (0,255,0), 3)
                #cv2.rectangle(frame,(brx,bry),(brx+brw,bry+brh),(0,255,0),2)

    #Find Arena bounds
    for c in contours_blue:
        area = cv2.contourArea(c)
        if (area > 100):
            arena_location = cv2.boundingRect(c)
            cv2.circle(frame, (int(arena_location[0]), int(arena_location[1])), 10, (255, 0, 0), -1)
            # cv2.putText(frame, "Arena Center", (int(x) - 20, int(y) - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            # #cv2.drawContours(frame, c, -1, (0,255,0), 3)
            #cv2.rectangle(frame,(brx,bry),(brx+brw,bry+brh),(0,255,0),2)
    print("Arena location: ", arena_location)
    print("Vibration lcoation: ", vibration_location)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    cv2.imshow('Frame', frame)


dest = "Data" + "/" + runName
if not os.path.isdir(dest):
    os.mkdir(dest)

result = cv2.VideoWriter(dest + '/video.avi', 
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         10, size)

s = serial.Serial(port=port, baudrate=115200)

t0 = time.time()
TOTAL_SAMPLE_TIME_SECS = 90

#while (time.time() - t0) < TOTAL_SAMPLE_TIME_SECS:
while(True):
    #ignore the first 10 frames, because gopro marketing is annoying
    ret, frame = cap.read()

    if(s.in_waiting > 0):
        rowProbData = np.zeros(n_robot)
        lines = s.readline().decode().rstrip()
        if len(lines):
            print(lines)
            decomp = lines.split(",")
            print(decomp)
            if decomp[0] == '1':
                b0.append(decomp[3])
            if decomp[0] == '2':
                b1.append(decomp[3])
            if decomp[0] == '3':
                b2.append(decomp[3])
            if decomp[0] == '4':
                b3.append(decomp[3])

    rowPosData = []

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    contours_green, hierarchy = cv2.findContours(mask_green, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contours_blue, hierarchy = cv2.findContours(mask_blue, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    for c in contours_green:
        area = cv2.contourArea(c)
        if (area > 25):
            brx,bry,brw,brh = cv2.boundingRect(c)
            x = brx+brw/2
            y = bry+brh/2
            cv2.circle(frame, (int(x), int(y)), 10, (0, 255, 0), -1)
            cv2.putText(frame, "Rovable", (int(x) - 20, int(y) - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            rowPosData.append(x)
            rowPosData.append(y)
            #cv2.drawContours(frame, c, -1, (0,255,0), 3)

    for c in contours_blue:
        area = cv2.contourArea(c)
        if (area > 20):
            brx,bry,brw,brh = cv2.boundingRect(c)
            x = brx+brw/2
            y = bry+brh/2
            cv2.circle(frame, (int(x), int(y)), 10, (255, 0, 0), -1)
            cv2.putText(frame, "Rovable", (int(x) - 20, int(y) - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            rowPosData.append(x)
            rowPosData.append(y)
            #cv2.drawContours(frame, c, -1, (0,255,0), 3)

    if frameCount == 20:
        cv2.imwrite(dest + "/" + "frameInitial.png", frame)

    if rowPosData:
        if (len(rowPosData) == 4):
            posData.append(rowPosData)

    # for i in np.arange(1, len(posData)):
    #     if (len(posData[i]) == 4) and (len(posData[i-1]) == 4):
    #         start = (int(posData[i-1][0]), int(posData[i-1][1]))
    #         end = (int(posData[i][0]), int(posData[i][1]))
    #         cv2.line(frame, start, end, (0, 0, 255), 4)

    #         start = (int(posData[i-1][2]), int(posData[i-1][3]))
    #         end = (int(posData[i][2]), int(posData[i][3]))
    #         cv2.line(frame, start, end, (0, 255, 0), 4)

    cv2.imshow('Frame', frame)
    result.write(frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.imwrite(dest + "/" + "frame.png", frame)
filenameBelief = dest + "/" + "b0.csv"
np.savetxt(filenameBelief, np.array(b0).astype(float), delimiter=',')
filenameBelief = dest + "/" + "b1.csv"
np.savetxt(filenameBelief, np.array(b1).astype(float), delimiter=',')
cv2.imwrite(dest + "/" + "frame.png", frame)
filenameBelief = dest + "/" + "b2.csv"
np.savetxt(filenameBelief, np.array(b2).astype(float), delimiter=',')
filenameBelief = dest + "/" + "b3.csv"
np.savetxt(filenameBelief, np.array(b3).astype(float), delimiter=',')

filenamePos = dest + "/" + "posData.csv"
with open(filenamePos, 'w') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerows(posData)


b0 = np.array(b0).astype(float)
b1 = np.array(b1).astype(float)
b2 = np.array(b2).astype(float)
b3 = np.array(b3).astype(float)
fig1, ax = plt.subplots()
ax.plot(b0, label='Rovable 1')
ax.plot(b1, label='Rovable 2')
ax.plot(b2, label='Rovable 3')
ax.plot(b3, label='Rovable 4')
plt.xlabel('Samples')
plt.ylabel('Robot Belief')
plt.title('Robot Belief')
plt.legend()
filenameBelief = dest + "/" + "robot_belief.png"
plt.savefig(filenameBelief, transparent=True)
plt.show()

# After the loop release the cap object
cap.release()
result.release()
# Destroy all the windows
cv2.destroyAllWindows()