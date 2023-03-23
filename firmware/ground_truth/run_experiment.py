import cv2
import numpy as np
import matplotlib.pyplot as plt
from serial import Serial
import time
'''
Below defines the workflow of the experiments needed for collecting ground truth data:
    1.) The gopro captures rovables positions
    2.) After pressing enter, a serial port is opened and T seconds of accelerometer data are collected. Then averaged.
    3.) Move the rovable to a new postion. Press enter to continue
    4.) Repeat by going back to step 1.
'''

T = 5 #This is in units of seconds
arena_location = 0
vibration_location = 0
filename = 'test.mp4'
ser = Serial('/dev/tty.usbmodem21201')  # open serial port
print("Reading Data from Serial Port: ", ser.name)

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

#pass in file name to save locally
#pass in integer to choose camera. 0 is usually the laptop webcam.
cap = cv2.VideoCapture(1)
test_video = True

rovPos = []
rovAccel = []

imWidth = cap.get(3)
imHeight = cap.get(4)

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



input("Press enter to begin experiment. . . ")

while (cap.isOpened()):
    ret, frame = cap.read()
    if ret == True:
        rovable_position = [0,0]

        while(True):
            ret, frame = cap.read()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, lower_green, upper_green)

            contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE) 
            for c in contours:
                area = cv2.contourArea(c)
                if (area > 1):
                    brx,bry,brw,brh = cv2.boundingRect(c)
                    x = brx+brw/2
                    y = bry+brh/2
                    cv2.circle(frame, (int(x), int(y)), 10, (255, 0, 0), -1)
                    cv2.putText(frame, "Rovable", (int(x) - 20, int(y) - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    rovable_position[0] = x
                    rovable_position[1] = y
            #cv2.drawContours(frame, contours, -1, (0,255,0), 3)
            cv2.imshow('Frame', frame)
            print("Rovable Position: ", rovable_position)
            if cv2.waitKey(25) & 0xFF ==ord('q'):
                break

        rovPos.append(rovable_position)
        input("Position data captured. Press Enter to continue. . .")
       

        startTime = time.time()
        count = 0
        ser.reset_input_buffer()
        accel_list = []
        while (time.time() - startTime < T):
            #if ser.in_waiting:
                #     read until EOL   decode        remove    split commas 
            accel_val = ser.readline().decode('utf-8').rstrip().split(",")
            float_accel = [float(i) for i in accel_val]
            accel_list.append(float_accel)
            print(float_accel)
            # tot = sum(accel_val)
            # count = count + 1
        
        rovAccel.append(accel_list)
        

        if cv2.waitKey(25) & 0xFF ==ord('q'):
            break

        with open('accel_data.txt', 'w') as f:
            for line in rovAccel:
                f.write(f"{line}\n")


        with open('pos_data.txt', 'w') as f:
            for line in rovPos:
                f.write(f"{line}\n")

        stop = input("Continue taking data? (y/n)")
        if (stop == 'n'):
            break
        if (stop == 'y'):
            continue

cap.release()
cv2.destroyAllWindows()


# plt.plot(rovX, rovY)
# plt.xlim([0, imWidth])
# plt.ylim([0, imHeight])
# plt.show()


####
