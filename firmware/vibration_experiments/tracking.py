import cv2
import numpy as np
import matplotlib.pyplot as plt


filename = 'test.mp4'

#pass in file name to save locally
#pass in integer to choose camera. 0 is usually the laptop webcam.
cam = cv2.VideoCapture(1)

rovX = []
rovY = []

imWidth = cam.get(3)
imHeight = cam.get(4)

while (cam.isOpened()):
    ret, frame = cam.read()
    if ret == True:

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        #Define HSV Values
        # lower = np.array([50, 100, 100])
        # upper = np.array([95, 255, 255])
        lower = np.array([50, 60, 100])
        upper = np.array([95, 140, 255])

        mask = cv2.inRange(hsv, lower, upper)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        for c in contours:
            area = cv2.contourArea(c)
            if (area > 2.5):
                brx,bry,brw,brh = cv2.boundingRect(c)
                x = brx+brw/2
                y = bry+brh/2
                cv2.circle(frame, (int(x), int(y)), 15, (255, 0, 0), -1)
                cv2.putText(frame, "Rovable", (int(x) - 20, int(y) - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                rovX.append(x)
                rovY.append(y)
        #input("Press Enter to continue...")

        cv2.drawContours(frame, contours, -1, (0,255,0), 3)
        cv2.imshow('Mask', mask)
        cv2.imshow('Frame', frame)
        

        if cv2.waitKey(25) & 0xFF ==ord('q'):
            break

    # else:
    #     break

cam.release()
cv2.destroyAllWindows()

# plt.plot(rovX, rovY)
# plt.xlim([0, imWidth])
# plt.ylim([0, imHeight])
# plt.show()


####
