import numpy as np
import cv2

filename = 'recording.mp4'
  
# This will return video from the first webcam on your computer.
cap = cv2.VideoCapture(1)  
w = int(cap.get(3))
h = int(cap.get(4))
  
print(w, h)
# Define the codec and create VideoWriter object
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))
  
fourcc = cv2.VideoWriter_fourcc(*'MP4V')
out = cv2.VideoWriter(filename, fourcc, 20.0, (w,h))
# loop runs if capturing has been initialized. 
while(True):
    # reads frames from a camera 
    # ret checks return at each frame
    ret, frame = cap.read() 
    for i in range(2000):
        pass
    out.write(frame)
    # The original input frame is shown in the window 
    cv2.imshow('Original', frame)
    # Wait for 'a' key to stop the program 
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
# Close the window / Release webcam
cap.release()
  
# After we release our webcam, we also release the output
out.release() 
  
# De-allocate any associated memory usage 
cv2.destroyAllWindows()