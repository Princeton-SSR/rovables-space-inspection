from serial import Serial
import time

ser = Serial('/dev/tty.usbmodem1101')  # open serial port
print("Reading Data from Serial Port: ", ser.name)

T = 60
accelFile = []

startTime = time.time()
while (time.time() - startTime < T):
  #if ser.in_waiting:
      #     read until EOL   decode        remove    split commas 
  accel_val = ser.readline().decode('utf-8').rstrip().split(",")
  if ('ovf' in accel_val):
      print("Encountered OVF Value. Continuing Experiment...")
  if ('' not in accel_val):
      float_accel = [float(i) for i in accel_val]
      if(len(float_accel) == 3):
          print(float_accel)
          accelFile.append(float_accel)

with open('accel_data_2_2.txt', mode='a') as f:
  f.write(f"{accelFile}\n")
