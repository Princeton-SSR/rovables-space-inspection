import serial
import time

# Replace with the actual serial port name and baud rate
serial_port = 'COM13'  # Update with the correct port name
baud_rate = 115200

# Open the serial connection
ser = serial.Serial(serial_port, baud_rate)

# Wait for a moment to let the Arduino reset after opening the serial port
time.sleep(2)

# Send a message to the Arduino to turn on
message = 1
ser.write(str(message).encode())  # Convert message to bytes and send

time.sleep(5)

# Send a message to the Arduino to turn off
message = 0
ser.write(str(message).encode())  # Convert message to bytes and send
 


# Close the serial connection
ser.close()