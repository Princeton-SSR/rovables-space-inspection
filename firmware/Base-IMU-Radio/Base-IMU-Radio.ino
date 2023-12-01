#include <RF24.h>
#include <RF24Network.h>
#include <ral-comp-radio-messages.h>
#include <ral-comp-radio.h>

Comms comm(00); 

void setup(void) {
  SerialUSB.begin(115200);
  while (!SerialUSB){}; 
  comm.setupRadio();
  comm.setupNetwork();
}

void loop(void) {
  message_IMURC receivedMessage = comm.recMessage<message_IMURC>();
  
  // Check if a valid message was received
  if (receivedMessage.ax != 0) {
    SerialUSB.print(receivedMessage.ax);
    SerialUSB.print(",");
    SerialUSB.print(receivedMessage.ay);
    SerialUSB.print(",");
    SerialUSB.print(receivedMessage.az);
    SerialUSB.print(",");
    SerialUSB.print(receivedMessage.gx);
    SerialUSB.print(",");
    SerialUSB.print(receivedMessage.gy);
    SerialUSB.print(",");
    SerialUSB.print(receivedMessage.gz);
    SerialUSB.print(",");
    SerialUSB.print(receivedMessage.rcControl);
    SerialUSB.println("");
    // You can perform further actions with the received message here
  }
  // Delay or perform other tasks as needed
  delay(100);
}

