#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>

// Manually setting clocking frequency for rovable is 2 MHz
RF24 radio(2, 10, 2000000);  // nRF24L01 (CE,CSN) For arduino zero base station use 5, 2

RF24Network network(radio);    // Network uses that radio
const uint16_t this_rov = 00;  // Address of our node in Octal format (04, 031, etc)

struct message_IMU {
  double ax;
  double ay;
  double az;
  double gx; 
  double gy; 
  double gz; 
  int rcControl; 

};

void setup(void) {
  SerialUSB.begin(115200);
  while (!SerialUSB){}; 
  SerialUSB.println("Starting Bayes Station");

  if (!radio.begin()) {
    SerialUSB.println(F("Radio hardware not responding!"));
    while (1) {
      // hold in infinite loop
    }
  }
  radio.setChannel(100);
  radio.setPALevel(RF24_PA_HIGH);
  network.begin(this_rov);
}

void loop(void) {
  network.update();           // Check the network regularly
  if (network.available()) {  // Is there anything ready for us?

    RF24NetworkHeader header;  // If so, grab it and print it out
    message_IMU message;
    network.read(header, &message, sizeof(message_IMU));

    // message.x = sq(message.x);
    // message.y = sq(message.y);
    // message.z = sq(message.z - 9.84);

    SerialUSB.print(message.ax);
    SerialUSB.print(",");
    SerialUSB.print(message.ay);
    SerialUSB.print(",");
    SerialUSB.print(message.az);
    SerialUSB.print(",");
    SerialUSB.print(message.gx);
    SerialUSB.print(",");
    SerialUSB.print(message.gy);
    SerialUSB.print(",");
    SerialUSB.print(message.gz);
    SerialUSB.print(",");
    SerialUSB.print(message.rcControl); 
    SerialUSB.print(",");
    SerialUSB.println(millis());
    delay(100);
  }
}
