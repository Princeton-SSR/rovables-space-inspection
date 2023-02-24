#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>
 
RF24 radio(5, 2);               // nRF24L01 (CE,CSN)

RF24Network network(radio);     // Network uses that radio
const uint16_t this_rov = 00;  // Address of our node in Octal format (04, 031, etc)

struct message_accel {
  double x;
  double y;
  double z;
};


void setup(void) {
  SerialUSB.begin(115200);
  while (!SerialUSB) {
    // some boards need this because of native USB capability
  }
//  while (SerialUSB.read() != 'y') // Wait until receive start from Serial
//  {
//     //don't do anything
//  }
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
  network.update();                  // Check the network regularly
  if (network.available()) {      // Is there anything ready for us?
 
    RF24NetworkHeader header;        // If so, grab it and print it out
    message_accel message;
    network.read(header, &message, sizeof(message_accel));
    Serial.print(message.x);
    Serial.print(",");
    Serial.print(message.y);
    Serial.print(",");
    Serial.println(message.z);
  } 
}
