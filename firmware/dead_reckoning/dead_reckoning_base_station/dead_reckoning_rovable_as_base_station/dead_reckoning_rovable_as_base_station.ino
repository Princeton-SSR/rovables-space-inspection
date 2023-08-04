#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>

// Manually setting clocking frequency for rovable is 2 MHz
RF24 radio(2, 10, 2000000);  // nRF24L01 (CE,CSN) For arduino zero base station use 5, 2

RF24Network network(radio);    // Network uses that radio
const uint16_t base_rov = 00;  // Address of our node in Octal format (04, 031, etc)
const uint16_t mov_rov = 01;


struct message_IMU {
  double ax;
  double ay;
  double az;
  double gx; 
  double gy; 
  double gz; 
  int rcControl; 
  int camControl; 

};

struct message_StartStop{
  int startFlag; 
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
  network.begin(base_rov);
}

void loop(void) {
  network.update();           // Check the network regularly
  if (network.available()) {  // Is there anything ready for us?

    RF24NetworkHeader headerIMU;  // If so, grab it and print it out
    message_IMU messageIMU;
    RF24NetworkHeader header(mov_rov); // Header denots intended recipient
    message_StartStop messageSS; 

    if (SerialUSB.available()){
      int recSSMes = SerialUSB.parseInt(); 
      messageSS = {recSSMes};
      network.write(header, &messageSS, sizeof(message_StartStop)); 
    }

    

    network.read(headerIMU, &messageIMU, sizeof(message_IMU));

    // message.x = sq(message.x);
    // message.y = sq(message.y);
    // message.z = sq(message.z - 9.84);

    SerialUSB.print(messageIMU.ax);
    SerialUSB.print(",");
    SerialUSB.print(messageIMU.ay);
    SerialUSB.print(",");
    SerialUSB.print(messageIMU.az);
    SerialUSB.print(",");
    SerialUSB.print(messageIMU.gx);
    SerialUSB.print(",");
    SerialUSB.print(messageIMU.gy);
    SerialUSB.print(",");
    SerialUSB.print(messageIMU.gz);
    SerialUSB.print(",");
    SerialUSB.print(messageIMU.rcControl); 
    SerialUSB.print(",");
    SerialUSB.print(messageIMU.camControl);
    SerialUSB.print(",");
    SerialUSB.println(millis());
    delay(100);
  }
}
