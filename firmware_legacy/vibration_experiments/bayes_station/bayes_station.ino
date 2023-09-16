#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>
 
RF24 radio(2, 10);               // nRF24L01 (CE,CSN)

RF24Network network(radio);     // Network uses that radio
const uint16_t this_rov = 00;  // Address of our node in Octal format (04, 031, etc)
char receivedChar;
char rx_byte = 0;

struct message_t {
  double belief;
  double alpha;
  double beta;
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
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(100);
  radio.setPALevel(RF24_PA_HIGH);
  network.begin(/*node address*/ this_rov);
//  network.update();
//  RF24NetworkHeader header(00);
//  message_t message = {this_rov, 0, -1, -1};
  // bool startExperiment = false;
//  startOk = network.multicast(header, &message, sizeof(message_t), 1);
//  SerialUSB.print("Starting Rovables: ");
//  SerialUSB.println(startOk);
  // unsigned long start = millis();
  // while(!startExperiment) {
  //   unsigned long now = millis();
  //   if (now - start >= 3000) {
  //     network.update();
  //     RF24NetworkHeader header(00);
  //     message_t message = {0, -1, -1};
  //     for (int i = 0; i < 3; i++) {
  //       network.update();
  //       if (network.multicast(header, &message, sizeof(message_t), i)) {
  //         SerialUSB.println("Starting Experiment");
  //         startExperiment = true;
  //       }
  //     }
  //   }
  // }
}
 
void loop(void) {
  network.update();                  // Check the network regularly
  if (network.available()) {      // Is there anything ready for us?
 
    RF24NetworkHeader header;        // If so, grab it and print it out
    message_t message;
    network.read(header, &message, sizeof(message_t));
    SerialUSB.print(header.from_node);
    SerialUSB.print(",");
    SerialUSB.print(message.alpha);
    SerialUSB.print(",");
    SerialUSB.print(message.beta);
    SerialUSB.print(",");
    SerialUSB.println(message.belief);
  }
}
