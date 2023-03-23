#include <RF24.h>
#include <RF24Network.h>

char receivedChar;

const uint16_t this_rov = 01; 

struct message_dir {
  int dir;
};
RF24 radio(5, 2);   // nRF24L01 (CE,CSN)
RF24Network network(radio);

void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(115200);
  SerialUSB.println("Starting Remote Controller");
  if (!radio.begin()) {
    SerialUSB.println("Failed to initialize NRF24 chip");
    while(1) {
      delay(10);
    }  // hold in infinite loop
  }
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(100);
  network.begin(this_rov);
  
  SerialUSB.println("Base Station Ready");
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if (SerialUSB.available() > 0) {
    receivedChar = SerialUSB.read();
//    SerialUSB.println(receivedChar - '0');
    network.update(); 
    message_dir message = {receivedChar - '0'};
    RF24NetworkHeader header(00);
    SerialUSB.print("Sending Direction: ");
    SerialUSB.println(message.dir);

    if (network.write(header, &message, sizeof(message_dir))) {
      SerialUSB.println("Message Sent");
    } else {
      SerialUSB.println("Message Failed to Send");
    }
  }
}
