
#include <RF24.h>
#include <RF24Network.h>


#define  M1_DIR   A2 
#define  M1_PWM   9 
#define  M2_PWM   4 
#define  M2_DIR   A1 
#define ENC1_INT  3  
#define ENC2_INT  7
#define ENC1_LED  8
#define ENC2_LED  6

RF24 radio(2, 10, 2000000);   // nRF24L01 (CE,CSN)
RF24Network network(radio);
const uint16_t this_rov = 00;


struct message_dir {
  int dir;
};

void runMotor() {
  analogWrite(M1_PWM, 75);
  analogWrite(M2_PWM, 75);
}

void moveForward() {
  digitalWrite(M1_DIR, 0);
  digitalWrite(M2_DIR, 1);
  
  runMotor();
}

void moveBackward() {
  digitalWrite(M1_DIR, 1);
  digitalWrite(M2_DIR, 0);

  runMotor();
}

void turnRight(){ 
  digitalWrite(M1_DIR, 1);
  digitalWrite(M2_DIR, 1);

  runMotor();
}

void turnLeft(){ 
  digitalWrite(M1_DIR, 0);
  digitalWrite(M2_DIR, 0);

  runMotor();
}

void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(115200);
  SerialUSB.println("Beginning Rovables Remote Controller. . .");
  if (!radio.begin()) {
    while (1) {
      // hold in infinite loop
      SerialUSB.println(F("Radio hardware not responding!"));
    }
  }
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(100);
  radio.setPALevel(RF24_PA_HIGH);
  network.begin(this_rov);
}

void loop() {
  // put your main code here, to run repeatedly:
  network.update();                  // Check the network regularly
  if (network.available()) {      // Is there anything ready for us?
    RF24NetworkHeader header;        // If so, grab it and print it out
    message_dir message;
    network.read(header, &message, sizeof(message_dir));

    if (message.dir == 71) {
      moveForward();
    }
    if (message.dir == -16) {
      analogWrite(M1_PWM, 0);
      analogWrite(M2_PWM, 0);
    }
    if (message.dir == 49) {
      turnLeft();
    }
    if (message.dir == 52) {
      turnRight();
    }
    if (message.dir == 67) {
      moveBackward();
    }
  } else {
    SerialUSB.println("Waiting for Packets on Network!");
  }
}
