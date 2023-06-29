#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RF24Network.h>
#include <RF24.h>

#define  M1_DIR   A2 
#define  M1_PWM   9 
#define  M2_PWM   4 
#define  M2_DIR   A1 
#define ENC1_INT  3  
#define ENC2_INT  7
#define ENC1_LED  8
#define ENC2_LED  6

struct message_IMU {
  double ax;
  double ay;
  double az;
  double gx; 
  double gy; 
  double gz; 

};

// All the motions 
void runMotor(int rightSpeed, int leftSpeed) {
  analogWrite(M1_PWM, rightSpeed);
  analogWrite(M2_PWM, leftSpeed);
}
void moveForward(int rightSpeed, int leftSpeed) {
  digitalWrite(M1_DIR, 0);
  digitalWrite(M2_DIR, 1);
  
  runMotor(rightSpeed, leftSpeed);
}
void moveBackward() {
  digitalWrite(M1_DIR, 1);
  digitalWrite(M2_DIR, 0);

  runMotor(75, 75);
}
void turnRight(){ 
  digitalWrite(M1_DIR, 1);
  digitalWrite(M2_DIR, 1);

  runMotor(75, 75);
}
void turnLeft(){ 
  digitalWrite(M1_DIR, 0);
  digitalWrite(M2_DIR, 0);

  runMotor(75, 75);
}




const uint16_t this_rov = 01; 
const uint16_t rec_rov = 00;

Adafruit_MPU6050 mpu;

// Manually setting clocking frequency for rovable is 2 MHz
RF24 radio(2, 10, 2000000);   // nRF24L01 (CE,CSN)
RF24Network network(radio);
sensors_event_t a, g, temp;

void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(115200);

  SerialUSB.println("Beginning Rovables Functional Check. . .");


  SerialUSB.println("Checking MPU6050. . .");
  if (!mpu.begin()) {
    SerialUSB.println("Failed to intialize MPU6050 chip");
    while(1) {
      delay(10);
    }
  }
  SerialUSB.println("MPU6050 Responsive!");
  SerialUSB.println("------------------------------------------");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  
  SerialUSB.println("Checking NRF24. . .");
  if (!radio.begin()) {
    SerialUSB.println("Failed to initialize NRF24 chip");
    while(1) {
      delay(10);
    }  // hold in infinite loop
  }

  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(100);
  network.begin(this_rov);

  SerialUSB.println("NRF24 Responsive!");
  SerialUSB.println("------------------------------------------");

//  SerialUSB.println("Turning Rovable Counter-clockwise!");
//  digitalWrite(M1_DIR, 0);
//  digitalWrite(M2_DIR, 0);  
//  analogWrite(M1_PWM, 75);
//  analogWrite(M2_PWM, 75);

}
void loop() {
  network.update();
  mpu.getEvent(&a, &g, &temp);
  RF24NetworkHeader header(rec_rov); // Header denots intended recipient
  message_IMU message = {a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z};
  moveForward(70, 70); 
  if (network.write(header, &message, sizeof(message_IMU))) {
    SerialUSB.println("Message Sent");
    delay(10);
  } else {
    SerialUSB.println("Message Failed to Send!");
  }
}
