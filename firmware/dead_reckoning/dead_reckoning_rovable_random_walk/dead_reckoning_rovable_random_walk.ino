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
  int rcControl; 
  int camControl; 

};

struct message_StartStop{
  int startFlag; 
};

// All the motions 
void runMotor(int rightSpeed, int leftSpeed) {
  analogWrite(M1_PWM, rightSpeed);
  analogWrite(M2_PWM, leftSpeed);
}
int moveForward(int rightSpeed, int leftSpeed) {
  digitalWrite(M1_DIR, 0);
  digitalWrite(M2_DIR, 1);
  
  runMotor(rightSpeed, leftSpeed);
  return 1;
}
int moveBackward(int rightSpeed, int leftSpeed) {
  digitalWrite(M1_DIR, 1);
  digitalWrite(M2_DIR, 0);

  runMotor(rightSpeed, leftSpeed);
  return 2; 
}
int turnRight(int rightSpeed, int leftSpeed){ 
  digitalWrite(M1_DIR, 1);
  digitalWrite(M2_DIR, 1);

  runMotor(rightSpeed, leftSpeed);
  return 3; 
}
int turnLeft(int rightSpeed, int leftSpeed){ 
  digitalWrite(M1_DIR, 0);
  digitalWrite(M2_DIR, 0);

  runMotor(rightSpeed, leftSpeed);
  return 4; 
}


int doRandomWalk() {
  int dir = random(1,10);
  //int dir = 3; 
  
  int rcControl; 
  if (dir == 1) {
    rcControl = turnLeft(75, 75);
    delay(random(500,1000)); 
  } else if (dir == 2) {
    rcControl = turnRight(75,75);
    delay(random(500,1000)); 
  }
  else{
    rcControl = moveForward(75, 75);
    delay(random(1000,2000)); 
  }
  return rcControl; 
  
}



const uint16_t mov_rov = 01; 
const uint16_t base_rov = 00;
int rcControl = 0; // By default the robot is stationary 
int camControl = 0; 

Adafruit_MPU6050 mpu;

// Manually setting clocking frequency for rovable is 2 MHz
RF24 radio(2, 10, 2000000);   // nRF24L01 (CE,CSN)
RF24Network network(radio);
sensors_event_t a, g, temp;
int iteration = 0; 
int dir = 3; 
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
  network.begin(mov_rov);

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
  RF24NetworkHeader header(base_rov); // Header denots intended recipient
  //rcControl = moveForward(71, 75); 
  //rcControl = turnRight(75, 75); 
  SerialUSB.println("--------");
  SerialUSB.println(iteration);
  SerialUSB.println(dir);
  if (iteration%20 == 0){
    dir = random(1,7);    
  }
  
  if (dir == 1) {
    rcControl = turnLeft(75, 75);
  } else if (dir == 2) {
    rcControl = turnRight(75,75);
  }
  else{
    rcControl = moveForward(75, 75);
  }
  message_IMU message = {a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z, rcControl, camControl};
  camControl = 0; 
  if (network.write(header, &message, sizeof(message_IMU))) {
    SerialUSB.println("Message Sent");
    delay(10);
  } else {
    SerialUSB.println("Message Failed to Send!");
  }
  
  if (iteration%50 == 0){
    runMotor(0,0);
    delay(1000); 
    camControl = 1; 
  }
  iteration++;
  
  
}