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


uint8_t encoder1Counter = 0;
uint8_t encoder2Counter = 0;


void runMotor(int dur) {
  analogWrite(M1_PWM, 75);
  analogWrite(M2_PWM, 75);
  delay(dur);
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
}

void moveForward(int dur) {
  digitalWrite(M1_DIR, 0);
  digitalWrite(M2_DIR, 1);
  
  runMotor(dur);
}

void moveBackward(int dur) {
  digitalWrite(M1_DIR, 1);
  digitalWrite(M2_DIR, 0);

  runMotor(dur);
}

void turnRight(int dur){ 
  digitalWrite(M1_DIR, 1);
  digitalWrite(M2_DIR, 1);

  runMotor(dur);
}

void turnLeft(int dur){ 
  digitalWrite(M1_DIR, 0);
  digitalWrite(M2_DIR, 0);

  runMotor(dur);
}

void moveForwardSkew(int rightSpeed, int leftSpeed, int dur){
  analogWrite(M1_PWM, rightSpeed);
  analogWrite(M2_PWM, leftSpeed);
  digitalWrite(M1_DIR, 0);
  digitalWrite(M2_DIR, 1);
  delay(dur);
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);  
}

void setup() {
  SerialUSB.begin(115200);
  SerialUSB.println("Starting Rovable");
  pinMode(M1_DIR, OUTPUT); 
  pinMode(M1_PWM, OUTPUT); 
  pinMode(M2_PWM, OUTPUT); 
  pinMode(M2_DIR, OUTPUT);
  pinMode(ENC1_INT, INPUT); 
  pinMode(ENC2_INT, INPUT); 
  pinMode(ENC1_LED, OUTPUT);
  pinMode(ENC2_LED, OUTPUT);
 
}

void loop() {
moveForwardSkew(80,  75, 1000);
delay(100);
}
