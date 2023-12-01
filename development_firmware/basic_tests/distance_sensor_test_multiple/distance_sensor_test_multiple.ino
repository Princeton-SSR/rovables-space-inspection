#include <Wire.h>
#include <VL53L1X.h>

#define  M1_DIR   A2 
#define  M1_PWM   9 
#define  M2_PWM   4 
#define  M2_DIR   A1 
#define ENC1_INT  3  
#define ENC2_INT  7
#define ENC1_LED  8
#define ENC2_LED  6
#define TX_GPIO 1
#define RX_GPIO 0

int caThresh = 50;

VL53L1X tof_right;
VL53L1X tof_left;

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
  for (int i=0; i < dur; i++){
    int16_t distance_r;
    int16_t distance_l;
    tof_left.read();
    tof_right.read();
    distance_r = tof_right.ranging_data.range_mm;
    distance_l = tof_left.ranging_data.range_mm;
    if (distance_r <= caThresh) {
      if (abs(distance_r - distance_l) < 10) {
        moveBackward(300);
      } else {
        turnLeft(300);
      }
    }
    if (distance_l <= caThresh) {
      if (abs(distance_r - distance_l) < 10) {
        moveBackward(300);
      } else {
        turnRight(300);
      }
    }
    else {
      // Go back to moving forward
      digitalWrite(M1_DIR, 0);
      digitalWrite(M2_DIR, 1);
      analogWrite(M1_PWM, 75);
      analogWrite(M2_PWM, 75);
      delay(1);
    }
  }
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

void doRandomWalk() {
  moveForward(random(600, 1001));
  delay(10);
  int dir = random(1,3);
  if (dir == 1) {
    turnLeft(random(600,1501));
  } else if (dir == 2) {
    turnRight(random(600,1501));
  }
  delay(10);
}


void setup() {
  // // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  // Disable/reset all sensors by driving their XSHUT pins low.
  pinMode(1, OUTPUT);
  digitalWrite(1, LOW);

  // Start middle tof sensor
  tof_right.setTimeout(500);
  if (!tof_right.init()) {
    while(1);
  }
  tof_right.setAddress(0x2F);
  tof_right.startContinuous(50);
  tof_right.setDistanceMode(VL53L1X::Short);
  tof_right.setMeasurementTimingBudget(20000);
  
  //Enable the left tof
  pinMode(1, INPUT);
  delay(10);

  tof_left.setTimeout(500);
  if (!tof_left.init())
  {
    while (1);
  }

  // Each sensor must have its address changed to a unique value other than
  // the default of 0x29 (except for the last one, which could be left at
  // the default). To make it simple, we'll just count up from 0x2A.
  tof_left.setAddress(0x2A);
  tof_left.setDistanceMode(VL53L1X::Short);
  tof_left.setMeasurementTimingBudget(20000);
  tof_left.startContinuous(50);
}

void loop() {
  // put your main code here, to run repeatedly:
  moveForward(100);
}
