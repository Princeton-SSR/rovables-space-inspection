/*
This file implements the Bayes Bot algorithm onto the Rovables platform
*/
#include <VL53L1X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RF24Network.h>
#include <RF24.h>
// #include "movement.h"
// #include "beta_infer.h"

#define  M1_DIR   A2 
#define  M1_PWM   9 
#define  M2_PWM   4 
#define  M2_DIR   A1 
#define ENC1_INT  3  
#define ENC2_INT  7
#define ENC1_LED  8
#define ENC2_LED  6
// 7 ->01
// 19 -> 02
// C -> 03
// A -> 04

//Hardware devices
Adafruit_MPU6050 mpu;
VL53L1X tof_left;
VL53L1X tof_right;
RF24 radio(2, 10);   // nRF24L01 (CE,CSN)
RF24Network network(radio);

const uint16_t this_rov = 04; //{00, 01, 02, 03, 04};
const int robot_num = 0;
const uint16_t base = 00;

//Algorithm Parameters
int num_robots = 4;
double alpha = 0;
double beta = 0;
uint32_t tau = 500;
unsigned long last_time_sent = 0; 
bool start = false;
double rov_belief = 0.5;
double p_c = 0.95;
int d_f = 1;
bool u_plus = 0;
bool ok;
int v = 1;

// Vibration detection parameters
double vibThresh = 0.1;
int caThresh = 100;
int forwardUpper = 564;
int pauseTime = 2000; 
const int numReadings = 10;
int vReadings [numReadings];
int readIndex  = 0;
long total  = 0;

uint8_t encoder1Counter = 0;
uint8_t encoder2Counter = 0;

//Structures for MPU6050 
sensors_event_t a, g, temp;

float accelx;
float accely;
float accelz;

const int numV = 200;
int inspectReadings [numV];
int inspectIndex = 0;

struct message_b {
  double belief;
  double alpha;
  double beta;
};

struct message_r {
  int v_prime;
};

long smooth() { /* function smooth */
  //Perform average on sensor readings
  long average;
  // subtract the last reading:
  total = total - vReadings[readIndex];
  // read the sensor:
  mpu.getEvent(&a, &g, &temp);
  vReadings[readIndex] =  a.acceleration.z;
  // add value to total:
  total = total + vReadings[readIndex];
  // handle index
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  // calculate the average:
  average = total / numReadings;

  return average;
}



/*
 * zlib License
 *
 * Regularized Incomplete Beta Function
 *
 * Copyright (c) 2016, 2017 Lewis Van Winkle
 * http://CodePlea.com
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgement in the product documentation would be
 *    appreciated but is nBE0ot required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#define STOP 1.0e-8
#define TINY 1.0e-30

double incbeta(double a, double b, double x) {
    if (x < 0.0 || x > 1.0) return 1.0/0.0;

    /*The continued fraction converges nicely for x < (a+1)/(a+b+2)*/
    if (x > (a+1.0)/(a+b+2.0)) {
        return (1.0-incbeta(b,a,1.0-x)); /*Use the fact that beta is symmetrical.*/
    }

    /*Find the first part before the continued fraction.*/
    const double lbeta_ab = lgamma(a)+lgamma(b)-lgamma(a+b);
    const double front = exp(log(x)*a+log(1.0-x)*b-lbeta_ab) / a;

    /*Use Lentz's algorithm to evaluate the continued fraction.*/
    double f = 1.0, c = 1.0, d = 0.0;

    int i, m;
    for (i = 0; i <= 200; ++i) {
        m = i/2;

        double numerator;
        if (i == 0) {
            numerator = 1.0; /*First numerator is 1.0.*/
        } else if (i % 2 == 0) {
            numerator = (m*(b-m)*x)/((a+2.0*m-1.0)*(a+2.0*m)); /*Even term.*/
        } else {
            numerator = -((a+m)*(a+b+m)*x)/((a+2.0*m)*(a+2.0*m+1)); /*Odd term.*/
        }

        /*Do an iteration of Lentz's algorithm.*/
        d = 1.0 + numerator * d;
        if (fabs(d) < TINY) d = TINY;
        d = 1.0 / d;

        c = 1.0 + numerator / c;
        if (fabs(c) < TINY) c = TINY;

        const double cd = c*d;
        f *= cd;

        /*Check for stop.*/
        if (fabs(1.0-cd) < STOP) {
            return front * (f-1.0);
        }
    }

    return 1.0/0.0; /*Needed more loops, did not converge.*/
}

//Turns the motor on for a duration of "dur" milliseconds
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
      // if (abs(distance_r - distance_l) < 10) {
      //   moveBackward(300);
      // } else {
      turnLeft(500);
      // }
    }
    if (distance_l <= caThresh) {
      // if (abs(distance_r - distance_l) < 10) {
      //   moveBackward(300);
      // } else {
      turnRight(500);
      // }
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
  moveForward(random(0, forwardUpper));
  delay(10);
  int dir = random(1,3);
  if (dir == 1) {
    ok = send_Base();
    turnLeft(random(600,1000));
  } else if (dir == 2) {
    ok = send_Base();
    turnRight(random(600,1000));
  }
  delay(10);
}

bool inspect() {
  inspectIndex = 0;
  // When inspecting we must stop the motor.
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  unsigned long inspect_start = millis();
  while(true){
    unsigned long now = millis();
    if (now - inspect_start >= pauseTime) {
      break;
    }

    mpu.getEvent(&a, &g, &temp);
    inspectReadings[inspectIndex] =  a.acceleration.z - smooth();

    inspectIndex = inspectIndex + 1;
    if (inspectIndex >= numV) {
      inspectIndex = 0;
    }
  }

  int sum = 0;
  for (int i = 0; i < numV; i++) {
    sum = sum + abs(inspectReadings[i]);
  }
  SerialUSB.println((double) sum / (double) numV);
  if (((double) sum / (double) numV) > vibThresh) {
    SerialUSB.println("Send 1");
    return 1;
  } else {
    SerialUSB.println("Send 0");
    return 0;
  }
}

// Receives traffic on Rovable to Rovable communication
void handle_Rov() {
  network.update();
  message_r message;
  RF24NetworkHeader incomingHeader;
  if (network.available()) {
    network.read(incomingHeader, &message, sizeof(message_r));
    beta = beta + (1 - message.v_prime);
    alpha = alpha + message.v_prime; 
  }
}

// Send message to base station
bool send_Base() {
  network.update();
  RF24NetworkHeader header(00);
  message_b message = {rov_belief, alpha, beta};
  return network.write(header, &message, sizeof(message_b));
}

// Send message to other rovable
void send_Rov(int v) {
  network.update();
  RF24NetworkHeader header(this_rov);
  message_r message = {v};
    // network.multicast(header, &message, sizeof(message_r), i);
  network.multicast(header, &message, sizeof(message_r));
}

bool checkStart() {
  network.update();
  if (network.available()) {
    // SerialUSB.println("read Network");
    RF24NetworkHeader header;
    message_b initM;
    network.read(header, &initM, sizeof(message_b));
    if (initM.alpha == -1 && initM.beta == -1) {
      RF24NetworkHeader header(00);
      message_b message = {0, -1, -1};
      network.multicast(header, &message, sizeof(message_b), 1);
      return true;
    }
  }
  else {
    SerialUSB.println("No Network Start!");
  }
  return false;
}

void setup() {
  randomSeed(this_rov);
  SerialUSB.begin(115200);

  SerialUSB.println("Starting Rovable");

  Wire.begin();
  Wire.setClock(400000);
  pinMode(M1_DIR, OUTPUT); 
  pinMode(M1_PWM, OUTPUT); 
  pinMode(M2_PWM, OUTPUT); 
  pinMode(M2_DIR, OUTPUT);
  pinMode(ENC1_INT, INPUT); 
  pinMode(ENC2_INT, INPUT); 
  pinMode(ENC1_LED, OUTPUT);
  pinMode(ENC2_LED, OUTPUT);

  // Disable left time of flight sensor to set address of right
  // The toggle is done through the RX pin
  pinMode(0, OUTPUT);
  digitalWrite(0, LOW);

  tof_right.setTimeout(500);
  if (!tof_right.init()) {
    while(1);
  }
  tof_right.setAddress(0x2F);
  tof_right.startContinuous(50);
  tof_right.setDistanceMode(VL53L1X::Short);
  tof_right.setMeasurementTimingBudget(20000);

  //Enable the left time of flight back to set address
  pinMode(0, INPUT);
  delay(10);

  
  tof_left.setTimeout(500);
  if (!tof_left.init()) {
    while (1);
  }
  tof_left.setAddress(0x2A);
  tof_left.startContinuous(50);
  tof_left.setDistanceMode(VL53L1X::Short);
  tof_left.setMeasurementTimingBudget(20000);

  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  if (!radio.begin()) {
      while(1) {
        SerialUSB.println("Radio failed to start");
      }
  }
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(100);
  network.begin(/*node address*/ this_rov);

  // bool startExperiment = false;

  // while(!startExperiment) {
  //   if (checkStart()) {
  //     startExperiment = true;
  //   }
  // }
}

void loop() {
  ok = send_Base();

  doRandomWalk();

  unsigned long now = millis();
  //Inspect the vibration every interval
  if (now - last_time_sent >= tau) {
    last_time_sent = now;
    v = inspect();
    beta = beta + (1 - v);
    alpha = alpha + v;
  }
  //Read message from other rovables
  handle_Rov();

  rov_belief = incbeta(alpha, beta, 0.5);
  ok = send_Base(); // Send current distribution to base

  if (rov_belief > p_c) {
    d_f = 1;
  }
  if ((1 - rov_belief) > p_c) {
    d_f = 0;
  }

  if (d_f != -1 & u_plus == 1) {
    send_Rov(d_f);
  } else {
    send_Rov(v);
  }

  // THIS IS JUST TEST CODE FOR COMMUNICATION
  // unsigned long now = millis();
  // if (now - last_time_sent >= tau) {
  //   last_time_sent = now;
  //   beta = beta;
  //   alpha = alpha + 1;
  //   send_Base();
  //   send_Rov(0);
  //   handle_Rov();
  // }
}
