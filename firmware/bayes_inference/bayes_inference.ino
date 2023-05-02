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
// C ->01
// A -> 02
// 7 -> 03
// 6 -> 04
Adafruit_MPU6050 mpu;
VL53L1X tof;
RF24 radio(2, 10);   // nRF24L01 (CE,CSN)
RF24Network network(radio);
const uint16_t this_rov = 02; //{00, 01, 02, 03, 04};
const uint16_t base = 00;
unsigned long last_time_sent = 0; 
bool start = false;
double rov_belief = 0.5;
double p_c = 0.95;
int d_f = 1;
bool u_plus = 0;
bool ok;
int v = 1;
double vibThresh = 0.1;
int caThresh = 100;
int forwardUpper = 564*8;
int pauseTime = 2000; 

double alpha = 0;
double beta = 0;
uint32_t tau = 200;
uint8_t encoder1Counter = 0;
uint8_t encoder2Counter = 0;
sensors_event_t a, g, temp;

float accelx;
float accely;
float accelz;



// int window = 5;
// float mWindow[5] = {0};
// float mAV = 0;
// int movAvgC = 0;
const int numReadings = 10;
int vReadings [numReadings];
int readIndex  = 0;
long total  = 0;

const int numV = 200;
int inspectReadings [numV];
int inspectIndex = 0;

// float xOffset = 0.19;
// float yOffset = 0.24;
// float zOffset = 10.53;
//ROV01: 0.19, 0.24, 10.53 
//ROV02: 0.32, 0.07, 9.5
//ROV03: 0.47, 0.01, 9.86
//ROV04: 1.03, 0.20, 9.98


struct message_t {
  double belief;
  double alpha;
  double beta;
};

struct message_r {
  uint16_t id;
  int v_prime;
};

long smooth() { /* function smooth */
  ////Perform average on sensor readings
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
    int16_t distance;
    tof.read();
    distance = tof.ranging_data.range_mm;
    if (distance <= caThresh) {
      ok = send_Base();
      turnLeft(1200);
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
  //When inspecting we must stop the motor.
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
  // mAV = 0;
  // mWindow[movAvgC] = reading;
  // movAvgC = movAvgC + 1;
  // if (movAvgC > window - 1) {
  //   movAvgC = 0;
  // }
  // for (int j = 0; j < window; j++) {
  //   mAV = mAV + mWindow[j];
  // }
  // mAV = mAV / window;
  // if (abs(reading) > vibThresh*mAV) {
  //   return 1;
  // } else {
  //   return 0;
  // }
}

void handle_R() {
  message_r message;
  RF24NetworkHeader header;
  network.read(header, &message, sizeof(message_r));
  // if (header.from_node != this_rov || header.from_node != 00) {
    // if (message.index_prime != last_index) {
    // beta = beta + (1 - v);
    // alpha = alpha + v;
    // last_index = message.index_prime;
  beta = beta + (1 - message.v_prime);
  alpha = alpha + message.v_prime; 
  // SerialUSB.print("Header: ");
  // SerialUSB.print(header.from_node);
  // SerialUSB.print(", ");
  // SerialUSB.print(alpha);
  // SerialUSB.print(", ");
  // SerialUSB.println(beta);
    // }
  // }
}

bool send_Base() {
  network.update();
  RF24NetworkHeader header(00);
  message_t message = {rov_belief, alpha, beta};
  return network.write(header, &message, sizeof(message_t));
}

void send_Rov(int v) {
  network.update();
  RF24NetworkHeader header(this_rov);
  message_r message = {this_rov, v};
  for (int i = 0; i < 3; i++) {
    network.multicast(header, &message, sizeof(message_r), i);
  // return network.write(header, &message, sizeof(message_t));
  }
}

bool checkStart() {
  network.update();
  if (network.available()) {
    // SerialUSB.println("read Network");
    RF24NetworkHeader header;
    message_t initM;
    network.read(header, &initM, sizeof(message_t));
    if (initM.alpha == -1 && initM.beta == -1) {
      RF24NetworkHeader header(00);
      message_t message = {0, -1, -1};
      network.multicast(header, &message, sizeof(message_t), 1);
      return true;
    }
  }
  else {
    SerialUSB.println("No Network Start!");
  }
  return false;
}

void setup() {
  SerialUSB.begin(115200);

  // while (!SerialUSB) {
  //   // some boards need this because of native USB capability
  // }
  // Serial1.begin(9600);
  // Serial1.println("Starting Rovable");
  // SerialUSB.println("Starting Rovable");
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

  randomSeed(this_rov);

  if (!tof.init()) {
      SerialUSB.println("Time of Flight failed to start");
      while(1);
  }
  tof.setTimeout(500);

  tof.setDistanceMode(VL53L1X::Short);
  tof.setMeasurementTimingBudget(20000);

  tof.startContinuous(50);

  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  // for (int j = 0; j < window; j++) {
  //   mpu.getEvent(&a, &g, &temp);
  //   accelx = a.acceleration.x;
  //   accely = a.acceleration.y;
  //   accelz = a.acceleration.z;
  //   double reading = accelz;
  //   mWindow[j] = reading;
  // }
  if (!radio.begin()) {
      while(1) {
        SerialUSB.println("Radio failed to start");
      }
  }
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
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
  // Random walk handles collision avoidance.
  // doRandomWalk();
  unsigned long now = millis();
  //Inspect the vibration every interval
  if (now - last_time_sent >= tau) {
    last_time_sent = now;
    v = inspect();
    beta = beta + (1 - v);
    alpha = alpha + v;
    // SerialUSB.print("Alpha: ");
    // SerialUSB.print(alpha);
    // SerialUSB.print("Beta: ");
    // SerialUSB.println(beta);
  }
  network.update();
  if(network.available()) {
    //Read message from other rovables
    // SerialUSB.println("Found Message");
    handle_R();
  }
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
}
