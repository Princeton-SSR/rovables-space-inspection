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

Adafruit_MPU6050 mpu;
RF24 radio(2, 10);   // nRF24L01 (CE,CSN)
RF24Network network(radio);
const uint16_t this_rov = 01; //{00, 01, 02, 03, 04};
const uint16_t base = 00;
const unsigned long interval = 500; //ms
unsigned long last_time_sent; 
bool simStart = false;
double rov_belief = 0;
double p_c = 0.5;
int d_f = 1;
int rov_index = 0;
bool u_plus = 0;
bool ok;
int last_index = -1;
int v = 0;
double vibThresh = 1.25;

double alpha = 0;
double beta = 0;
uint32_t tau = 3000;
uint8_t encoder1Counter = 0;
uint8_t encoder2Counter = 0;
sensors_event_t a, g, temp;

float accelx;
float accely;
float accelz;
float deady;
float reading;
bool deadlock = false;

int window = 3;
float mWindow[3] = {0};
float mAV = 0;
int movAvgC = 0;

float xOffset = 0.19;
float yOffset = 0.24;
float zOffset = 10.53;
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
  int index_prime;
  int v_prime;
};
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
  delay(50);
  mpu.getEvent(&a, &g, &temp);
  deady = a.acceleration.y - yOffset;
  if (deady < -0.75) {
    deadlock = true;
  }
  delay(dur);
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
}

void moveForward(int dur) {
  ok = send_Base();
  digitalWrite(M1_DIR, 0);
  digitalWrite(M2_DIR, 1);
  
  runMotor(dur);
}

void moveBackward(int dur) {
  ok = send_Base();
  digitalWrite(M1_DIR, 1);
  digitalWrite(M2_DIR, 0);

  runMotor(dur);
}

void turnRight(int dur){ 
  ok = send_Base();
  digitalWrite(M1_DIR, 1);
  digitalWrite(M2_DIR, 1);

  runMotor(dur);
}

void turnLeft(int dur){ 
  ok = send_Base();
  digitalWrite(M1_DIR, 0);
  digitalWrite(M2_DIR, 0);

  runMotor(dur);
}

void doRandomWalk() {
  int dir = random(1,3);
  if (dir == 1) {
    turnLeft(random(600,1501));
  } else if (dir == 2) {
    turnRight(random(600,1501));
  }
  delay(10);
  moveForward(random(600, 2001));
  delay(10);
}

bool inspect() {
  mAV = 0;
  mWindow[movAvgC] = reading;
  movAvgC = movAvgC + 1;
  if (movAvgC > window - 1) {
    movAvgC = 0;
  }
  for (int j = 0; j < window; j++) {
    mAV = mAV + mWindow[j];
  }
  mAV = mAV / window;
  if (abs(reading) > vibThresh*mAV) {
    return 1;
  } else {
    return 0;
  }
}

void handle_R(RF24NetworkHeader& header) {
  message_r message;
  network.read(header, &message, sizeof(message_r));
  if (header.from_node != this_rov || header.from_node != 00) {
    if (message.index_prime != last_index) {
      last_index = message.index_prime;
      beta = beta + message.v_prime;
      alpha = alpha + (1-message.v_prime); 
    }
  }
}

bool send_Base() {
  RF24NetworkHeader header(00);
  message_t message = {rov_belief, alpha, beta};
  return network.write(header, &message, sizeof(message_t));
}

bool send_Rov(int three) {
  RF24NetworkHeader header(this_rov, 'R');
  message_r message = {this_rov, rov_index, three};
  return network.multicast(header, &message, sizeof(message_r), 1);
  //return network.write(header, &message, sizeof(message_t));
}

void checkStart() {
  network.update();
  if (network.available()) {
    RF24NetworkHeader header;
    message_t initM;
    network.read(header, &initM, sizeof(message_t));
    if (initM.alpha == -1 && initM.beta == -1) {
      simStart = true;
    }
  }
  else {
    SerialUSB.println("No Network Start!");
  }
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

  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  for (int j = 0; j < window; j++) {
    mpu.getEvent(&a, &g, &temp);
    accelx = a.acceleration.x;
    accely = a.acceleration.y;
    accelz = a.acceleration.z;
    double reading = accelz;
    mWindow[j] = reading;
  }

  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(100);
  network.begin(/*node address*/ this_rov);
  SerialUSB.println("Setting offset");
  if (this_rov == 01) {
    xOffset = 0.19;
    yOffset = 0.24;
    zOffset = 10.53;
  }

  if (this_rov == 02) {
    SerialUSB.println("Set offset");
    xOffset = 0.32;
    yOffset = 0.07;
    zOffset = 9.5;
  }

  if (this_rov == 03) {
    xOffset = 0.47;
    yOffset = 0.01;
    zOffset = 9.86;
  }

  if (this_rov == 04) {
    xOffset = 1.03;
    yOffset = 0.2;
    zOffset = 9.98;
  }
}

void loop() {
  // Check the sampling rate. 
//  moveForward(1000);
//  delay(100);
//  turnRight(1000);
//  delay(100);
//  turnLeft(1000);
//  delay(100);
//  mpu.getEvent(&a, &g, &temp);
//  accelx = a.acceleration.x - xOffset;
//  accely = a.acceleration.y - yOffset;
//  accelz = a.acceleration.z - zOffset;
//  reading = abs(accelz);
//  //SerialUSB.print("Raw_Combined_Accel:");
//  //SerialUSB.print("X_accel:");
//  SerialUSB.print(accelx);
//  SerialUSB.print(",");
//  //SerialUSB.print("Y_accel:");
//  SerialUSB.print(accely);
//  SerialUSB.print(",");
//  //SerialUSB.print("Z_accel:");
//  SerialUSB.println(accelz);
//  SerialUSB.print(",");
//  SerialUSB.print("Moving_Avg_Threshold:");
//  SerialUSB.println(mAV*vibThresh);
//  SerialUSB.print(",");
//  SerialUSB.print("Detect:");
//  SerialUSB.println(v);
//  if (deadlock) {
//    moveBackward(800);
//    deadlock = false;
//  } else {
//    doRandomWalk();
//    delay(10);
//  }
  doRandomWalk();
//  network.update();
//  if(network.available()) {
//    RF24NetworkHeader header;
//    network.peek(header);
//
//    switch(header.type) { // Check if the message is from the Base Station or another Rovable
//      case 'B':
//        break;
//      case 'R':
//        handle_R(header); // Get message from other rovable
//        break;
//    }
//    rov_index = rov_index + 1;
//  }
//  unsigned long now = millis();
//  if (now - last_time_sent >= interval) {
//    last_time_sent = now;
//    v = inspect();
//    beta = beta + v;
//    alpha = alpha + (1 - v);
//  }
//  rov_belief = incbeta(alpha, beta, 0.5);
//  ok = send_Base(); // Send current distribution to base
//  if (rov_belief > p_c) {
//    d_f = 1;
//  }
//  if ((1 - rov_belief) > p_c) {
//    d_f = 0;
//  }
//  if (d_f != -1 & u_plus == 1) {
//    ok = send_Rov(d_f);
//  } else {
//    ok = send_Rov(v);
//  }
}
