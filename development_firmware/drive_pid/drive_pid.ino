/*
This file implements the Bayes Bot algorithm onto the Rovables platform
*/
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RF24Network.h>
#include <RF24.h>
#include "definitions.h"

#include <printf.h>

//Hardware devices
Adafruit_MPU6050 mpu;


/*motor values used for PID etc.
*/
uint16_t motor_speed = 75;
uint16_t motor_speed_l = motor_speed;  //used during coll avoid and turning
uint16_t motor_speed_r = motor_speed;  //used during coll avoid and turning
uint16_t motor_left = motor_speed;     // used during driving striaght
uint16_t motor_right = motor_speed;    //used during driving straight
uint16_t motor_max = 90;
bool stuck = false;
int stuck_motor_speed = 27;
unsigned long stuck_start = 0;
float gyro_z = 0;


/*
MPU 6050 data structure
*/
sensors_event_t a, g, temp;




void turn_angle(double angle, uint8_t dir) {  // dir = 1 (cw) dir =2 (ccw)
  /* This function takes angle as argument in degrees and turns based on PI control on the gyro in either cw or ccw direction.
  */

  //Stopping the motor
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, HIGH);
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  digitalWrite(M1_DIR, 0);
  digitalWrite(M2_DIR, 1);
  delay(5);

  //INIT VALUES OF CONTROLLERS
  double angle_current = 0;
  double error = angle - angle_current;  //absolute error
  double d_error = 0;                    //derivative error
  double c_error = 0;                    //cumulative error
  double P_action = 0;
  double I_action = 0;
  double D_action = 0;
  double P_gain = 1.25;
  double I_gain = 0.01;
  double D_gain = 0.0;
  double dt = (2.64 / 1000);  //approximate using serial
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  while (abs(error) > 1) {
    mpu.getEvent(&a, &g, &temp);                                     // get sensor data
    angle_current = dt * (g.gyro.z * 180 / 3.1415) + angle_current;  //loop time is about 2.64 milliseconds
    if (dir == 1) {
      d_error = ((angle + angle_current) - error) / dt;
      error = angle + angle_current;  // calculate error
    }
    if (dir == 2) {
      d_error = ((angle - angle_current) - error) / dt;
      error = angle - angle_current;  // calculate error
    }
    c_error += error;
    P_action = error * P_gain;
    I_action = c_error * I_gain;
    D_action = d_error * D_gain;

    motor_speed = (uint16_t)(P_action + I_action + D_action);  // combination of used as control
    // Limiting and setting the motor inputs
    if (motor_speed > 100) { motor_speed = 100; }
    if (motor_speed < 10) { motor_speed = 10; }
    //Sending controller output to motors

    if (dir == 1) {
      analogWrite(M1_PWM, motor_speed);
      analogWrite(M2_PWM, motor_speed);
      digitalWrite(M1_DIR, 1);
      digitalWrite(M2_DIR, 1);
    }
    if (dir == 2) {
      analogWrite(M1_PWM, motor_speed);
      analogWrite(M2_PWM, motor_speed);
      digitalWrite(M1_DIR, 0);
      digitalWrite(M2_DIR, 0);
    }
  }
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  digitalWrite(M1_DIR, 0);
  digitalWrite(M2_DIR, 0);
}

void reset_motor() {
  //Set all used motor speed to the reference speed that was initialized
  motor_speed_r = motor_speed;
  motor_speed_l = motor_speed;
  motor_left = motor_speed;
  motor_right = motor_speed;
}

void drive_time(int dur, uint8_t dir) {  // dir = 1 (fw) dir =2 (bw)
  /* 
  This function drives for an amount of time (without CA) and tries to drive straight based on the gyro
  */

  //Initializing values

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, HIGH);
  analogWrite(M1_PWM, motor_speed_r);
  analogWrite(M2_PWM, motor_speed_l);
  digitalWrite(M1_DIR, 0);
  digitalWrite(M2_DIR, 1);
  unsigned long now_i = millis();
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);//set low-pass of gyro
  //INIT VALUES OF CONTROLLERS
  gyro_z = 0;
  double setp = 0;
  double error = setp - gyro_z;  //absolute error
  double c_error = 0;            //cumulative error
  double P_action = 0;
  double I_action = 0;
  double P_gain = 5;
  double I_gain = .15;
  reset_motor();
  while ((millis() - now_i ) < dur) {//move for duriation without counting the CA time
    if (true) {//if nothing is seen on the CA try to drive straight according to the following controller
      mpu.getEvent(&a, &g, &temp);
      gyro_z = g.gyro.z;
      error = setp - gyro_z;  //absolute error
      c_error += error;
      P_action = error * P_gain;
      I_action = c_error * I_gain;
      motor_left -= (uint16_t)(P_action + I_action);   // combination of used as control
      motor_right += (uint16_t)(P_action + I_action);  // combination of used as control
      if (motor_left > motor_max) { motor_left = motor_max; }
      if (motor_right > motor_max) { motor_right = motor_max; }
      if (dir == 1) {
        analogWrite(M1_PWM, motor_right);
        analogWrite(M2_PWM, motor_left);
        digitalWrite(M1_DIR, 0);
        digitalWrite(M2_DIR, 1);
      }
      if (dir == 2) {
        analogWrite(M1_PWM, motor_right);
        analogWrite(M2_PWM, motor_left);
        digitalWrite(M1_DIR, 1);
        digitalWrite(M2_DIR, 0);
      }
    } else {// if we see something on the ToF use the motor values set in the CA function
      if (dir == 1) {
        analogWrite(M1_PWM, motor_speed_r);
        analogWrite(M2_PWM, motor_speed_l);
        digitalWrite(M1_DIR, 0);
        digitalWrite(M2_DIR, 1);
      }
      if (dir == 2) {
        analogWrite(M1_PWM, motor_speed_r);
        analogWrite(M2_PWM, motor_speed_l);
        digitalWrite(M1_DIR, 1);
        digitalWrite(M2_DIR, 0);
      }
    }
  }
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);//disable low-pass of gyro
  stuck = false;
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  digitalWrite(M1_DIR, 0);
  digitalWrite(M2_DIR, 1);
  reset_motor();
}





void doRandomWalk() {//random walk function pretty straight forward, drive for x time and turn randomly
  unsigned long random_walk_start = millis();
  reset_motor();
  drive_time(random(2000, 8000), 1);
  reset_motor();
  turn_angle(random(0, 180), random(1, 3));
  reset_motor();

}



void setup() {

  SerialUSB.begin(115200);
  delay(2000);
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


  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);

  

  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
}

void loop() {

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, HIGH);
  //doRandomWalk();
  turn_angle(90, 1);
  delay(1000); 
  

}
