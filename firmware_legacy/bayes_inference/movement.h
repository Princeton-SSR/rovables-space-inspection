#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

class Movement
{
  private:
    byte M1_dir;
    byte M1_pwm;
    byte M2_pwm;
    byte M2_dir;
    byte ENC1_int;
    byte ENC2_int;
    byte ENC1_led;
    byte ENC2_led;
    int caThresh;

    VL53L1X tof;

  public:
    Movement(byte M1_dir, byte M1_pwm, byte M2_pwm, byte M2_dir, byte ENC1_int, byte ENC2_int, byte ENC1_led, byte ENC2_led, int caThresh);
    void init();
    void runMotor(int dur);
    void moveForward(int dur);
    void moveBackward(int dur);
    void turnRight(int dur);
    void turnLeft(int dur);
    void doRandomWalk();
};



#endif