#include "movement.h"

Movement::Movement(byte M1_dir, byte M1_pwm, byte M2_pwm, byte M2_dir, byte ENC1_int, byte ENC2_int, byte ENC1_led, byte ENC2_led, int caThresh)
{
    this->M1_dir = M1_dir;
    this->M1_pwm = M1_pwm;
    this->M2_pwm = M2_pwm;
    this->M2_dir = M2_dir;
    this->ENC1_int = ENC1_int;
    this->ENC2_int = ENC2_int;
    this->ENC1_led = ENC1_led;
    this->ENC2_led = ENC2_led;
    this->caThresh = caThresh;

    init();

}

void Movement::init() {
    pinMode(M1_dir, OUTPUT); 
    pinMode(M1_pwm, OUTPUT); 
    pinMode(M2_pwm, OUTPUT); 
    pinMode(M2_dir, OUTPUT);
    pinMode(ENC1_int, INPUT); 
    pinMode(ENC2_int, INPUT); 
    pinMode(ENC1_led, OUTPUT);
    pinMode(ENC2_led, OUTPUT);

    tof.setTimeout(500);
    if (!tof.init()) {
        Serial1.println("Time of Flight failed to start");
        while(1);
    }

    tof.setDistanceMode(VL53L1X::Short);
    tof.setMeasurementTimingBudget(20000);

    tof.startContinuous(50);
}

void Movement::runMotor(int dur) {
    analogWrite(M1_pwm, 75);
    analogWrite(M2_pwm, 75);
    delay(dur);
    analogWrite(M1_pwm, 0);
    analogWrite(M2_pwm, 0);
}

void Movement::moveForward(int dur) {
    digitalWrite(M1_dir, 0);
    digitalWrite(M2_dir, 1);
    for (int i=0; i < dur; i++){ 
        int16_t distance;

        tof.read();
        distance = tof.ranging_data.range_mm;
        if (distance <= caThresh) {
            turnLeft(700);
        }
        else {
        // Go back to moving forward
        digitalWrite(M1_dir, 0);
        digitalWrite(M2_dir, 1);
        analogWrite(M1_pwm, 75);
        analogWrite(M2_pwm, 75);
        delay(1);
        }
    }
}

void Movement::moveBackward(int dur) {
            digitalWrite(M1_dir, 1);
            digitalWrite(M2_dir, 0);

            runMotor(dur);
        }

void Movement::turnRight(int dur){ 
    digitalWrite(M1_dir, 1);
    digitalWrite(M2_dir, 1);

    runMotor(dur);
}

void Movement::turnLeft(int dur){ 
    digitalWrite(M1_dir, 0);
    digitalWrite(M2_dir, 0);

    runMotor(dur);
}

void Movement::doRandomWalk() {
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

// class Movement {
//     private:
//         byte M1_dir;
//         byte M1_pwm;
//         byte M2_pwm;
//         byte M2_dir;
//         byte ENC1_int;
//         byte ENC2_int;
//         byte ENC1_led;
//         byte ENC2_led;
//         int caThresh;

//         VL53L1X tof;
//     public: 
//         Movement(byte M1_dir, byte M1_pwm, byte M2_pwm, byte M2_dir, byte ENC1_int, byte ENC2_int, byte ENC1_led, byte ENC2_led, byte caThresh) {
//             this->M1_dir = M1_dir;
//             this->M1_pwm = M1_pwm;
//             this->M2_pwm = M2_pwm;
//             this->M2_dir = M2_dir;
//             this->ENC1_int = ENC1_int;
//             this->ENC2_int = ENC2_int;
//             this->ENC1_led = ENC1_led;
//             this->ENC2_led = ENC2_led;
//             this->caThresh = caThresh;

//             init();
//         }

//         init() {
//             pinMode(M1_dir, OUTPUT); 
//             pinMode(M1_pwm, OUTPUT); 
//             pinMode(M2_pwm, OUTPUT); 
//             pinMode(M2_dir, OUTPUT);
//             pinMode(ENC1_int, INPUT); 
//             pinMode(ENC2_int, INPUT); 
//             pinMode(ENC1_led, OUTPUT);
//             pinMode(ENC2_led, OUTPUT);

//             tof.setTimeout(500);
//             if (!tof.init()) {
//                 Serial1.println("Time of Flight failed to start");
//                 while(1);
//             }

//             tof.setDistanceMode(VL53L1X::Short);
//             tof.setMeasurementTimingBudget(20000);

//             tof.startContinuous(50);
//         }

//         void runMotor(int dur) {
//             analogWrite(M1_pwm, 75);
//             analogWrite(M2_pwm, 75);
//             delay(dur);
//             analogWrite(M1_pwm, 0);
//             analogWrite(M2_pwm, 0);
//         }

//         void moveForward(int dur) {
//             digitalWrite(M1_dir, 0);
//             digitalWrite(M2_dir, 1);
//             for (int i=0; i < dur; i++){ 
//                 int16_t distance;

//                 tof.read();
//                 distance = tof.ranging_data.range_mm;
//                 if (distance <= caThresh) {
//                     turnLeft(700);
//                 }
//                 else {
//                 // Go back to moving forward
//                 digitalWrite(M1_dir, 0);
//                 digitalWrite(M2_dir, 1);
//                 analogWrite(M1_pwm, 75);
//                 analogWrite(M2_pwm, 75);
//                 delay(1);
//                 }
//             }
//         }

// };


