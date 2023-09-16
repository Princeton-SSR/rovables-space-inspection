#include <VL53L1X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RF24Network.h>
#include <RF24.h>
#include "definitions.h"
#include <printf.h>


/* Hardware devices */
Adafruit_MPU6050 mpu;
VL53L1X tof_left;
VL53L1X tof_right;
VL53L1X tof_middle;
RF24 radio(2, 10, 2000000);
RF24Network network(radio);

/*Setting Parameters*/
const uint16_t this_rov = 01;  //{00, 01, 02, 03, 04}; = Rovable ID on the network
const uint16_t base = 00; // address of base station

// Parameters used in the ToF Collision Avoidance
int caThresh = 50; 
bool nearObs = false;


struct message_tof {

  uint16_t distance_l;
  uint16_t distance_m;
  uint16_t distance_r;

};



bool isNearObs(){
  //Booleans to state if something is within the threshold left middle or right
  bool bool_m = false;
  bool bool_l = false;
  bool bool_r = false;
  if (tof_left.dataReady() && tof_right.dataReady() && tof_middle.dataReady()) {//only enter function if data is ready
    //read left tof sensor and check if it is in the threshold, the read(false) is non-blocking 
    tof_left.read(false);
    distance_l = tof_left.ranging_data.range_mm;
    if (distance_l <= caThresh) { bool_l = true; }
    //read right tof sensor and check if it is in the threshold, the read(false) is non-blocking 
    tof_right.read(false);
    distance_r = tof_right.ranging_data.range_mm;
    if (distance_r <= caThresh) { bool_r = true; }
    //read middle tof sensor and check if it is in the threshold, the read(false) is non-blocking 
    tof_middle.read(false);
    distance_m = tof_middle.ranging_data.range_mm;
    if (distance_m <= caThresh) { bool_m = true; }

    LowPassFilterTof();
  } 


  if (bool_l || bool_r || bool_m) {               //if something is within the threshold move in this statement
    //send_tof_base();
    digitalWrite(LED_RED, HIGH);                  //put red light high
    digitalWrite(LED_BLUE, LOW);
    nearObs = true;                            //set global variable true "I see something"
      //do similar thing on the left

  }

  else {                                            //if nothing is within the threshold move in this statement
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_BLUE, HIGH);                   //burn blue light
    nearObs = false;                             //set global variable to false
  }

  return nearObs; 
}

void coll_avoid() {
  if (isNearObs()){
    send_tof_base(); 
    moveBackward(75, 75); 
    delay(1000); 
    runMotor(0,0); 
  }
}

// Send message to base station
void send_tof_base() {
  //send the message to the base station
  bool ok = false;
  network.update();
  RF24NetworkHeader header(00);
  message_tof message = {  distance_l, distance_m, distance_r };
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_BLUE, LOW);
  ok = network.write(header, &message, sizeof(message_tof));
  delay(100);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, HIGH);
}


void setupTof(){
  // Disable left time of flight sensor to set address of right
  pinMode(TX_GPIO, OUTPUT);
  digitalWrite(TX_GPIO, LOW);
  pinMode(RX_GPIO, OUTPUT);
  digitalWrite(RX_GPIO, LOW);

  // Start middle tof sensor
  tof_middle.setTimeout(500);
  if (!tof_middle.init()) {
    SerialUSB.println("ToF Failed");

  }
  tof_middle.setAddress(0x2F);
  tof_middle.setDistanceMode(VL53L1X::Short);
  tof_middle.setMeasurementTimingBudget(100);
  tof_middle.startContinuous(100);

  //Enable the left tof
  pinMode(TX_GPIO, INPUT);
  delay(10);

  tof_left.setTimeout(500);
  if (!tof_left.init()) {
    SerialUSB.println("ToF Failed");

  }

  // Each sensor must have its address changed to a unique value other than
  // the default of 0x29 (except for the last one, which could be left at
  // the default). To make it simple, we'll just count up from 0x2A.
  tof_left.setAddress(0x2A);
  tof_left.setDistanceMode(VL53L1X::Short);
  tof_left.setMeasurementTimingBudget(100);
  tof_left.startContinuous(100);

  //Enable the right tof
  pinMode(RX_GPIO, INPUT);
  delay(10);

  tof_right.setTimeout(500);
  if (!tof_right.init()) {
    SerialUSB.println("ToF Failed");

  }

  // Each sensor must have its address changed to a unique value other than
  // the default of 0x29 (except for the last one, which could be left at
  // the default). To make it simple, we'll just count up from 0x2A.
  tof_right.setAddress(0x2C);
  tof_right.setDistanceMode(VL53L1X::Short);
  tof_right.setMeasurementTimingBudget(100);
  tof_right.startContinuous(100);

  // //Read sensors before starting
  tof_left.read();
  tof_right.read();
  tof_middle.read();

}

// All the motions 
void runMotor(int rightSpeed, int leftSpeed) {
  analogWrite(M1_PWM, rightSpeed);
  analogWrite(M2_PWM, leftSpeed);
}
int moveBackward(int rightSpeed, int leftSpeed) {
  digitalWrite(M1_DIR, 1);
  digitalWrite(M2_DIR, 0);

  runMotor(rightSpeed, leftSpeed);
  return 2; 
}

void setup() {

  randomSeed(this_rov);
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

  
  setupTof(); 
  
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);

  if (!radio.begin()) {
    while (1) {
      SerialUSB.println("Radio failed to start");
    }
  }

  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(100);
  radio.setRetries(0, 0);
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_LOW);
  network.begin(this_rov); /*node address*/




}

void loop() {
  coll_avoid(); 
  network.update();
}
