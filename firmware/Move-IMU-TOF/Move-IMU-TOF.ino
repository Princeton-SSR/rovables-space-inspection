#include <ral-comp-imu.h>
#include <RF24Network.h>
#include <RF24.h>
#include <ral-comp-radio-messages.h>
#include <ral-comp-radio.h>
#include <ral-mod-motion-walks.h>


IMU imu; 
Comms comm(01); 
Walks walk(&imu); 


int rcControl = 0; 
double startTime = millis();

void setup() {
  SerialUSB.begin(115200);
  delay(2000);
  SerialUSB.println("Starting Rovable");
  Wire.begin();
  Wire.setClock(400000);
  imu.init(); 
  imu.lpfOn(); 
  comm.setupRadio(); 
  comm.setupNetwork(); 
}

void loop() {
  int straightOrTurn = random(0, 2);
  SerialUSB.println(straightOrTurn);
  
  if (straightOrTurn != 0){
    walk.straightWithComms(1000, 00, &comm); 
    //walk.straight(1000); 
    delay(500);
  }
  
  else{
    int turnDir = random(0, 2); //Turn clockwise or counter clockwise
    walk.turnWithComms(90, turnDir, 00, &comm); 
    //walk.turn(90, turnDir); 
    //walk.turn(90, turnDir);  
    delay(500);
  }
  
  /*
  int turnDir = random(0, 2); //Turn clockwise or counter clockwise
  walk.turnWithComms(90, turnDir, 00, comm); 
  //walk.turn(90, turnDir); 
  delay(500);
  */
}

