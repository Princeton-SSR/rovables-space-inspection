
#include <ral-comp-imu.h>
#include <ral-mod-motion-walks.h>
#include <ral-mod-motion-basic.h>


IMU imu; 

Walks walk(&imu); 
BasicMotion bm; 
void setup() {

  SerialUSB.begin(115200);
  delay(2000);
  SerialUSB.println("Starting Rovable");
  Wire.begin();
  Wire.setClock(400000);
  imu.init(); 
  imu.lpfOn(); 
  
}



void loop() {
  int straightOrTurn = random(0, 2);
  if (straightOrTurn == 0){
    walk.straight(1000); 
    delay(500);
  }
  else{
    int turnDir = random(0, 2); //Turn clockwise or counter clockwise
    walk.turn(90, turnDir); 
    delay(500);
  }
  
}