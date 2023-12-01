#include "Adafruit_VL53L1X.h"

#define IRQ_PIN 2
#define XSHUT_PIN 3

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

void setup() {
  SerialUSB.begin(115200);
  while (!Serial) delay(10);

  SerialUSB.println(F("Adafruit VL53L1X sensor demo"));

  Wire.begin();
  if (! vl53.begin(0x29, &Wire)) {
    SerialUSB.print(F("Error on init of VL sensor: "));
    SerialUSB.println(vl53.vl_status);
    while (1)       delay(10);
  }
  SerialUSB.println(F("VL53L1X sensor OK!"));

  SerialUSB.print(F("Sensor ID: 0x"));
  SerialUSB.println(vl53.sensorID(), HEX);

  if (! vl53.startRanging()) {
    SerialUSB.print(F("Couldn't start ranging: "));
    SerialUSB.println(vl53.vl_status);
    while (1)       delay(10);
  }
  SerialUSB.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(50);
  SerialUSB.print(F("Timing budget (ms): "));
  SerialUSB.println(vl53.getTimingBudget());

  /*
  vl.VL53L1X_SetDistanceThreshold(100, 300, 3, 1);
  vl.VL53L1X_SetInterruptPolarity(0);
  */
}

void loop() {
  int16_t distance;

  if (vl53.dataReady()) {
    // new measurement for the taking!
    distance = vl53.distance();
    if (distance == -1) {
      // something went wrong!
      SerialUSB.print(F("Couldn't get distance: "));
      SerialUSB.println(vl53.vl_status);
      return;
    }
    SerialUSB.print(F("Distance: "));
    SerialUSB.print(distance);
    SerialUSB.println(" mm");

    // data is read out, time for another reading!
    vl53.clearInterrupt();
  }
  delay(10);
}
