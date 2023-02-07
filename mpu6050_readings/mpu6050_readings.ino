// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float xOffset = 0.03;
float yOffset = 0.05;
float zOffset = 9.43;

void setup(void) {
  SerialUSB.begin(115200);
  while (!SerialUSB)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  SerialUSB.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    SerialUSB.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  SerialUSB.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
 
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  SerialUSB.print(a.acceleration.x - xOffset);
  SerialUSB.print(",");
  SerialUSB.print(a.acceleration.y - yOffset);
  SerialUSB.print(",");
  SerialUSB.println(a.acceleration.z - zOffset);
}
