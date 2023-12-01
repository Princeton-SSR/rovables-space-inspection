#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>
#include <accIntegral.h>


// Gyro settings:
#define         LP_FILTER   3           // Low pass filter.                    Value from 0 to 6
#define         GYRO_SENS   0           // Gyro sensitivity.                   Value from 0 to 3
#define         ACCEL_SENS  0           // Accelerometer sensitivity.          Value from 0 to 3
#define         ADDRESS_A0  LOW         // I2C address from state of A0 pin.   A0 -> GND : ADDRESS_A0 = LOW
                                        //                                     A0 -> 5v  : ADDRESS_A0 = HIGH
// Accelerometer offset:
constexpr int   AX_OFFSET = 0.095;       // Use these values to calibrate the accelerometer. The sensor should output 1.0g if held level. 
constexpr int   AY_OFFSET = 0.675;       // These values are unlikely to be zero.
constexpr int   AZ_OFFSET = -0.74;

// =========== Settings ===========
accIntegral fusion;

// Filter coefficients                       //  Unit           
constexpr float GRAVITY = 9.81e3;            //  mm/s^2             Magnitude of gravity at rest. Determines units of velocity. [UNITS MUST MATCH ACCELERATION]
constexpr float SD_ACC  = 1000 / GRAVITY;    //  mm/s^2 / g-force   Standard deviation of acceleration. Deviations from zero are suppressed.
constexpr float SD_VEL  = 200  / GRAVITY;    //  mm/s   / g-force   Standard deviation of velocity. Deviations from target value are suppressed.
constexpr float ALPHA   = 0.5;               //                     Gain of heading update - See example "output" for more information.





 
RF24 radio(5, 2);               // nRF24L01 (CE,CSN)

RF24Network network(radio);     // Network uses that radio
const uint16_t this_rov = 00;  // Address of our node in Octal format (04, 031, etc)

struct message_accel {
  double x;
  double y;
  double z;
};


void setup(void) {
  SerialUSB.begin(115200);
  while (!SerialUSB) {
    // some boards need this because of native USB capability
  }
//  while (SerialUSB.read() != 'y') // Wait until receive start from Serial
//  {
//     //don't do anything
//  }
  SerialUSB.println("Starting Bayes Station");
 
  if (!radio.begin()) {
    SerialUSB.println(F("Radio hardware not responding!"));
    while (1) {
      // hold in infinite loop
    }
  }
  radio.setChannel(100);
  radio.setPALevel(RF24_PA_HIGH);
  network.begin(this_rov);
  
  // initialize sensor fusion
  //fusion.setup( imu.ax(), imu.ay(), imu.az() );   // ALWAYS set initial heading with acceleration 
  fusion.setup();
}
 
void loop(void) {
  network.update();                  // Check the network regularly
  if (network.available()) {      // Is there anything ready for us?
 
    RF24NetworkHeader header;        // If so, grab it and print it out
    message_accel message;
    network.read(header, &message, sizeof(message_accel));
    Serial.print(message.x);
    Serial.print(",");
    Serial.print(message.y);
    Serial.print(",");
    Serial.println(message.z);
  } 
}
