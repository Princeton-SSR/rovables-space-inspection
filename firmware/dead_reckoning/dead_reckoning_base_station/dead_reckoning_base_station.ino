#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>
#include <accIntegral.h>




// Filter coefficients                       //  Unit           
constexpr float GRAVITY = 9.81e3;            //  mm/s^2             Magnitude of gravity at rest. Determines units of velocity. [UNITS MUST MATCH ACCELERATION]
constexpr float SD_ACC  = 1000 / GRAVITY;    //  mm/s^2 / g-force   Standard deviation of acceleration. Deviations from zero are suppressed.
constexpr float SD_VEL  = 200  / GRAVITY;    //  mm/s   / g-force   Standard deviation of velocity. Deviations from target value are suppressed.
constexpr float ALPHA   = 0.5;               //                     Gain of heading update - See example "output" for more information.


RF24 radio(5, 2);               // nRF24L01 (CE,CSN)

RF24Network network(radio);     // Network uses that radio
const uint16_t this_rov = 00;  // Address of our node in Octal format (04, 031, etc)

struct message_IMU {
  double ax; 
  double ay; 
  double az;

  //double gx; 
  //double gy; 
  //double gz; 
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
  //fusion.setup();
}
 
void loop(void) {
  network.update();                  // Check the network regularly
  if (network.available()) {      // Is there anything ready for us?
 
    RF24NetworkHeader header;        // If so, grab it and print it out
    message_IMU message;
    network.read(header, &message, sizeof(message_IMU));
    Serial.println(message.ax);
    Serial.print(",");
    Serial.print(message.ay);
    Serial.print(",");
    Serial.println(message.az);
  } 
}
