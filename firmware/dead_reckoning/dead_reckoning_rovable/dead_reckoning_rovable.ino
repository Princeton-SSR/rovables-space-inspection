#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RF24Network.h>
#include <RF24.h>
#include <accIntegral.h>

#define  M1_DIR   A2 
#define  M1_PWM   9 
#define  M2_PWM   4 
#define  M2_DIR   A1 
#define ENC1_INT  3  
#define ENC2_INT  7
#define ENC1_LED  8
#define ENC2_LED  6

struct message_IMU {
  double ax; 
  double ay; 
  double az; 

  //double gx; 
  //double gy; 
  //double gz; 
};

accIntegral fusion;

// Filter coefficients                       //  Unit           
constexpr float GRAVITY = 9.81e3;            //  mm/s^2             Magnitude of gravity at rest. Determines units of velocity. [UNITS MUST MATCH ACCELERATION]
constexpr float SD_ACC  = 1000 / GRAVITY;    //  mm/s^2 / g-force   Standard deviation of acceleration. Deviations from zero are suppressed.
constexpr float SD_VEL  = 200  / GRAVITY;    //  mm/s   / g-force   Standard deviation of velocity. Deviations from target value are suppressed.
constexpr float ALPHA   = 0.5;               //                     Gain of heading update - See example "output" for more information.

const uint16_t this_rov = 01; 


Adafruit_MPU6050 mpu;
RF24 radio(2, 10);   // nRF24L01 (CE,CSN)
RF24Network network(radio);
sensors_event_t a, g, temp;

void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(115200);

  SerialUSB.println("Beginning Dead Reckoning. . .");

  fusion.setup();

  SerialUSB.println("Checking MPU6050. . .");
  if (!mpu.begin()) {
    SerialUSB.println("Failed to intialize MPU6050 chip");
    while(1) {
      delay(10);
    }
  }
  SerialUSB.println("MPU6050 Responsive!");
  SerialUSB.println("------------------------------------------");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  SerialUSB.println("Checking NRF24. . .");
  if (!radio.begin()) {
    SerialUSB.println("Failed to initialize NRF24 chip");
    while(1) {
      delay(10);
    }  // hold in infinite loop
  }

  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(100);
  network.begin(this_rov);

  SerialUSB.println("NRF24 Responsive!");
  SerialUSB.println("------------------------------------------");

//  SerialUSB.println("Turning Rovable Counter-clockwise!");
//  digitalWrite(M1_DIR, 0);
//  digitalWrite(M2_DIR, 0);  
//  analogWrite(M1_PWM, 75);
//  analogWrite(M2_PWM, 75);

}
void loop() {
  network.update();
  
  mpu.getEvent(&a, &g, &temp);
  
  //Get Velocity 
  vec3_t accel = {a.acceleration.x, a.acceleration.y, a.acceleration.z};
  vec3_t gyro = {g.gyro.x, g.gyro.y, g.gyro.z};

  // Update heading and velocity estimate:
  
      // known measured velocity (target state). Estimate will be forced towards this vector
  vec3_t vel_t = {0,0,0};
  vel_t /= GRAVITY;                         // must have unit: g-force * second

  /* note: all coefficients are optional and have default values */
  fusion.update( gyro, accel, vel_t, SD_ACC, SD_VEL, ALPHA ); 

  // obtain velocity estimate
  vec3_t vel = fusion.getVel() * GRAVITY;   // scale by gravity for desired units

  // Display velocity components: [view with serial plotter]

  SerialUSB.print( vel.x, 2 );
  SerialUSB.print( " " );
  SerialUSB.print( vel.y, 2 );
  SerialUSB.print( " " );
  SerialUSB.print( vel.z, 2 );  
  SerialUSB.println();
  
  /*
  RF24NetworkHeader header(00); // Header denots intended recipient
  message_IMU message = {a.acceleration.x, a.acceleration.y, a.acceleration.z};
  if (network.write(header, &message, sizeof(message_IMU))) {
    SerialUSB.println("Message Sent");
    SerialUSB.println(message.ax);
    delay(10);
  } else {
    SerialUSB.println("Message Failed to Send!");
  }
  */
}
