// Pin definitions 
#define  M1_DIR   A2 //A2 is good
#define  M1_PWM   9  // 9 is good
#define  M2_PWM   4  // 4 is good
#define  M2_DIR   A1 //A1 is good
#define ENC1_INT  3  
#define ENC2_INT  7
#define ENC1_LED  8
#define ENC2_LED  6
#define TX_GPIO 1
#define RX_GPIO 0
#define  LED_BLUE  A3   /* blue led, Arduino: A3, mcu: PA4 pin9 */
#define  LED_RED   A4 

// Tof 
#define WINDOW_SIZE_L 5
#define WINDOW_SIZE_M 5
#define WINDOW_SIZE_R 5
#define WINDOW_SIZE_Z 5

extern int16_t distance_r = 0;
extern int16_t distance_l = 0;
extern int16_t distance_m = 0;