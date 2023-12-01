#define STOP 1.0e-8
#define TINY 1.0e-30


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

#define MEASURING 1
#define WALKING 2
#define AVOIDING 3
#define SENDING 4
#define SEND_SUC 5
#define RECEIVING 6
#define NOTHING 7
#define SEND_BASE 8
#define SEND_BASE_DONE 9
#define NTWK_UPDATE 10
#define UPDATE_TRUE 11
#define UPDATE_FALSE 12
#define FLUSHTX 13
#define FLUSHRX 14



#define WINDOW_SIZE_L 5
#define WINDOW_SIZE_M 5
#define WINDOW_SIZE_R 5
#define WINDOW_SIZE_Z 5