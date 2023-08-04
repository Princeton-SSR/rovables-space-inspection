#define  M1_DIR   A2 
#define  M1_PWM   9 
#define  M2_PWM   4 
#define  M2_DIR   A1 
#define ENC1_INT  3  
#define ENC2_INT  7
#define ENC1_LED  8
#define ENC2_LED  6

void runMotor(int rightSpeed, int leftSpeed) {
  analogWrite(M1_PWM, rightSpeed);
  analogWrite(M2_PWM, leftSpeed);
}

void setup() {
  SerialUSB.begin(115200); // Make sure the baud rate matches the Python code
}



void loop() {
  if (SerialUSB.available()) {
    int receivedNum = SerialUSB.parseInt();

    if (receivedNum == 1){
      runMotor(75,75); 
    }
    if (receivedNum == 0){
      runMotor(0,0); 
    }
    
    
  }
}







