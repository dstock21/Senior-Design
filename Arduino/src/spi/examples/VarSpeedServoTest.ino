#include <VarSpeedServo.h> 

// create servo object to control a servo 
VarSpeedServo myservo1;
int input;
 
void setup() {

  // initialize serial:
 Serial.begin(9600);
  
  myservo1.attach(9);
} 
   
void loop() {
  
  int LEF = 0;
  int RIG = 180;
  
  int SPEED1 = 160;
  int SPEED2 = 100;

  if (Serial.available() > 0) {
    input = Serial.parseInt();
    myservo1.write(input,SPEED1);
    myservo1.wait();
    Serial.println(input);
  }
}

