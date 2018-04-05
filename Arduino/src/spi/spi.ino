#include <SPI.h>
#include "VarSpeedServo.h"
#include "MatrixMath.h"

// NOTES:
// Reference gait is still not being uploaded to Arduino yet.
// In order to apply servo positions manually, go to function "run_servo()" 
// and manually apply an angle in microseconds. also, ake sure run_servo() 
// is uncommented in loop(). 
// The only non-deterministic function in loop() is get_angle(). Comment 
// all instances if they are not relevant to current test.

// UPDATE
#define NREF 139
#define BETA 0.9 //0 = filter is off
#define T_OFFSET 1.85
#define T_SENSITIVITY (8*6/5.6) 
#define K 0.1 // UPDATE
#define KP 0
#define KD 0
#define ERR_RANGE 3 // how far off from reference is "correct" in degrees
#define PHIK_M 72.1
#define PHIK_B 1155
#define PHIH_M 72.1
#define PHIH_B 1155
#define MIN_PHI 1150

// PINS
#define KNEE_ANGLE 3 //Chip or Slave select 
#define HIP_ANGLE 4 //Chip or Slave select
#define KNEE_TORQUE A1
#define HIP_TORQUE A2
#define SERVO_KNEE 5
#define SERVO_HIP 6
#define ZERO_RESET 7

// Other
#define SPEED 160
#define CONV_D2R M_PI/180
#define BEGIN_SEND -10000.0

VarSpeedServo ServoH;
VarSpeedServo ServoK;

uint16_t ABSposition = 0;
uint16_t ABSposition_last = 0;
uint8_t temp[2];

// State:
// 0: knee angle
// 1: hip angle
// 2: knee torque
// 3: hip torque
float state[4];
float stateavg[4];
// 0: hip-knee
// 1: knee-ankle
//float L[2];

float Ref[2*NREF];
int i_err;
float qk_err;
float qh_err;
float qk_off;
float qh_off;

float t_err[4]; //knee, hip, knee', hip'
float ttemp;
float phik;
float phih;
float tk_des;
float th_des;

float deg = 0.00;

unsigned long time;
unsigned long temptime;
float T;

unsigned long count = 0;
char incomingByte;

void setup()
{
  // encoder set up
  pinMode(KNEE_ANGLE,OUTPUT);//Slave Select
  digitalWrite(KNEE_ANGLE,HIGH);
  pinMode(HIP_ANGLE,OUTPUT);//Slave Select
  digitalWrite(HIP_ANGLE,HIGH);
  pinMode(ZERO_RESET,OUTPUT);//Slave Select
  digitalWrite(ZERO_RESET,HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16); 

  //torque sensor set up
  pinMode(HIP_TORQUE, INPUT);
  digitalWrite(HIP_TORQUE, LOW);
  pinMode(KNEE_TORQUE, INPUT);
  digitalWrite(KNEE_TORQUE, LOW);

  // general set up
  Serial.begin(9600);
//  Serial.println("STARTING...");
//  Serial.println("SETTING ZERO...");
  
//  uint8_t signal = SPI_T(0x70);    // set zero at set up
//  while (signal != 0x80) {
//    signal = SPI_T(0x00);
//  }
  Serial.flush();
  delay(2000);
  SPI.end();

  ServoH.attach(SERVO_HIP);
  ServoK.attach(SERVO_KNEE);

  for (int i = 0; i < 4; i++) {
    state[i] = 0;
    stateavg[i] = 0;
  }

  for (int i = 0; i < 2*NREF; i++) {
    Ref[i] = 0;
  }

  //L[0] = 0.32;
  //L[1] = 0.42;

  time = millis();
  T = time;
}
uint8_t SPI_T (uint8_t msg, int joint)    //Repetive SPI transmit sequence
{
   uint8_t msg_temp = 0;  //vairable to hold recieved data
   digitalWrite(joint,LOW);     //select spi device
   msg_temp = SPI.transfer(msg);    //send and recieve
   digitalWrite(joint,HIGH);    //deselect spi device
   return(msg_temp);      //return recieved byte
}

float get_angle(int joint) {
  uint8_t recieved = 0xA5;    //just a temp vairable
   ABSposition = 0;    //reset position vairable
   
   SPI.begin();    //start transmition
   digitalWrite(joint,LOW);
   
   SPI_T(0x10, joint);   //issue read command
   recieved = SPI_T(0x00, joint);    //issue NOP to check if encoder is ready to send
   
   while (recieved != 0x10)    //loop while encoder is not ready to send
   { 
     recieved = SPI_T(0x00, joint); 

     //Serial.println(joint); //cleck again if encoder is still workin
     delayMicroseconds(20);    //wait a bit
   }
   
   temp[0] = SPI_T(0x00, joint);    //Recieve MSB
   temp[1] = SPI_T(0x00, joint);    // recieve LSB
   
   digitalWrite(joint,HIGH);  //just to make sure   
   SPI.end();    //end transmition
   
   temp[0] &=~ 0xF0;    //mask out the first 4 bits
    
   ABSposition = temp[0] << 8;    //shift MSB to correct ABSposition in ABSposition message
   ABSposition += temp[1];    // add LSB to ABSposition message to complete message
    
   if (ABSposition != ABSposition_last)    //if nothing has changed dont wast time sending position
   {
     ABSposition_last = ABSposition;    //set last position to current position
     deg = ABSposition;
     deg = deg * 0.08789;    // aprox 360/4096
     return deg;
   }   

}

void setzero() {
  for (int i = 0; i < 2; i++) {
  uint8_t signal = SPI_T(0x70, i + 3);    // set zero at set up
  while (signal != 0x80) {
     signal = SPI_T(0x00, i + 3);
     }
     digitalWrite(ZERO_RESET,LOW);
     delay(50);
     digitalWrite(ZERO_RESET,HIGH);
  }

  qk_off = get_torque(KNEE_TORQUE);
  qh_off = get_torque(HIP_TORQUE);
}

float get_torque(int joint) {
  float adc_output = analogRead(joint);
  float voltage = (5.0/1024)*adc_output;
  
  float torque = (voltage-T_OFFSET)*T_SENSITIVITY;
  
  return torque;
}

void send_values(float* values, int len) {
  Serial.println(BEGIN_SEND);
  for (int i = 0; i < len; i++) {
    Serial.println(values[i]);
  }
}

void average(float* avg, float* curr, int len) {
  for(int i = 0; i < len; i++) {
    avg[i] = BETA*avg[i]+(1-BETA)*curr[i];
  }
}

//void fk() {
//  state[6] = L[0]*sin(stateavg[1]*CONV_D2R)+L[1]*sin(stateavg[0]*CONV_D2R);
//  state[7] = -L[0]*cos(stateavg[1]*CONV_D2R)-L[1]*cos(stateavg[0]*CONV_D2R);
//}
//
//// row major -> (0,0)=0, (0,1)=1, (1,0)=2, (1,1)=3                                                                                                                                                                                                                                                                                                                                                                                                                             
//void jacobian(float* jacobian) {
//  jacobian[0] = L[1]*cos((stateavg[0]-stateavg[1])*CONV_D2R) + L[0]*cos(stateavg[1]*CONV_D2R);
//  jacobian[1] = -L[1]*cos((stateavg[0]-stateavg[1])*CONV_D2R);
//  jacobian[2] = -L[1]*sin((stateavg[0]-stateavg[1])*CONV_D2R) - L[0]*sin(stateavg[1]*CONV_D2R);
//  jacobian[3] = L[1]*sin((stateavg[0]-stateavg[1])*CONV_D2R);
//}

// Returns min position error index
int err() {
  // joint space implementation
  float err;
  int i_err;
  float error = sq(Ref[0]-stateavg[0]) + sq(Ref[1]-stateavg[1]);
  for(int i = 3; i < NREF; i+=3) {
    err = sq(Ref[2*i]-stateavg[0]) + sq(Ref[2*i+1]-stateavg[1]);
    if(err < error) {
      i_err = i;
      error = err;
    }
  }
  for(int i = i_err-2; i < i_err+3; i++) {
    int j = i % NREF;
    err = sq(Ref[2*j]-stateavg[0]) + sq(Ref[2*j+1]-stateavg[1]);
    if(err < error) {
      i_err = j;
      error = err;
    }
  }
  return i_err;
}

float phi_est(float t, float m, float b) {
  float phi_des = m*t + b;
  if (phi_des < MIN_PHI) {
    return MIN_PHI;
  }
  return phi_des;
}

void run_servo() {
  // correct servos
  if (phik < MIN_PHI) {
   ServoK.write(phik,SPEED);
  } else {
   ServoK.write(phik, SPEED);
  }
  if (phih < MIN_PHI) {
   ServoH.write(phih,SPEED);
  } else {
   ServoH.write(phih, SPEED);
  }
}

void loop()
{
       // take measurements
       state[0] = get_angle(KNEE_ANGLE)/4;
       state[2] = get_torque(KNEE_TORQUE)-qk_off;
       state[3] = get_torque(HIP_TORQUE)-qh_off;
       state[1] = get_angle(HIP_ANGLE)/4;
       
       // calculate T for derivatives
       temptime = time;
       time = millis();
       T = (time-temptime)/1000.0;

       average(stateavg, state, 4);

       // compare to reference
       i_err = err();

       qk_err = Ref[2*i_err]-stateavg[0] - ERR_RANGE;
       qh_err = Ref[2*i_err+1]-stateavg[1] - ERR_RANGE;
       if (qk_err < 0) qk_err = 0;
       if (qh_err < 0) qh_err = 0;
       
       // controls
       tk_des = K*sq(qk_err);
       th_des = K*sq(qh_err); 

       ttemp = tk_des - stateavg[2];
       t_err[2] = (t_err[0]-ttemp)/T;
       t_err[0] = ttemp;
       ttemp = tk_des - stateavg[3];
       t_err[3] = (t_err[1]-ttemp)/T;
       t_err[1] = ttemp;

       phik = phi_est(tk_des, PHIK_M, PHIK_B) + KP*t_err[0] + KD*t_err[2];
       phih = phi_est(th_des, PHIH_M, PHIH_B) + KP*t_err[1] + KD*t_err[3];

       // correct servos
       // run_servo();

       // troubleshooting timer
//       state[2] = T*1000.0;
//       state[3] = time;
//       state[0] = count;

       if (count++ % 10 ==0) {
        send_values(state, 4);

        if (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.read();

            // say what you got:
            if (incomingByte == 'r') {
              setzero();
            }
        }
       }
}
