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
#define BETA 0.85 //0 = filter is off
#define T_OFFSET_K 1.85
#define T_SENSITIVITY_K 6.82
#define T_OFFSET_H 1.85
#define T_SENSITIVITY_H 6.667
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
#define KNEE_TORQUE A0
#define HIP_TORQUE A1
#define SERVO_KNEE 5
#define SERVO_HIP 6
#define ZERO_RESET 7

// Other
#define SPEED 160
#define CONV_D2R M_PI/180
#define BEGIN_SEND -10000.0
#define MAX_TORQUE 10
#define WRAP_THRESH 45
#define WRAP 90

VarSpeedServo ServoH;
VarSpeedServo ServoK;

uint16_t ABSposition = 0;
uint16_t ABSposition_last = 0;
uint8_t temp[2];
float k = 0.1; // UPDATE

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

float Ref[2*NREF] = {18.3,6.7,17.75,4.7,17.2,2.7,16.7,1.3,16.2,-0.1,15.7,-0.95,15.2,-1.8,14.75,
-2.05,14.3,-2.3,13.95,-2.05,13.6,-1.8,13.2,-1.2,12.8,-0.6,12.5,0.25,12.2,1.1,11.95,2.05,11.7,3,
11.55,4.05,11.4,5.1,11.3,6.25,11.2,7.4,11.25,8.6,11.3,9.8,11.4,10.95,11.5,12.1,11.55,13.05,11.6,
14,11.5,14.7,11.4,15.4,11.1,15.8,10.8,16.2,10.3,16.25,9.8,16.3,9.2,16.1,8.6,15.9,7.9,15.55,7.2,15.2,
6.5,14.7,5.8,14.2,5.15,13.7,4.5,13.2,4,12.75,3.5,12.3,3.1,11.85,2.7,11.4,2.35,11.05,2,10.7,1.75,10.4,
1.5,10.1,1.3,9.75,1.1,9.4,0.95,9.1,0.8,8.8,0.6,8.5,0.4,8.2,0.2,7.9,0,7.6,-0.25,7.3,-0.5,7,-0.85,6.7,
-1.2,6.4,-1.5,6.15,-1.8,5.9,-2.15,5.65,-2.5,5.4,-2.8,5.3,-3.1,5.2,-3.4,5.15,-3.7,5.1,-3.95,5.3,-4.2,
5.5,-4.4,5.85,-4.6,6.2,-4.75,6.75,-4.9,7.3,-5.05,8.1,-5.2,8.9,-5.4,9.9,-5.6,10.9,-5.75,12.1,-5.9,13.3,
-6,14.75,-6.1,16.2,-6.15,17.9,-6.2,19.6,-6.2,21.55,-6.2,23.5,-6.1,25.65,-6,27.8,-5.85,30.1,-5.7,32.4,
-5.35,34.9,-5,37.4,-4.5,39.95,-4,42.5,-3.25,45.05,-2.5,47.6,-1.5,50,-0.5,52.4,0.65,54.55,1.8,56.7,3.15,
58.55,4.5,60.4,5.9,61.9,7.3,63.4,8.75,64.4,10.2,65.4,11.55,65.95,12.9,66.5,14.1,66.55,15.3,66.6,16.3,
66.15,17.3,65.7,18.1,64.8,18.9,63.9,19.5,62.6,20.1,61.3,20.5,59.65,20.9,58,21.25,56.1,21.6,54.2,21.85,
52.05,22.1,49.9,22.25,47.55,22.4,45.2,22.5,42.6,22.6,40,22.55,37.25};
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
int wrap_k = 0;
int wrap_h = 0;
float ka;
float ha;

unsigned long time;
unsigned long temptime;
float T;

unsigned long count = 0;
char incomingByte;

void setup()
{
  //For testing
  pinMode(LED_BUILTIN, OUTPUT);
  
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
    t_err[i] = 0;
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
  SPI.begin();
  uint8_t signal = SPI_T(0x70, i + 3);    // set zero at set up  
  while (signal != 0x80) {
     signal = SPI_T(0x00, i + 3);
     } 
  SPI.end();    //end transmition
    
  }
  digitalWrite(ZERO_RESET,LOW);
  delay(50);
  digitalWrite(ZERO_RESET,HIGH);

  qk_off = get_torque(KNEE_TORQUE, T_OFFSET_K, T_SENSITIVITY_K);
  qh_off = get_torque(HIP_TORQUE, T_OFFSET_H, T_SENSITIVITY_H);
}

float get_torque(int joint, float off, float sens) {
  float adc_output = analogRead(joint);
  float voltage = (5.0/1024)*adc_output;

  float torque = (voltage-off)*sens;

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
  float phi_des = m*abs(t) + b;
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
       ka = state[0];
       ha = state[1];
       
       // take measurements
       state[0] = (get_angle(KNEE_ANGLE)/4 + WRAP*wrap_k);
       state[2] = get_torque(KNEE_TORQUE, T_OFFSET_K, T_SENSITIVITY_K)-qk_off;
       state[3] = get_torque(HIP_TORQUE, T_OFFSET_H, T_SENSITIVITY_H)-qh_off;
       state[1] = (get_angle(HIP_ANGLE)/4 + WRAP*wrap_h);

       if (abs(ka-state[0]) > WRAP_THRESH) {
        if (ka > state[0]) {
          state[0] += WRAP;
          wrap_k +=1;
        }
        else {
          state[0] -= WRAP;
          wrap_k -= 1;
        }
       }
       if (abs(ha-state[1]) > WRAP_THRESH) {
        if (ha > state[1]) {
          state[1] += WRAP;
          wrap_h +=1;
        }
        else {
          state[1] -= WRAP;
          wrap_h -=1;
        }
       }
       
       // calculate T for derivatives
       temptime = time;
       time = micros(); // resets to 0 every ~40 min
       // if micros() wraps, T doesn't update. Next iteration time>temptime and T updates
       if (time > temptime) T = (time-temptime)/1000000.0; 

       average(stateavg, state, 4);

       // compare to reference
       i_err = err();

       qk_err = Ref[2*i_err]-stateavg[0] - ERR_RANGE;
       qh_err = Ref[2*i_err+1]-stateavg[1] - ERR_RANGE;
       if (qk_err < 0) qk_err = 0;
       if (qh_err < 0) qh_err = 0;
       
       // controls
       tk_des = k*sq(qk_err);
       th_des = k*sq(qh_err);
       if (tk_des > MAX_TORQUE) tk_des = MAX_TORQUE;
       if (th_des > MAX_TORQUE) th_des = MAX_TORQUE;

       ttemp = tk_des - stateavg[2];
       t_err[2] = (t_err[0]-ttemp)/T;
       t_err[0] = ttemp;
       ttemp = th_des - stateavg[3];
       t_err[3] = (t_err[1]-ttemp)/T;
       t_err[1] = ttemp;

       phik = phi_est(tk_des, PHIK_M, PHIK_B) + KP*t_err[0] + KD*t_err[2];
       phih = phi_est(th_des, PHIH_M, PHIH_B) + KP*t_err[1] + KD*t_err[3];

       // correct servos
       //run_servo();

       //Test servos
//       if (count % 200 == 0) {
//        ServoK.write(1400,SPEED);
//        ServoH.write(1600,SPEED);
//        digitalWrite(LED_BUILTIN, HIGH);
//       } else if (count % 200 == 100) {
//        ServoK.write(1600,SPEED);
//        ServoH.write(1400,SPEED);
//        digitalWrite(LED_BUILTIN, LOW);
//       }

       // troubleshooting timer
//       state[2] = T*1000.0;
//       state[3] = time;
//       state[0] = count;

       if (count++ % 20 ==0) {
        send_values(stateavg, 4);
        count = 1;

        

        if (Serial.available() > 0) {
          delay(200);
                // read the incoming byte:
                incomingByte = Serial.read();

            // say what you got:
            if (incomingByte == 'r') {
              setzero();
              
            }

            if (incomingByte == 'k') {
              int ticker = 0;
              while (Serial.available() < 4 && ticker < 30) {
                continue;
                ticker++;
              }
              if (Serial.available() >= 4) {
                k = Serial.read();
              }
            }
        }
       }
}
