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
#define T_OFFSET_K 1.87
#define T_SENSITIVITY_K 7.1
#define T_OFFSET_H 1.85
#define T_SENSITIVITY_H 6.667
#define KP 0
#define KD 0
#define ERR_RANGE 3 // how far off from reference is "correct" in degrees
#define PHIK_A 0.6549
#define PHIK_B -11.458
#define PHIK_C 87.148
#define PHIK_D 1797.9
#define PHIH_A 0.6549
#define PHIH_B -11.458
#define PHIH_C 87.148
#define PHIH_D 1797.9
#define MIN_PHI 1300
#define MAX_PHI 2250

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
float k = 0.05; // UPDATE

// State:
// 0: knee angle
// 1: hip angle
// 2: knee torque
// 3: hip torque
float state[4];
float stateavg[4];

float Ref[2*NREF] = {6.7,18.3,4.7,17.75,2.7,17.2,1.3,16.7,-0.1,16.2,-0.95,15.7,-1.8,
15.2,-2.05,14.75,-2.3,14.3,-2.05,13.95,-1.8,13.6,-1.2,13.2,-0.6,12.8,0.25,12.5,1.1,
12.2,2.05,11.95,3,11.7,4.05,11.55,5.1,11.4,6.25,11.3,7.4,11.2,8.6,11.25,9.8,11.3,10.95,
11.4,12.1,11.5,13.05,11.55,14,11.6,14.7,11.5,15.4,11.4,15.8,11.1,16.2,10.8,16.25,10.3,
16.3,9.8,16.1,9.2,15.9,8.6,15.55,7.9,15.2,7.2,14.7,6.5,14.2,5.8,13.7,5.15,13.2,4.5,
12.75,4,12.3,3.5,11.85,3.1,11.4,2.7,11.05,2.35,10.7,2,10.4,1.75,10.1,1.5,9.75,1.3,9.4,
1.1,9.1,0.95,8.8,0.8,8.5,0.6,8.2,0.4,7.9,0.2,7.6,0,7.3,-0.25,7,-0.5,6.7,-0.85,6.4,-1.2,
6.15,-1.5,5.9,-1.8,5.65,-2.15,5.4,-2.5,5.3,-2.8,5.2,-3.1,5.15,-3.4,5.1,-3.7,5.3,-3.95,
5.5,-4.2,5.85,-4.4,6.2,-4.6,6.75,-4.75,7.3,-4.9,8.1,-5.05,8.9,-5.2,9.9,-5.4,10.9,-5.6,
12.1,-5.75,13.3,-5.9,14.75,-6,16.2,-6.1,17.9,-6.15,19.6,-6.2,21.55,-6.2,23.5,-6.2,25.65,
-6.1,27.8,-6,30.1,-5.85,32.4,-5.7,34.9,-5.35,37.4,-5,39.95,-4.5,42.5,-4,45.05,-3.25,47.6,
-2.5,50,-1.5,52.4,-0.5,54.55,0.65,56.7,1.8,58.55,3.15,60.4,4.5,61.9,5.9,63.4,7.3,64.4,
8.75,65.4,10.2,65.95,11.55,66.5,12.9,66.55,14.1,66.6,15.3,66.15,16.3,65.7,17.3,64.8,18.1,
63.9,18.9,62.6,19.5,61.3,20.1,59.65,20.5,58,20.9,56.1,21.25,54.2,21.6,52.05,21.85,49.9,
22.1,47.55,22.25,45.2,22.4,42.6,22.5,40,22.6,37.25,22.55,34.5,22.5,31.55,22.35,28.6,22.2,
25.6,21.9,22.6,21.6,19.55,21.15,16.5,20.7,13.7,20.15,10.9,19.6,8.35,18.95,5.8,18.3};
int i_err;
float qk_err;
float qh_err;
float qk_off;
float qh_off;

float t_err[4]; //knee, hip, knee', hip'
float ttemp;
float phik = MIN_PHI;
float phih = MIN_PHI;
float tk_des = 0;;
float th_des = 0;;

float deg = 0.00;
int wrap_k = 0;
int wrap_h = 0;
float ka;
float ha;

unsigned long time;
unsigned long temptime;
float T;
float T0 = 0;

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

  time = millis();
  T = time;

  phik = 0;
  phih = 0;
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

  qk_off = qk_off + stateavg[2];
  qh_off = qh_off + stateavg[3];
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

// Returns min position error index
int err() {
  // joint space implementation
  float err;
  int i_err;
  int skip = 2;
  float error = sq(Ref[0]-stateavg[0]) + sq(Ref[1]-stateavg[1]);
  for(int i = 1+skip; i < NREF; i+=1+skip) {
    err = sq(Ref[2*i]-stateavg[0]) + sq(Ref[2*i+1]-stateavg[1]);
    if(err < error) {
      i_err = i;
      error = err;
    }
  }
  for(int i = i_err-skip; i <= i_err+skip; i++) {
    int j = i % NREF;
    err = sq(Ref[2*j]-stateavg[0]) + sq(Ref[2*j+1]-stateavg[1]);
    if(err < error) {
      i_err = j;
      error = err;
    }
  }
  return i_err;
}

float phi_est(float t, float a, float b, float c, float d) {
  float phi_des = a*sq(t)*abs(t) + b*sq(t) + c*abs(t) + d;
  if (phi_des < MIN_PHI) {
    return MIN_PHI;
  }
  return phi_des;
}

void run_servo() {
  // correct servos
  if (phik < MIN_PHI) {
   ServoK.write(MIN_PHI,SPEED);
  } else if (phik > MAX_PHI) {
   ServoK.write(MAX_PHI,SPEED);
  } else {
   ServoK.write(phik, SPEED);
  }
  if (phih < MIN_PHI) {
   ServoH.write(MIN_PHI,SPEED);
  } else if (phih > MAX_PHI) {
   ServoH.write(MAX_PHI,SPEED);
  } else {
   ServoH.write(phih, SPEED);
  }
}

void loop()
{
       ka = state[0];
       ha = state[1];
       
       // take measurementsM
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

//       qk_err = abs(stateavg[0]) - ERR_RANGE;
//       qh_err = abs(stateavg[1]) - ERR_RANGE;
       qk_err = abs(Ref[2*i_err]-stateavg[0]) - ERR_RANGE;
       qh_err = abs(Ref[2*i_err+1]-stateavg[1]) - ERR_RANGE;
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

       phik = phi_est(tk_des, PHIK_A, PHIK_B, PHIK_C, PHIK_D) + KP*t_err[0] + KD*t_err[2];
       phih = phi_est(th_des, PHIH_A, PHIH_B, PHIH_C, PHIH_D) + KP*t_err[1] + KD*t_err[3];

       // correct servos
       run_servo();

       //Test servos
//       if (count % 200 == 0) {
//        ServoK.write(MIN_PHI,SPEED);
//        ServoH.write(MAX_PHI,SPEED);
//        digitalWrite(LED_BUILTIN, HIGH);
//       } else if (count % 200 == 100) {
//        ServoK.write(MAX_PHI,SPEED);
//        ServoH.write(MIN_PHI,SPEED);
//        digitalWrite(LED_BUILTIN, LOW);
//       }

//       // troubleshooting timer
//       stateavg[0] = (time - T0)/1000000.0;

       if (count++ % 20 ==0) {
        send_values(stateavg, 4);

        if (Serial.available() > 0) {
          delay(200);
                // read the incoming byte:
                incomingByte = Serial.read();

            // say what you got:
            if (incomingByte == 'r') {
              setzero();
              ka = 0;
              ha = 0;
              wrap_k = 0;
              wrap_h = 0;
              state[0] = 0;
              state[1] = 0;
            }

            if (incomingByte == 'k') {
              int ticker = 0;
              while (Serial.available() < 4 && ticker < 30) {
                continue;
                ticker++;
              }
              if (Serial.available() >= 4) {
                k = Serial.parseFloat();
              }
            }

            if (incomingByte == 's') {
              int ticker = 0;
              while (Serial.available() < 4 && ticker < 30) {
                continue;
                ticker++;
              }
              if (Serial.available() >= 4) {
                phik = Serial.parseFloat();
                phih = phik;
              }
            }
//
//            if (incomingByte == 'a') {
//              int ticker = 0;
//              while (Serial.available() < 4 && ticker < 30) {
//                continue;
//                ticker++;
//              }
//              if (Serial.available() >= 4) {
//                state[0] = Serial.parseFloat();
//              }
//            }
//
//            if (incomingByte == 'b') {
//              int ticker = 0;
//              while (Serial.available() < 4 && ticker < 30) {
//                continue;
//                ticker++;
//              }
//              if (Serial.available() >= 4) {
//                state[1] = Serial.parseFloat();
//              }
//            }
        }
       }
}
