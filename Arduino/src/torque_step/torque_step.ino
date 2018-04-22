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
#define NREF 150
#define BETA 0.85 //0 = filter is off
#define T_OFFSET_K 1.87
#define T_SENSITIVITY_K 7.1
#define T_OFFSET_H 1.85
#define T_SENSITIVITY_H 6.667
#define KP 40 //26
#define KD 0.0015 //0.007
#define ERR_RANGE 3 // how far off from reference is "correct" in degrees
#define PHIK_A 0.6549
#define PHIK_B -11.458
#define PHIK_C 87.148
#define PHIK_D 1797.9
#define PHIH_A 0.6549
#define PHIH_B -11.458
#define PHIH_C 87.148
#define PHIH_D 1797.9
#define MIN_PHI 1800
#define MAX_PHI 2250

// PINS
#define KNEE_ANGLE 3 //Chip or Slave select 
#define HIP_ANGLE 4 //Chip or Slave select
#define LED_PIN 10
#define KNEE_TORQUE A0
#define HIP_TORQUE A1
#define SERVO_KNEE 5
#define SERVO_HIP 6
#define ZERO_RESET 7

// Other
#define SPEED 160
#define CONV_D2R M_PI/180
#define BEGIN_SEND -10000.0
#define END_SEND -5000.0
#define MAX_TORQUE 2.5
#define WRAP_THRESH 45
#define WRAP 90

VarSpeedServo ServoH;
VarSpeedServo ServoK;

float k = 0.05; // UPDATE

// State:
// 0: knee angle
// 1: hip angle
// 2: knee torque
// 3: hip torque
float state[4];
float stateavg[4];

float out[2*NREF];
int i_err;
float qk_err;
float qh_err;
float qk_off;
float qh_off;

float t_err[4]; //knee, hip, knee', hip'
float ttemp;
float phik = MIN_PHI;
float phih = MIN_PHI;
float tk_des = 0;
float th_des = 0;

float deg = 0.00;
int wrap_k = 0;
int wrap_h = 0;
float ka;
float ha;
int i = 0;

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
  pinMode(LED_PIN, OUTPUT);
  
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

  for (i = 0; i < 4; i++) {
    state[i] = 0;
    stateavg[i] = 0;
    t_err[i] = 0;
  }

  time = millis();
  T = time;

  digitalWrite(LED_PIN, HIGH);
  phik = MIN_PHI;
  phih = MIN_PHI;
  run_servo();
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(400);
  digitalWrite(LED_PIN, HIGH);

  // start collecting data
  i = 0;
  while (i < NREF) {
       // take measurements
       //state[0] = (get_angle(KNEE_ANGLE)/4 + WRAP*wrap_k);
       state[2] = get_torque(KNEE_TORQUE, T_OFFSET_K, T_SENSITIVITY_K)-qk_off;
       state[3] = get_torque(HIP_TORQUE, T_OFFSET_H, T_SENSITIVITY_H)-qh_off;
       //state[1] = (get_angle(HIP_ANGLE)/4 + WRAP*wrap_h);
       
       // calculate T for derivatives
       temptime = time;
       time = micros(); // resets to 0 every ~40 min
       // if micros() wraps, T doesn't update. Next iteration time>temptime and T updates
       if (time > temptime) T = (time-temptime)/1000000.0; 

       average(stateavg, state, 4);

       tk_des = (i+1)*10.0/NREF;
       th_des = MAX_TORQUE;

       ttemp = tk_des - abs(stateavg[2]);
       t_err[2] = (t_err[0]-ttemp)/T;
       t_err[0] = ttemp;
       ttemp = th_des - abs(stateavg[3]);
       t_err[3] = (t_err[1]-ttemp)/T;
       t_err[1] = ttemp;

       phik = phi_est(tk_des, PHIK_A, PHIK_B, PHIK_C, PHIK_D) + KP*t_err[0] + KD*t_err[2];
       phih = phi_est(th_des, PHIH_A, PHIH_B, PHIH_C, PHIH_D) + KP*t_err[1] + KD*t_err[3];

       // correct servos
       run_servo();

       if (count++ % 5 == 0) {
         out[2*i] = time;
         out[2*i+1] = stateavg[2];
         i++;
       }

       delay(4);
  }

  digitalWrite(LED_PIN, LOW);
  phik = MIN_PHI;
  phih = MIN_PHI;
  run_servo();
}

float get_torque(int joint, float off, float sens) {
  float adc_output = analogRead(joint);
  float voltage = (5.0/1024)*adc_output;

  float torque = (voltage-off)*sens;

  return torque;
}

void average(float* avg, float* curr, int len) {
  for(int i = 0; i < len; i++) {
    avg[i] = BETA*avg[i]+(1-BETA)*curr[i];
  }
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
  if (Serial.available() > 0) {
    delay(200);
    // read the incoming byte:
    incomingByte = Serial.read();

    // say what you got:
    if (incomingByte == 's') {
      for (int i = 0; i < NREF; i++) {
        delay(60);
        Serial.println(BEGIN_SEND);
        Serial.println(out[2*i]);
        Serial.println(out[2*i+1]);
      }
      Serial.println(END_SEND);
    }
  }
}
