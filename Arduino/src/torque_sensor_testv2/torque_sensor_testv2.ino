#define DATA A1
#define OFFSET 1.85
#define SENSITIVITY (8*6/5.6)
int speed = 9600;    //data rate in bits per second
int test;
int i = 0;
float voltage;
float torque;
float avg;


void setup() {
  // put your setup code here, to run once:
 pinMode(DATA, INPUT);
 pinMode(LED_BUILTIN, OUTPUT);
 digitalWrite(DATA, LOW);
 while(!Serial);
 Serial.begin(speed);
 avg = 0;
}

void loop() {
  delay(50);
  // put your main code here, to run repeatedly:
  test = analogRead(DATA);
  voltage = (5.0/1024)*test;
  
  torque = (voltage-OFFSET)*SENSITIVITY;

  avg = 0.9*avg+0.1*torque;
  i++;
  Serial.println(voltage);
}
