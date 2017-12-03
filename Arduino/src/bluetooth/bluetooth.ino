char INBYTE;
int  LED = 13; // LED on pin 13

void setup() {
  Serial.begin(9600); 
  pinMode(LED, OUTPUT);
}

void loop() {
  Serial.println("Press 1 to turn Arduino pin 13 LED ON or 0 to turn it OFF:");
  while (!Serial.available());   // stay here so long as COM port is empty   
  
}

void read() {
  INBYTE = Serial.read();        // read next available byte
  if( INBYTE == '0' ) digitalWrite(LED, LOW);  // if it's a 0 (zero) tun LED off
  if( INBYTE == '1' ) digitalWrite(LED, HIGH); // if it's a 1 (one) turn LED on
}

void write(float angle) {
  Serial.println(angle);
}

void serialEvent1() {
  while (Serial.available()) {
    read();
    }
  }

