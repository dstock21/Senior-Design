/*
SSI (Serial Synchron Interface) for the Kübler GmbH rotary absolute multiturn encoder 5862 series, see www.kuebler.com. 
Hardware specifications: 
* 25-bit rotary absolute multiturn encoder
* Interface: SSI (with RS485 line definitions)
* encoded as: Gray-Code

* SSI:
 - SSI-Clock TicTac must be equal or less 10µs. 
 - 26 Clock TicTac are needed for the whole Position consiting of a 25 Bit word, the first TicTac is needed for he status bit, which enconde the status of the device, (0) for not clear. 
 - After the the 26 TicTacs a puse of max. 80µs is needed for a new Position to be available.

* This Program:
 - emulates a SSI with the Arduino Ports 12 (clock) and 11 (data).
 - transform the gray encoded Position to binary encoded Position
 - send the Position to Max/MSP/Jitter via USB
 
* Electronics needed:
 - 2 RS485 line drivers (e.g. sn75176) are needed, if you want to save ports and clean up the ignals.
 - for the start-sequence and the reset a uln 2803 could used (not implemented yet).
 - a few resitors for cleaning the singnals.... 
*/


/*
Variables
*/
byte getByte;         //dummy Byte for serialAvailable
#define SCL 30
#define NSL 50
#define DATA 26 
byte dummy;           //dummy
int i = 0;            //loop counter
int j = 0;            //loop counter
int test;
int counter = 0;
float angle = 0;
int speed = 9600;    //data rate in bits per second
const int numOfBits = 17;   // number of bits per angle
byte grayArray[numOfBits];   //Array reseved for the recieved Gray Code
byte binaryArray[numOfBits]; //Array reserved for the binary transformed Code

/*
functionprototypes
*/
//void getPosition();               // saves the status bit and the 25 position bits in the grayArray, read from the register as PINB. The only time-critical process.
//void condensePosition();          // condenses the recieved Byte (PINB) in grayArray to the only intersting bit (PB3) in PINB.
//void transformPosition();         // transforms the 25 postion bits to binary and saves them in binaryArray
//void sendPosition();              // sends the binarayArray - enclosed in '255' - via USB to Max/MSP/Jitter
////void resetRoutine();            // external Reset of the encoder, to select rotary direction and define zero Position.

/*
setup
*/
void setup()
{
 pinMode(SCL, OUTPUT);
 pinMode(DATA, INPUT);
 pinMode(NSL, OUTPUT);
 pinMode(LED_BUILTIN, OUTPUT);
 digitalWrite(DATA, LOW);
 SerialUSB.begin(speed);  
 SerialUSB.println("This is a test/n");
}

/*
Main
*/
void loop()

{
  if (counter > numOfBits) {
    counter = 0;
    angle = angle/131071.0 * 360;
    SerialUSB.println("Angle: ");
    SerialUSB.println(angle);
    digitalWrite(NSL, LOW);
  }
  
  digitalWrite(SCL, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1);
  digitalWrite(SCL, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  test = digitalRead(DATA);
  angle = angle + test * pow(2, numOfBits - counter);
  SerialUSB.print("LOW: ");
  SerialUSB.println(counter);
  SerialUSB.println(test);
  SerialUSB.print("Angle: ");
  SerialUSB.println(counter); 
  SerialUSB.println(angle);
  delay(1);
  counter++;
  if (counter == 1) {
    digitalWrite(NSL, HIGH);
    angle = 0;
  }
  
// if(PINB & (1<<PB3))
// {
// getPosition();  
// }
//            // delay for device to become ready again 
// if(Serial.available())            // check if a request form max is there. To avoid a buffer overrun, MAX/MSP/Jitter is playing PingPong with the arduino Board, if max send any via sierial, arduino responds with the newest Position
// {
//   while(Serial.available() > 0)   // reads all the data sent to the arduino board
//   {
//     dummy = Serial.read();
//   }                               // and finaly sends the Position
//   condensePosition();
//   transformPosition();
//   sendPosition();
// }
// delay(2);
}


/*
function definitions
*/

//void getPosition()
//{
// for(j = 0;j < 26; j++)          // Untested yet
// {
//    grayArray[j] = 0;
//    binaryArray[j] = 0; 
// }
//
// delayMicroseconds(40);
// 
// for(j = 0;j < 26; j++)
// {
//   PORTB = (0<<PB4);
//   delayMicroseconds(3);
//   grayArray[j]= PINB;  //delayMicroseconds(3) fits a 10us tictac
//   PORTB = (1<<PB4);
//   delayMicroseconds(3);   
// }
// 
// delayMicroseconds(100);
// 
//}
//
//void condensePosition()
//{
//   for(i = 0;i < numOfBits; i++)
//   {
//     if(grayArray[i] & (1<<PB3))
//     {
//       grayArray[i] = 1;
//     }
//     
//     else
//     {
//       grayArray[i] = 0;
//     }
//   }
//}
//
//void transformPosition()
//{
//   binaryArray[0]= grayArray[0];                          // transfer status bit
//   binaryArray[1] = grayArray[1];                         // gray to binary, transfer the MSB
//   for(i = 2; i < numOfBits; i++)                                // all other bits are transformed according to: binaryArray[i] = XOR(grayArray[i], binaryArray[i - 1]), cf. http://www.faqs.org/faqs/ai-faq/genetic/part6/section-1.html
//   {
//     binaryArray[i]= grayArray[i] ^ binaryArray[i - 1]; 
//   } 
//}
//
//void sendPosition()                                        //This function just sends the raw data (not packed in 4 bytes)- encolsed in  a '255'-masking -  to the serial Object of MAX/MSP/Jitter
//{
//   Serial.write(255);
//   for(i = 0; i < numOfBits; i++)
//   {
//     Serial.write(binaryArray[i]);
//     binaryArray[i] = 0;
//     grayArray[i] = 0; 
//   }
//   
//   Serial.write(128);
//}

/*
void resetRoutine() //to be filled for external reset via max/msp
{

 
}
*/


