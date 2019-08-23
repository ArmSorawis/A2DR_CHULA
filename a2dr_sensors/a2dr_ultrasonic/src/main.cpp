#include <Arduino.h>

// void setup() {
//   // put your setup code here, to run once:
// }

// void loop() {
//   // put your main code here, to run repeatedly:
// }
////initial SoftwareSerial
//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(10, 11); // RX, TX

const int ID = 0x02;

//initial PIn
const int trigPinL = 5;
const int echoPinL = 6;
const int trigPinR = 9;
const int echoPinR = 10;




const int timeOut = 5500;// 80 cm
const int timeDelay = 50; //20Hz



//*********************************************************//
//                         Variable                        // 
//*********************************************************//
long durationL;
long durationR;
int distanceL = 0;
uint8_t distanceR = 0;
int buf[5] = {0xFF, 0xFF, ID, 0, 0};
//uint8_t buf[5] = {0xFF, 0xFF, ID, 0x05, 0x25};
void sendData();

//*********************************************************//
//                        void setup                       // 
//*********************************************************//
void setup() {
  // set serial
  Serial.begin(115200);
  
  // set pin
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  
}// setup()


//*********************************************************//
//                        void Loop                        // 
//*********************************************************//
void loop() {

 
  digitalWrite(trigPinL, LOW);
  delayMicroseconds(2);
 
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);
  durationL = pulseIn(echoPinL, HIGH, timeOut);


  digitalWrite(trigPinR, LOW);
  delayMicroseconds(2);
 
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinR, LOW);
  durationR = pulseIn(echoPinR, HIGH, timeOut);
  
  buf[3] = durationL * 0.034 / 2;
  buf[4] = durationR * 0.034 / 2;

  sendData();
  
//  Serial.println(buf[3]);
//  Serial.print(duration);


}// void Loop()

//*********************************************************//
//                         Function                        // 
//*********************************************************//
void sendData(){
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(ID);
  Serial.write(buf[3]);
  Serial.write(buf[4]);
  delay(timeDelay);
} //sendData()