#include <SoftwareSerial.h>  //header file of software serial port
#include <Encoder.h>
SoftwareSerial Serial1a(10, 9); //define software serial port name as Serial1a and define pin2 as RX and pin3 as TX
//SoftwareSerial Serial2a(4, 5);
/* For Arduinoboards with multiple serial ports like DUEboard, interpret above two pieces of code and directly use Serial1a serial port*/

//Using libraries
//

int dist; //actual distance measurements of LiDAR
int strength; //signal strength of LiDAR
float temprature;
int check;  //save check value
int i;
int uart[9];  //save data measured by LiDAR
const int HEADER = 0x59; //frame header of data package

void setup() {

  Serial.begin(115200); //set bit rate of serial port connecting Arduino with computer
  Serial1a.begin(115200);
  //Serial2a.begin(115200);
}


// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder black(2, 3);
Encoder red(18, 19);
//   avoid using pins with LEDs attached

long oldPositionBlack  = -999;
long oldPositionRed  = -999;


unsigned long oldTime = 0;

void loop() {
  long newPositionBlack = black.read();
  if (newPositionBlack != oldPositionBlack) {
    oldPositionBlack = newPositionBlack;
    
  }
    long newPositionRed = red.read();
  if (newPositionRed != oldPositionRed) {
    oldPositionRed = newPositionRed;    
  }
  Serial1a.listen();  //set bit rate of serial port connecting LiDAR with Arduino
  bool gotDataRight = false;
  while (!gotDataRight) {
    if (Serial1a.available()) {  //check if serial port has data input
      if (Serial1a.read() == HEADER) { //assess data package frame header 0x59
        uart[0] = HEADER;
        if (Serial1a.read() == HEADER) { //assess data package frame header 0x59
          uart[1] = HEADER;
          for (i = 2; i < 9; i++) { //save data in array
            uart[i] = Serial1a.read();
          }
          check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
          if (uart[8] == (check & 0xff)) { //verify the received data as per protocol
            dist = uart[2] + uart[3] * 256;     //calculate distance value
            temprature = temprature / 8 - 256;
            unsigned long currentTime = millis();
            if(currentTime>oldTime + 100){
            Serial.print("#");
            Serial.print(dist); //output measure distance value of LiDAR
            Serial.print(" ");
            Serial.print(oldPositionBlack);
            Serial.print(" ");
            Serial.println(oldPositionRed);
            }
            Serial1a.end();
            
            gotDataRight = true;
          }
        }
      }
    }
  }
//  bool gotDataLeft = false;
//  while (!gotDataLeft) {
//    Serial2a.listen();
//    if (Serial2a.available()) {  //check if serial port has data input
//      if (Serial2a.read() == HEADER) { //assess data package frame header 0x59
//        uart[0] = HEADER;
//        if (Serial2a.read() == HEADER) { //assess data package frame header 0x59
//          uart[1] = HEADER;
//          for (i = 2; i < 9; i++) { //save data in array
//            uart[i] = Serial2a.read();
//          }
//          check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
//          if (uart[8] == (check & 0xff)) { //verify the received data as per protocol
//            dist = uart[2] + uart[3] * 256;     //calculate distance value
//            Serial.print("left dist = ");
//            Serial.print(dist); //output measure distance value of LiDAR
//            Serial.println('\t');
//            Serial2a.end();
//            gotDataLeft = true;
//          }
//        }
//      }
//    }


  }
//}
