using namespace std;
#include <SoftwareSerial.h>
#include <string.h>
#include <ArduPID.h>
#include <Encoder.h>
ArduPID PIDControllerL;
ArduPID PIDControllerR;
SoftwareSerial Serial1a(10, 9);
SoftwareSerial foreBrainComm(12, 13); // (COMRX, COMTX); //talk from board to robot
bool debug = true;
//Sage Santomenna (MSSM '22) and Dr. Hamlin, 2020-2022
//using libraries:
// ArduPID by PowerBroker2 (for PID)
//    FireTimer by PowerBroker2 (dependency)
// Encoder by Paul Stoffregen (guess what this is for)
// TFmini by Peter Jansen (Time-of-flight distance sensor)
// L298N by Andrea Lombardo (not currently in use 2/1/22)

// soft e-stop switch
const int estop1 = 6;  // OUTPUT LOW
const int estop2 = 7;  // pulled up to High.  These pins are connected together with a NC switch    HIGH means STOP

void wakeWheelchair()
{
  foreBrainComm.write('w');
  foreBrainComm.write('a');
  foreBrainComm.write('k');
  foreBrainComm.write('e');
  Serial.println("Woken");
}
double inputL = 0;
double inputR = 0;
double targetL = 4000;
double targetR = 4000;
double outputR = 0;
double outputL = 0;

double p = 0.001;
double i = 0.001;
double d = 0;

void enableDebug()
{
  foreBrainComm.write('d');
  foreBrainComm.write('b');
  foreBrainComm.write('u');
  foreBrainComm.write('g');
  Serial.println("Debug enabled");
}
void setMotorSpeeds(int left, int right)
{
  char m1 = (char)left;
  char m2 = (char)right;
//  if (debug) {
//    Serial.print("left ");
//    Serial.print(left);
//    Serial.print(" right ");
//    Serial.println(right);
//  }
  foreBrainComm.write('m');
  foreBrainComm.write(m1);
  foreBrainComm.write(m2);
  foreBrainComm.write('x');
}

unsigned long lastTime;
void wheelChairSetup()
{
  Serial.begin(115200);
  foreBrainComm.begin(9600);
  delay(1000);
  wakeWheelchair();
  lastTime = millis();
  PIDControllerL.begin(&inputL, &outputL, &targetL, p, i, d);
  PIDControllerR.begin(&inputR, &outputR, &targetR, p, i, d);
  PIDControllerL.setOutputLimits(-35, 35);
  PIDControllerR.setOutputLimits(-35, 35);
}

void softwareSetup() {
  Serial1a.begin(115200);
}
void setup()
{
  wheelChairSetup();
  softwareSetup();
  pinMode(estop1, OUTPUT);
  digitalWrite(estop1, LOW);
  pinMode(estop2, INPUT_PULLUP);
}

//create variables for wheelchair control
int disconnect = 0;
int timeout = 500;
String data;

//create variables for sensor control
int dist; //actual distance measurements of LiDAR

int strength; //signal strength of LiDAR

float temprature;
int check;  //save check value
int j;
int uart[9];  //save data measured by LiDAR
const int HEADER = 0x59; //frame header of data package

Encoder black(2, 3); //these colors refer to the colors of the 3d printed wheels on the robot as of Feb 2022
Encoder red(18, 19); //black right red left
//   avoid using pins with LEDs attached for encoders

long oldPositionBlack  = -999;
long oldPositionRed  = -999;
unsigned long oldTime = 0;

void checkEstop() 
{
    if (digitalRead(estop2) == HIGH) {
        setMotorSpeeds(0,0);
        Serial.println("E-STOP Detected!!!");
        while (true) {
           // spin!
        }
    }
}
void loop()
{
//  while (true) {
//    setMotorSpeeds(-39, -39);
//    Serial.println("Boop");
//    delay(300);
//    checkEstop();
//  }  

//  seems like the motors only work between -39, -20 and 20, 40
  //SENSORS
   checkEstop();
  
//you'll never guess what these do
  long newPositionBlack = black.read();
  if (newPositionBlack != oldPositionBlack) {
    oldPositionBlack = newPositionBlack;
  }
  
  long newPositionRed = red.read();
  if (newPositionRed != oldPositionRed) {
    oldPositionRed = newPositionRed;
  }

//  read from the time-of-flight distance sensor 
  Serial1a.listen();  //set bit rate of serial port connecting LiDAR with Arduino
  bool gotDataRight = false;
  while (!gotDataRight) {
    checkEstop();
    
    if (Serial1a.available()) {  //check if serial port has data input
      if (Serial1a.read() == HEADER) { //assess data package frame header 0x59
        uart[0] = HEADER;
        if (Serial1a.read() == HEADER) { //assess data package frame header 0x59
          uart[1] = HEADER;
          for (j = 2; j < 9; j++) { //save data in array
            uart[j] = Serial1a.read();
          }
          check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
          if (uart[8] == (check & 0xff)) { //verify the received data as per protocol
            dist = uart[2] + uart[3] * 256;     //calculate distance value
            temprature = temprature / 8 - 256;
            unsigned long currentTime = millis();

//do the sending
//            if (currentTime > oldTime + 100) {
//              oldTime = millis();
//              Serial.print("#");
//              Serial.print(dist); //output measure distance value of LiDAR
//              Serial.print(" ");
//              Serial.print(oldPositionBlack);
//              Serial.print(" ");
//              Serial.println(oldPositionRed);
//            }
            Serial1a.end();

            gotDataRight = true;
          }
        }
      }
    }
  }


// WHEELCHAIR

//increment the amount of disconnect
  disconnect ++;

// read data from computer
  while (Serial.available() > 0) {
    disconnect = 0;
    int dat = Serial.read();
    data.concat(static_cast<char>(dat));
  }
//format data
  int endIndex = data.indexOf('\n');
  data = data.substring(endIndex + 1);

  if (endIndex != -1) {
    if (data.startsWith("debug")) {
      debug = !debug;
      if (debug)
      {
        Serial.println("ok debug enabled");
      }
      else {
        Serial.println("ok debug off");
      }
      lastTime = millis();
    }
  }

//get target encoder values for the PID
    else if (data.startsWith("target")) {
      Serial.println("trying to set target");
      data =  data.substring(data.indexOf(" ") + 1, -1);

      if (data.substring(0, 1) == "R") {
        targetR = data.substring(data.indexOf(" ") + 1, -1).toDouble();
        Serial.println("Recieved Target " + String(targetR));
        PIDControllerR.begin(&inputR, &outputR, &targetR, p, i, d);
      }

      else if (data.substring(0, 1) == "L") {
        targetL = data.substring(data.indexOf(" ") + 1, -1).toDouble();
        Serial.println("Recieved Target " + String(targetL));
        PIDControllerL.begin(&inputL, &outputL, &targetL, p, i, d);
      }
      Serial.println("Recieved Target");
      lastTime = millis();
    }

//use the encoder values we sent as PID inputs
    oldPositionRed = -oldPositionRed;
    inputL = oldPositionRed;
    oldPositionBlack = -oldPositionBlack;
    inputR = oldPositionBlack;

//compute PID and set motors
  PIDControllerL.compute(); //these will change outputL and outputR by reference, not by return value
  PIDControllerR.compute(); 
  setMotorSpeeds(outputL, outputR);
  delay(100);
  if (targetL != 0 || targetR != 0) { //this might be a timeout issue if we actually do try to have encoders at 0,0
    lastTime = millis();
  }
  if (debug) {
    Serial.print("Left: ");
    Serial.print(outputL);
    Serial.print(" Right: ");
    Serial.print(outputR);
    Serial.print(" InputL: ");
    Serial.print(inputL);
    Serial.print(" InputR: ");
    Serial.print(inputR);
    Serial.print(" TargetL: ");
    Serial.print(targetL);
    Serial.print(" TargetR: ");
    Serial.print(targetR);
    Serial.print("\n");
  }

//  check for disconnect
  if (millis() - lastTime >= timeout) {
    while (true) {
      Serial.println("ERROR Disconnect");
      setMotorSpeeds(0, 0);
      delay(200);
      checkEstop();
    }
  }
}
