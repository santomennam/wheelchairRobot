#include "ArduinoSTL.h"
#include "system_configuration.h"
#include "unwind-cxx.h"
#include "array"
using namespace std;

#include <SoftwareSerial.h>
#include <string.h>
#include <ArduPID.h>
#include <Encoder.h>
#include <Vec2d.h>
ArduPID PIDControllerL;
ArduPID PIDControllerR;

SoftwareSerial Serial1a(10, 9);
SoftwareSerial foreBrainComm(12, 13); // (COMRX, COMTX); //talk from board to robot

bool debug = true;

//Sage Santomenna (MSSM '22) and Dr. Hamlin, 2020-2022
//using libraries:
//Vec2d by Sage Santomenna (for convenient data storage)
//ArduinoSTL by Mike Matera (c++ standard features)
//ArduPID by PowerBroker2 (for PID)
//    FireTimer by PowerBroker2 (dependency)
// Encoder by Paul Stoffregen (guess what this is for)
// TFmini by Peter Jansen (Time-of-flight distance sensor)
// L298N by Andrea Lombardo (not currently in use 2/1/22)

// soft e-stop switch
const int estop1 = 6; // OUTPUT LOW
const int estop2 = 7; // pulled up to High.  These pins are connected together with a NC switch    HIGH means STOP

void wakeWheelchair()
{
  foreBrainComm.write('w');
  foreBrainComm.write('a');
  foreBrainComm.write('k');
  foreBrainComm.write('e');
  //Serial.println("Woken");
}


int threshold = 200; //amount of acceptable error (encoder units: 600/rotation)

int targetIndex = 0;

//this is just how u initialize a vector
// std::vector<Vec2d> targets; //this will hold (targetL,targetR) 

array<Vec2d, 3> targets = { Vec2d(8000,8000), generateTurn(90), generateTurn(-90) };

//= {Vec2d(8000,8000)};


float wheelCircumference = 10.01383; //inches. circ of encoder dummy wheels
int encUnitsRot = 600; //encoder units per rotation 
float barDiameter = 21;//inches. this is the distance between the wheels

double inputL = 0;
double inputR = 0;
double targetL = targets[0].x;
double targetR = targets[0].y;
double outputR = 0;
double outputL = 0;

double pL = 0.037; //0.05 is pretty close but overshoots a little bit
double iL = 0.0005;
double dL = 0.001;

double pR = 0.038;
double iR = 0.00009;
double dR = 0.0015;

//takes inches and returns encoder units
int encoders(double distance) 
{
  return int(distance*encUnitsRot/wheelCircumference);
}

//takes encoder units and returns inches
float inches(int encs) 
{
  return(wheelCircumference*encs/encUnitsRot);
}

// float degrees(double rads)
// {
//   return rads * 57.2958; //conversion factor
// }

float rads(int degrees)
{
  return degrees / 57.2958;
}

float radToArc(float rads){ //returns arc in inches
  return barDiameter/2 * rads;
}

Vec2d generateTurn(int degrees)
{
  int sign = degrees < 0 ? -1 : 1;
  //positive turn is counterclockwise
  degrees = sign * (abs(degrees)%360); // constrain to +/- 360
  if (abs(degrees) > 180){
    degrees = (360-abs(degrees)) * -sign; //find complement, flip sign
  }
  float angle = rads(degrees);
  float arc = radToArc(angle);
  double dist = encoders(arc);
  Vec2d motorVals(-dist,dist);
  return motorVals;
}


void enableDebug()
{
  foreBrainComm.write('d');
  foreBrainComm.write('b');
  foreBrainComm.write('u');
  foreBrainComm.write('g');
  //Serial.println("Debug enabled");
}

void setMotorSpeeds(int left, int right)
{
  char m1 = (char)left;
  char m2 = (char)right;
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
  PIDControllerL.begin(&inputL, &outputL, &targetL, pL, iL, dL);
  PIDControllerR.begin(&inputR, &outputR, &targetR, pR, iR, dR);
  PIDControllerL.setOutputLimits(-35, 35);
  PIDControllerR.setOutputLimits(-35, 35);
}

void softwareSetup()
{
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
//int dist; //actual distance measurements of LiDAR



Encoder black(2, 3); //these colors refer to the colors of the 3d printed wheels on the robot as of Feb 2022
Encoder red(18, 19); //black right red left
//   avoid using pins with LEDs attached for encoders

long oldPositionBlack = -999;
long oldPositionRed   = -999;

unsigned long oldTime = 0;


void checkEstop()
{
  if (digitalRead(estop2) == HIGH)
  {
    setMotorSpeeds(0, 0);
    Serial.println("E-STOP Detected!!!");
    while (true)
    {
      // spin!
    }
  }
}

int readLidarDist()
{
  int dist = -1;
  int strength; //signal strength of LiDAR

  float temprature;
  int check; //save check value
  int j;
  int uart[9];             //save data measured by LiDAR
  const int HEADER = 0x59; //frame header of data package

  //  read from the time-of-flight distance sensor
  Serial1a.listen(); //set bit rate of serial port connecting LiDAR with Arduino

  bool gotDataRight = false;
  while (!gotDataRight)
  {
    checkEstop();

    if (Serial1a.available())
    { //check if serial port has data input
      if (Serial1a.read() == HEADER)
      { //assess data package frame header 0x59
        uart[0] = HEADER;
        if (Serial1a.read() == HEADER)
        { //assess data package frame header 0x59
          uart[1] = HEADER;
          for (j = 2; j < 9; j++)
          { //save data in array
            uart[j] = Serial1a.read();
          }
          check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
          if (uart[8] == (check & 0xff))
          {                                 //verify the received data as per protocol
            dist = uart[2] + uart[3] * 256; //calculate distance value
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
  return dist;
}

void thisCouldBeImproved()
{
    // read data from computer
  while (Serial.available() > 0)
  {
    int dat = Serial.read();
    data.concat(static_cast<char>(dat));
  }

  //format data
  int endIndex = data.indexOf('\n');
  data = data.substring(endIndex + 1);

  if (endIndex != -1)
  {
    if (data.startsWith("debug"))
    {
      debug = !debug;
      if (debug)
      {
        Serial.println("ok debug enabled");
      }
      else
      {
        Serial.println("ok debug off");
      }
      lastTime = millis();
    }
  }

  //get target encoder values for the PID
  else if (data.startsWith("target"))
  {
    data = data.substring(data.indexOf(" ") + 1, -1);

    if (data[0] == 'R')
    {
      targetR = data.substring(data.indexOf(" ") + 1, -1).toDouble();
      Serial.println("Recieved Target " + String(targetR));
    }
    else if (data[0] == 'L')
    {
      targetL = data.substring(data.indexOf(" ") + 1, -1).toDouble();
      Serial.println("Recieved Target " + String(targetL));
    }
    Serial.println("Recieved Target");
  }
}
bool closeEnough(int threshold)
{
  if(abs(inputL-targetL) < threshold && abs(inputR-targetR) < threshold)
  {
     return true;
  }
  return false;
}
void loop()
{
  checkEstop();

  //you'll never guess what these do
  long newPositionBlack = black.read();
  if (newPositionBlack != oldPositionBlack)
  {
     oldPositionBlack = newPositionBlack;
  }

  long newPositionRed = red.read();
  if (newPositionRed != oldPositionRed)
  {
     oldPositionRed = newPositionRed;
  }

  //use the encoder values we sent as PID inputs
  inputL = -oldPositionRed;
  inputR = -oldPositionBlack;

  if(!closeEnough(threshold))
  {
  //compute PID and set motors
  PIDControllerL.compute(); //these will change outputL and outputR by reference, not by return value
  PIDControllerR.compute();

  setMotorSpeeds(outputL, outputR);
  }
  else{
    for(int i = 0; i < 5; i++){
      setMotorSpeeds(0,0);
      delay(200);
    }
    targetIndex++;
    if(targetIndex < targets.size()){
      Vec2d newTargets = targets[targetIndex];
      targetL += newTargets.x;
      targetR += newTargets.y; 
    }
    PIDControllerL.begin(&inputL, &outputL, &targetL, pL, iL, dL);
    PIDControllerR.begin(&inputR, &outputR, &targetR, pR, iR, dR);  
  }
  
  delay(100);
  
  if(debug)
  {
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
}
