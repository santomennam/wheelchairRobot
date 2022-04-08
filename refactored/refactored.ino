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

// Sage Santomenna (MSSM '22) and Dr. Hamlin, 2020-2022
// using libraries:
// Vec2d by Sage Santomenna (for convenient data storage)
// ArduPID by PowerBroker2 (for PID)
//     FireTimer by PowerBroker2 (dependency)
//  Encoder by Paul Stoffregen (guess what this is for)
//  TFmini by Peter Jansen (Time-of-flight distance sensor)
//  L298N by Andrea Lombardo (not currently in use 2/1/22)

// soft e-stop switch
const int estop1 = 6; // OUTPUT LOW
const int estop2 = 7; // pulled up to High.  These pins are connected together with a NC switch    HIGH means STOP
const int modeLED = 4;

void wakeWheelchair()
{
  foreBrainComm.write('w');
  foreBrainComm.write('a');
  foreBrainComm.write('k');
  foreBrainComm.write('e');
  // Serial.println("Woken");
}


//GLOBAL VARIABLES//

bool gotFirstTargets = false;
int threshold = 600; // amount of acceptable error (encoder units: 2400/rotation) //was 1200
int targetIndex = 0;
double cruisingSpeed = 5; // desired crusing speed, in encs per millisecond
int velSwitchThreshold = 6000; //how close should we be, in encoder units, to our target before switching to fine position control?
bool velocityMode = true; // if this is true, we're far away and want to use PID to control our velocity instead of our enc counts
const int numTargs = 4;
long oldPositionBlack = -999;
long oldPositionRed = -999;
double oldTime = millis();
Vec2d oldVals = {0,0};
unsigned long lastTime;
int disconnect = 0;
int timeout = 500;
String data;
bool gotNewLeft = false;
bool gotNewRight = false;
Vec2d targets[numTargs] = {Vec2d(8000, 8000), generateTurn(90), generateTurn(-90), Vec2d(-8000,-8000)};

//= {Vec2d(8000,8000)};

float wheelCircumference = 10.01383; // inches. circ of encoder dummy wheels
int encUnitsRot = 2400;               // encoder units per rotation
float barDiameter = 17;              // inches. this is the distance between the dummy wheels

double inputL = 0;
double inputR = 0;
double targetL;
double targetR;
double outputR = 0;
double outputL = 0;

double pL = 0.03; // 0.05 is pretty close but overshoots a little bit
double iL = 0.005; // 0.0005;
double dL = 0.002;

double pR = 0.05;
double iR = 0.005; //0.0005;
double dR = 0.002;

// takes inches and returns encoder units
int encoders(double distance)
{
  return int(distance * encUnitsRot / wheelCircumference);
}

// takes encoder units and returns inches
float inches(int encs)
{
  return (wheelCircumference * encs / encUnitsRot);
}

// float degrees(double rads)
// {
//   return rads * 57.2958; //conversion factor
// }

float rads(int degrees)
{
  return degrees / 57.2958;
}

float radToArc(float rads)
{ // returns arc in inches
  return barDiameter / 2 * rads;
}

Vec2d generateTurn(int degrees)
{
  int sign = degrees < 0 ? -1 : 1;
  // positive turn is counterclockwise
  degrees = sign * (abs(degrees) % 360); // constrain to +/- 360
  if (abs(degrees) > 180)
  {
    degrees = (360 - abs(degrees)) * -sign; // find complement, flip sign
  }
  float angle = rads(degrees);
  float arc = radToArc(angle);
  double dist = encoders(arc);
  Vec2d motorVals(-dist, dist);
  return motorVals;
}

Vec2d calcVelocities(Vec2d encReadings)
{
  double elapsedTime = millis()-oldTime;
  Vec2d diffEncs = encReadings - oldVals;
  oldVals = encReadings;
  oldTime = millis();
  return diffEncs * (1/(elapsedTime)); 
}

void enableDebug()
{
  foreBrainComm.write('d');
  foreBrainComm.write('b');
  foreBrainComm.write('u');
  foreBrainComm.write('g');
  // Serial.println("Debug enabled");
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

void resetGlobals()
{
  gotFirstTargets = false;
  threshold = 600; 
  targetIndex = 0;
  cruisingSpeed = 5; 
  velSwitchThreshold = 6000; 
  velocityMode = true; numTargs = 4;
  oldPositionBlack = -999;
  oldPositionRed = -999;
  oldTime = millis();
  Vec2d oldVals = {0,0};
  lastTime = unsigned long;
  disconnect = 0;
  timeout = 500;
  data;
  gotNewLeft = false;
  gotNewRight = false;
  targets[numTargs] = {Vec2d(8000, 8000), generateTurn(90), generateTurn(-90), Vec2d(-8000,-8000)};
  inputL = 0;
  inputR = 0;
  targetL;
  targetR;
  outputR = 0;
  outputL = 0;
}

void wheelChairSetup()
{
  gotFirstTargets = false;
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
  resetGlobals();
  wheelChairSetup();
  softwareSetup();
  pinMode(estop1, OUTPUT);
  pinMode(modeLED,OUTPUT);
  digitalWrite(estop1, LOW);
  pinMode(estop2, INPUT_PULLUP);
}

// create variables for wheelchair control


// create variables for sensor control
// int dist; //actual distance measurements of LiDAR

Encoder black(2, 3); // these colors refer to the colors of the 3d printed wheels on the robot as of Feb 2022
Encoder red(18, 19); // black right red left
//   avoid using pins with LEDs attached for encoders

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void checkEstop()
{
  if (digitalRead(estop2) == HIGH)
  {
    setMotorSpeeds(0, 0);
    Serial.println("E-STOP Detected!!!");
    while (true)
    {
      //in an e-stop holding pattern
      delay(100);
      
      //measure
       long newPositionBlack = black.read();
       long newPositionRed = red.read();
      // send
      Serial.print("#");
      Serial.print(0); // output measure distance value of LiDAR, currently hardcoded to 0 for convenience.
      Serial.print(" ");
      Serial.print(newPositionRed);
      Serial.print(" ");
      Serial.println(newPositionBlack);
    }
  }
}

void switchToPos(int leftEncs, int rightEncs) //input readings
{
  velocityMode = false;
  inputL = leftEncs; inputR = rightEncs; 
  PIDControllerL.begin(&inputL, &outputL, &targetL, pL, iL, dL);
  PIDControllerR.begin(&inputR, &outputR, &targetR, pR, iR, dR);
}
void switchToVel()
{
  velocityMode = true;
  inputL = 0; inputR = 0; 
  PIDControllerL.begin(&inputL, &outputL, &cruisingSpeed, pL, iL, dL);
  PIDControllerR.begin(&inputR, &outputR, &cruisingSpeed, pR, iR, dR);
}

void resetEncoders()
{
  red.write(0);
  black.write(0);  
}

int readLidarDist()
{
  int dist = -1;
  int strength; // signal strength of LiDAR

  float temprature;
  int check; // save check value
  int j;
  int uart[9];             // save data measured by LiDAR
  const int HEADER = 0x59; // frame header of data package

  //  read from the time-of-flight distance sensor
  Serial1a.listen(); // set bit rate of serial port connecting LiDAR with Arduino

  bool gotDataRight = false;
  while (!gotDataRight)
  {
    checkEstop();

    if (Serial1a.available())
    { // check if serial port has data input
      if (Serial1a.read() == HEADER)
      { // assess data package frame header 0x59
        uart[0] = HEADER;
        if (Serial1a.read() == HEADER)
        { // assess data package frame header 0x59
          uart[1] = HEADER;
          for (j = 2; j < 9; j++)
          { // save data in array
            uart[j] = Serial1a.read();
          }
          check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
          if (uart[8] == (check & 0xff))
          {                                 // verify the received data as per protocol
            dist = uart[2] + uart[3] * 256; // calculate distance value
            temprature = temprature / 8 - 256;
            unsigned long currentTime = millis();
            Serial1a.end();

            gotDataRight = true;
          }
        }
      }
    }
  }
  return dist;
}
  
void getCommands()
{
  // read data from computer
  while (Serial.available() > 0)
  {
    int dat = Serial.read();
    data.concat(static_cast<char>(dat));
  }

  // format data
  int endIndex = data.indexOf('\n');
  data = data.substring(endIndex + 1);

  if (endIndex != -1)
  {
    if (data.startsWith("debug"))
    {
      debug = !debug;
      lastTime = millis();
      return;
    }

    if (data.startsWith("reset"))
    {
      clearTargets();
      setup();
    }

  else if(data.startsWith("ask")){
    delay(100);
    Serial.println(String("ans # " + String(targetL)+" "+String(targetR)+"\n"));
    delay(100);
    return;
  }
  // get target encoder values for the PID
  else if (data.startsWith("target"))
  {
    
    gotFirstTargets = true;
    data = data.substring(data.indexOf(" ") + 1, -1);
    targetL = (getValue(data, ' ', 0)).toDouble();
    targetR = (getValue(data, ' ', 1)).toDouble();  //these are enc targets

    PIDControllerL.begin(&inputL, &outputL, &targetL, pL, iL, dL);
    PIDControllerR.begin(&inputR, &outputR, &targetR, pR, iR, dR);
    setMotorSpeeds(0,0);
    delay(1000);
    wakeWheelchair();
  }
 }
}

bool closeEnough(int threshold)
{
  if (abs(inputL - targetL) < threshold && abs(inputR - targetR) < threshold)
  {
    return true;
  }
  return false;
}
void loop()
{
  checkEstop();

  getCommands();

  // you'll never guess what these do
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

  //check if we are in the appropriate mode, change if not

  if(closeEnough(velSwitchThreshold))
  {
    if(velocityMode)
    {
      switchToPos(newPositionRed,newPositionBlack);
    }
  }
  else if(!velocityMode)
  {
    switchToVel();
  }

  // use the encoder values we sent as PID inputs, or use velocities
  if(velocityMode)
  {
    digitalWrite(modeLED,HIGH);
    Vec2d vels = calcVelocities(Vec2d{-1.0*newPositionRed,-1.0*newPositionBlack});
    inputL = vels.x;
    inputR = vels.y;
  }
  else{
    inputL = -newPositionRed;
    inputR = -newPositionBlack;
  }
  
  if (!closeEnough(threshold) && gotFirstTargets)
  {
    // compute PID and set motors
    PIDControllerL.compute(); // these will change outputL and outputR by reference, not by return value
    PIDControllerR.compute();
    setMotorSpeeds(outputL, outputR);
  }

  delay(100);
  Serial.print("#");
  Serial.print(0); // output measure distance value of LiDAR, currently hardcoded to 0 for convenience.
  Serial.print(" ");
  Serial.print(-newPositionRed);
  Serial.print(" ");
  Serial.print(-newPositionBlack);
   Serial.print(" ");
  Serial.print(inputL);
   Serial.print(" ");
  Serial.println(inputR);

}
