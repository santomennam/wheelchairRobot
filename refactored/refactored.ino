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
const int leftModeLED = 4;
const int rightModeLED = 5;

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
bool leftVelocityMode = true; // if this is true, we're far away and want to use PID to control our velocity instead of our enc counts
bool rightVelocityMode = true;
const int numTargs = 4;

long leftEncoderCount = 0;
long rightEncoderCount = 0;
long prevLeftEncoderCount = 0;
long prevRightEncoderCount = 0;

double oldTime = millis();
Vec2d oldVals = {0, 0};
unsigned long lastTime;
int disconnect = 0;
int timeout = 500;

bool gotNewLeft = false;
bool gotNewRight = false;
Vec2d targets[numTargs] = {Vec2d(8000, 8000), generateTurn(90), generateTurn(-90), Vec2d(-8000, -8000)};
Vec2d encTargets;
//= {Vec2d(8000,8000)};

double wheelSpeedRatio = 1; // ratio of right/left speeds when given the same power

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

String readDelimited(bool flushData)
{
  static bool gotStart = false;
  static String data;
  String result;

  if (flushData) {
    data = "";    
  }

  // read data from computer
  while (Serial.available() > 0)
  {
    int dat = Serial.read();

    if (flushData) {
      continue;
    }
    
    if (dat == '#') {
      gotStart = true;
      data = ""; // clear and
    }
    else if (dat == '\n') {
      if (gotStart) {
        // end of line
        result = data;
        data = "";
        gotStart = false;
            Serial.println("Handling Command: " + data);
        return result;
      }
    }
    else {
      data.concat(static_cast<char>(dat));
      if (data.length() > 100) {
        data = "";
        gotStart = false;
      }
    }


  }

  return result;
}

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
  leftVelocityMode = true;


  
  oldTime = millis();
  Vec2d oldVals = {0, 0};
  lastTime = 0;
  disconnect = 0;
  timeout = 500;
  gotNewLeft = false;
  gotNewRight = false;
  inputL = 0;
  inputR = 0;
  targetL;
  targetR;
  outputR = 0;
  outputL = 0;
  Vec2d encTargets = {0, 0};
}

void wheelChairReset()
{
  gotFirstTargets = false;
  setMotorSpeeds(0, 0);
  wakeWheelchair();
  lastTime = millis();
  PIDControllerL.begin(&inputL, &outputL, &targetL, pL, iL, dL);
  PIDControllerR.begin(&inputR, &outputR, &targetR, pR, iR, dR);
  PIDControllerL.setOutputLimits(-35, 35);
  PIDControllerR.setOutputLimits(-35, 35);
  resetGlobals();
  resetEncoders();
}

void setup()
{
  pinMode(estop1, OUTPUT);
  pinMode(leftModeLED, OUTPUT);
  pinMode(rightModeLED, OUTPUT);
  digitalWrite(estop1, LOW);
  pinMode(estop2, INPUT_PULLUP);
  
  Serial.begin(115200);
  foreBrainComm.begin(9600);
  Serial1a.begin(115200);

  delay(500);
  wheelChairReset();  
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
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}


void resetEncoders()
{
  red.write(0);  // left
  black.write(0);  // right
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  prevLeftEncoderCount = 0;
  prevRightEncoderCount = 0;
}

bool updateEncoders()
{
  bool changed = false;
  
  leftEncoderCount = -red.read();
  rightEncoderCount = -black.read();

  if (leftEncoderCount != prevLeftEncoderCount) {
    changed = true;
    prevLeftEncoderCount = leftEncoderCount;
  }

  if (rightEncoderCount != prevRightEncoderCount) {
    changed = true;
    prevRightEncoderCount = rightEncoderCount;
  }

  return changed;
}


void checkEstop()
{
  if (digitalRead(estop2) == HIGH)
  {
    setMotorSpeeds(0, 0);
    readDelimited(true);
    
    bool first = true;

    while (true)
    {
      //in an e-stop holding pattern
      delay(100);

      bool encChanged = updateEncoders();
      
      if (first || encChanged) {
        Serial.print("#X ESTOP "); // estop
        Serial.print(0); // output measure distance value of LiDAR, currently hardcoded to 0 for convenience.
        Serial.print(" ");
        Serial.print(leftEncoderCount);
        Serial.print(" ");
        Serial.println(leftEncoderCount);
        first = false;
      }

      String command = readDelimited(false);
      
      if (command.startsWith("reset"))
      {
        Serial.println("#R Reset Received");// T indicates target data
        wheelChairReset();
        return;
      }
      else if (command.length() > 0) {
        Serial.println("#I Ignoring: " + command);
      }
    }
  }
}


void leftSwitchToPos(int leftEncs) //input readings
{
  leftVelocityMode = false;
  inputL = leftEncs;
  PIDControllerL.begin(&inputL, &outputL, &targetL, pL, iL, dL);
}

void leftSwitchToVel()
{
  outputR = 0;
  leftVelocityMode = true;
  PIDControllerL.stop();
}

void rightSwitchToPos(int rightEncs) //input readings
{
  outputR = 0;
  rightVelocityMode = false;
  inputR = rightEncs;
  PIDControllerR.begin(&inputR, &outputR, &targetR, pR, iR, dR);
}

void rightSwitchToVel()
{
  outputR = 0;
  rightVelocityMode = true;
  PIDControllerL.stop();
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
          { // verify the received data as per protocol
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
  String command = readDelimited(false);

  if (command.length() == 0) {
    return;
  }

  if (command.startsWith("debug"))
  {
    debug = !debug;
    if (debug) {
      Serial.println("#D Debug ON");
    }
    else {
      Serial.println("#D Debug OFF");
    }
    lastTime = millis();
    return;
  }
  else if (command.startsWith("reset"))
  {
    Serial.println("#R Reset Received");// T indicates target data
    wheelChairReset();
    return;
  }
  else if (command.startsWith("ask")) {
    Serial.println(String("#A " + String(targetL) + " " + String(targetR))); // T indicates target data
    return;
  }
  // get target encoder values for the PID
  else if (command.startsWith("target"))
  {
    gotFirstTargets = true;
    command = command.substring(command.indexOf(" ") + 1, -1);
    targetL = (getValue(command, ' ', 0)).toDouble();
    targetR = (getValue(command, ' ', 1)).toDouble();  //these are enc targets
    encTargets = {targetR, targetL};
    PIDControllerL.begin(&inputL, &outputL, &targetL, pL, iL, dL);
    PIDControllerR.begin(&inputR, &outputR, &targetR, pR, iR, dR);
    Serial.println(String("#T " + String(targetL) + " " + String(targetR))); // T indicates target data
    setMotorSpeeds(0, 0);
    wakeWheelchair();
    return;
  }
  else {
    Serial.println(String("#E Invalid command: ") + command);
  }
}

bool closeEnough(int threshold, int encoder, int encTarget)
{
  if (abs(encoder - encTarget) < threshold)
  {
    return true;
  }
  return false;
}



void loop()
{
  checkEstop();
  //  delay(100);
  getCommands();
  //  delay(100);

  bool encodersChanged = updateEncoders();

 // Vec2d encoders{ leftEncoderCount, rightEncoderCount};
  //check if we are in the appropriate mode, change if not

  if (closeEnough(velSwitchThreshold, leftEncoderCount, encTargets.x))
  {
    if (leftVelocityMode)
    {
      leftSwitchToPos(-1 * leftEncoderCount);
    }
  }
  else if (!leftVelocityMode)
  {
    leftSwitchToVel();
  }


  if (closeEnough(velSwitchThreshold, rightEncoderCount, encTargets.y))
  {
    if (rightVelocityMode)
    {
      rightSwitchToPos(-1 * rightEncoderCount);
    }
  }
  else if (!rightVelocityMode)
  {
    rightSwitchToVel();
  }

  // use the encoder values we sent as PID inputs, or use velocities
  if (leftVelocityMode)
  {
    digitalWrite(leftModeLED, HIGH);
    inputL = 6969;
  }
  else {
    digitalWrite(leftModeLED, LOW);
    inputL = leftEncoderCount;
  }

  if (rightVelocityMode)
  {
    digitalWrite(rightModeLED, HIGH);
    inputR = 6969;
  }
  else {
    digitalWrite(rightModeLED, LOW);
    inputR = rightEncoderCount;
  }

  if ((!closeEnough(threshold, leftEncoderCount,  encTargets.x) || 
       !closeEnough(threshold, rightEncoderCount, encTargets.y)) && gotFirstTargets) //potentially change this to be by wheel instead of activating both on the or
  {
    if (leftVelocityMode)
    {
      outputL = 35 * wheelSpeedRatio;
    }
    else {
      PIDControllerL.compute(); // these will change outputL and outputR by reference, not by return value
    }

    if (rightVelocityMode)
    {
      outputR = 35;
    }
    else {
      PIDControllerR.compute(); // these will change outputL and outputR by reference, not by return value
    }
    setMotorSpeeds(outputL, outputR);
  }

  delay(100);
  if (encodersChanged) {
    Serial.print("#P ");  // P indicates position data
    Serial.print(0); // output measure distance value of LiDAR, currently hardcoded to 0 for convenience.
    Serial.print(" ");
    Serial.print(leftEncoderCount);
    Serial.print(" ");
    Serial.print(rightEncoderCount);
    Serial.print(" ");
    Serial.print(inputL);
    Serial.print(" ");
    Serial.println(inputR);
  }
  else {
    Serial.println("#H"); // heartbeat
  }

}
