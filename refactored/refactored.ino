
#include <string.h>
#include <ArduPID.h>
#include <Encoder.h>
#include <FireTimer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <CmdLink.h>

Adafruit_8x16matrix matrix = Adafruit_8x16matrix();

ArduPID PIDControllerL;
ArduPID PIDControllerR;

int32_t encThreshold = 300; // amount of acceptable error (encoder units: 2400/rotation) //was 1200

int brakeReleaseTime = 250;    // time in ms from "wake" to when the brakes should be released and we can begin giving motor commands
int motorHeartbeatTime = 200;  // resend motor values at least this often, even if motor values are unchanged (to keep hindbrain awake)

FireTimer brakeReleaseTimer;
FireTimer motorHeartbeatTimer;
FireTimer commTimeout;

// brakeReleaseTimer.begin(brakeReleaseTime);

//Encoder black(2, 3); // these colors refer to the colors of the 3d printed wheels on the robot as of Feb 2022
//Encoder red(18, 19); // black right red left

//   avoid using pins with LEDs attached for encoders


// Sage Santomenna (MSSM '22) and Dr. Hamlin, 2020-2022
// using libraries:
//  Vec2d by Sage Santomenna (for convenient data storage)
//  ArduPID by PowerBroker2 (for PID)
//  FireTimer by PowerBroker2 (dependency)
//  Encoder by Paul Stoffregen (guess what this is for)
//  TFmini by Peter Jansen (Time-of-flight distance sensor)
//  L298N by Andrea Lombardo (not currently in use 2/1/22)

// soft e-stop switch
const int estop1 = 6; // OUTPUT LOW
const int estop2 = 7; // pulled up to High.  These pins are connected together with a NC switch    HIGH means STOP

static const uint8_t PROGMEM
  smile_bmp[] =
  { B00111100,
    B01000010,
    B10100101,
    B10000001,
    B10100101,
    B10011001,
    B01000010,
    B00111100 },
  stop_bmp[] =
  { B00111100,
    B01000010,
    B10100001,
    B10010001,
    B10001001,
    B10000101,
    B01000010,
    B00111100 },
  arrow_up[] =
  { 
    B00010000,
    B00111000,
    B01010100,
    B10010010,
    B00010000,
    B00010000,
    B00010000,
    B00010000 
  },
  arrow_dn[] =
  { 
    B00010000,
    B00010000,
    B00010000,
    B00010000,
    B10010010,
    B01010100,
    B00111000,
    B00010000 
  },
  frown_bmp[] =
  { B00111100,
    B01000010,
    B10100101,
    B10000001,
    B10011001,
    B10100101,
    B01000010,
    B00111100 };


bool needDisplayUpdate = false;

void beginDraw()
{
  if (!needDisplayUpdate) {
    needDisplayUpdate = true;
    matrix.clear();
  }
}

void updateDisplay()
{
  if (needDisplayUpdate) {
    matrix.writeDisplay();
    needDisplayUpdate = false;  
  }
}

void displayX( uint8_t r, uint8_t g, uint8_t b)
{
  beginDraw();
  matrix.drawLine(0,0, 7, 7, LED_ON);
  matrix.drawLine(7,0, 0, 7, LED_ON);
}

void drawArrow(int x, int value, bool fwd)
{
   beginDraw();
   if (value != 0) {
      matrix.drawBitmap(x, 0, fwd ? arrow_up : arrow_dn, 8, 8, LED_ON);
   }
}

void drawArrows(int leftPower, int rightPower)
{
   bool leftFwd  = leftPower >= 0;
   bool rightFwd = rightPower >= 0;
   drawArrow(0, leftPower, leftFwd);
   drawArrow(8, rightPower, rightFwd);
}

///////////////////////////////////

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
  //Serial3.listen(); // set bit rate of serial port connecting LiDAR with Arduino

  bool gotDataRight = false;
  while (!gotDataRight)
  {
    checkEstop();

    if (Serial3.available())
    { // check if serial port has data input
      if (Serial3.read() == HEADER)
      { // assess data package frame header 0x59
        uart[0] = HEADER;
        if (Serial3.read() == HEADER)
        { // assess data package frame header 0x59
          uart[1] = HEADER;
          for (j = 2; j < 9; j++)
          { // save data in array
            uart[j] = Serial3.read();
          }
          check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
          if (uart[8] == (check & 0xff))
          { // verify the received data as per protocol
            dist = uart[2] + uart[3] * 256; // calculate distance value
            temprature = temprature / 8 - 256;
            unsigned long currentTime = millis();
            Serial3.end();

            gotDataRight = true;
          }
        }
      }
    }
  }
  return dist;
}

///////////////////////////////////


class SmartEncoder {
  private:
    ArduPID& pid;
  private:
    int32_t count{0};
    int32_t prevCount{0};
    int32_t target{0};
    bool hasTarget{false};
    bool usingPid{false};
    double pidInput;
    double pidOutput;
    double pidTarget;
    double kP = 0.002;  // 0.05 is pretty close but overshoots a little bit
    double kI = 0.0005; // 0.0005;
    double kD = 00; // 1000;
  public:
    SmartEncoder(ArduPID& pid) : pid{pid} {}
    bool refresh(int32_t count);
    void reset();
    int32_t getTarget() {
      return target;
    }
    int32_t getCount() {
      return count;
    }
    void setTarget(int32_t targ);
    void setRelativeTarget(int32_t offset) {
      setTarget(target + offset);
    }
    void clearTarget();
    bool getHasTarget() { return hasTarget; }
    int32_t dist() {
      return target - count;
    }
    int32_t absDist() {
      return abs(target - count);
    }
    bool reachedTarget(int32_t threshold) {
      return absDist() <= threshold;
    }
    int  computeMotorSpeed();
    bool isUsingPid() {
      return usingPid;
    }
  private:
    int  fixMotorSpeedSign(int motorSpeed) {
      return (target < count) ? -motorSpeed : motorSpeed;
    }
    void enablePid();
    void disablePid();
};

bool SmartEncoder::refresh(int32_t newCount)
{
  count = newCount;
  if (count != prevCount) {
    prevCount = count;
    return true;
  }
  return false;
}

void SmartEncoder::reset()
{
  count = 0;
  prevCount = 0;
  target = 0;
  clearTarget();
  disablePid();
}

void SmartEncoder::enablePid()
{
  if (!usingPid) {
    usingPid = true;
    pidInput  = count;
    pidTarget = target;
    pid.begin(&pidInput, &pidOutput, &pidTarget, kP, kI, kD);
    pid.setOutputLimits(-30, 30);
    pid.setWindUpLimits(-20, 20);
  }
}

void SmartEncoder::disablePid()
{
  if (usingPid) {
    usingPid = false;
    pid.stop();
  }
}

void SmartEncoder::setTarget(int32_t targ)
{
  target = targ;
  hasTarget = true;
}

void SmartEncoder::clearTarget()
{
  target = count;
  hasTarget = false;
}

int SmartEncoder::computeMotorSpeed()
{
  if (reachedTarget(encThreshold)) {
    disablePid();
    return 0;
  }

  enablePid();
  pid.compute();
  
  return pidOutput;
}

SmartEncoder leftEncoder(PIDControllerL);
SmartEncoder rightEncoder(PIDControllerR);

//bool refreshEncoders()
//{
//  bool leftUpdated  = leftEncoder.refresh();
//  bool rightUpdated = rightEncoder.refresh();
//  return leftUpdated || rightUpdated;  // if you refactor this, be careful about short circuit evaluation... we want both refresh calls to happen!
//}





///////////////////////////////////


int32_t  commTimeoutMs = 500;
bool  commEstablished = false;
bool  waitingForBrakeRelease = false;

bool  tankDriveMode = false;
float tankDriveLeft = 0;
float tankDriveRight = 0;

CmdLink host(Serial, 115200);
CmdLink hindbrain(Serial2, 19200);


void wakeWheelchair()
{
  hindbrain.sendCmd('W');
  brakeReleaseTimer.begin(brakeReleaseTime);
  motorHeartbeatTimer.begin(motorHeartbeatTime);
  waitingForBrakeRelease = true;
}

bool setMotorSpeeds(int left, int right)
{
  static int lastLeft  = 0;
  static int lastRight = 0;
  
  if (motorHeartbeatTimer.fire() || lastLeft != left || lastRight != right) {
    hindbrain.sendCmdBB('M', left, right);
    lastLeft = left;
    lastRight = right;
    drawArrows(left, right);  
    return true;
  }

  return false;
}

void setMotorSpeedZero()
{
  hindbrain.sendCmdBB('M', 0, 0);
}

void wheelChairReset()
{
  setMotorSpeedZero();
  wakeWheelchair();
  leftEncoder.reset();
  rightEncoder.reset();
  tankDriveMode = false;
}

void wheelChairStop()
{
  setMotorSpeedZero();
  leftEncoder.clearTarget();
  rightEncoder.clearTarget();
  tankDriveMode = false;
  beginDraw();
  matrix.drawBitmap(4, 0, stop_bmp, 8, 8, LED_ON);
}

void checkEstop()
{
  if (digitalRead(estop2) == HIGH)
  {
    tankDriveMode = false;
    setMotorSpeeds(0, 0);

    displayX(20,0,0);
    updateDisplay();
    
    while (true)
    {
      //in an e-stop holding pattern
      delay(100);

      bool gotCmd = host.readCmd();

      if (gotCmd)
      {
        if (host.cmd() == 'R') {
          host.sendInfo("Reset");
          wheelChairStop();
          host.sendCmd('K');
          return;
        }
        else {        
          host.sendCmdFmt('I', "Ign: %c", host.cmd());
          host.sendCmd('K'); // ack
        }
      }
    }
  }
}

bool processCommands()
{
  if (!host.readCmd()) {
    return false;
  }

  char c1;
  char c2;
  int32_t  v1;
  int32_t  v2;

  switch (host.cmd()) {
    case 'D': // tank mode
      tankDriveMode = true;
      host.getParam(c1);
      host.getParam(c2);
      tankDriveLeft  = c1;
      tankDriveRight = c2;
      host.sendCmdBB('I',c1, c2);
      break;
    case 'R': // reset
      beginDraw();
      matrix.drawBitmap(4, 0, frown_bmp, 8, 8, LED_ON);
      wheelChairReset();
      host.sendInfo("Reset");
      break;
    case 'T': // target 
      tankDriveMode = false;
      host.getParam(v1);
      host.getParam(v2);
      leftEncoder.setTarget(v1);
      rightEncoder.setTarget(v2);
      host.sendCmdII('T', leftEncoder.getCount(), rightEncoder.getCount());
      break;
    case 'P': // ping
      host.sendInfo("Ping");
      break;
    default:
      host.sendCmdFmt('E', "Unk: %c", host.cmd());
      break;     
  }

  return true;
}


void setup()
{
  pinMode(estop1, OUTPUT);
  digitalWrite(estop1, LOW);
  pinMode(estop2, INPUT_PULLUP);

  matrix.begin(0x70); 
  matrix.setRotation(1);
  matrix.setBrightness(5);
  
  matrix.clear();
  matrix.drawBitmap(4, 0, frown_bmp, 8, 8, LED_ON);
  matrix.writeDisplay();  // write the changes we just made to the display

  host.start();
  hindbrain.start();
  
  delay(500);

  hindbrain.sendCmdBI('c','R',21);  
  host.sendCmdStr('I',"Forebrain");
  
  wheelChairReset();

  matrix.clear();
  matrix.drawBitmap(4, 0, smile_bmp, 8, 8, LED_ON);
  matrix.writeDisplay();  // write the changes we just made to the display
}


void loop()
{
  //checkEstop();

  bool encodersChanged = false;

  if (hindbrain.readCmd()) {
    int32_t v1;
    int32_t v2;
    switch (hindbrain.cmd()) {
      case 'C': // encoder count
        hindbrain.getParam(v1);
        hindbrain.getParam(v2);
        leftEncoder.refresh(v1);
        rightEncoder.refresh(v2);
        encodersChanged = true;
        break;
    }
  }

  bool gotCommand = processCommands();

  if (gotCommand) {
     if (!commEstablished) {
         // established connection to host!
         commEstablished = true;
         wakeWheelchair();
         commTimeout.begin(commTimeoutMs);
     }
     else {
         commTimeout.start();
     }
  }
  else if (commTimeout.fire()) {
     // communication from host timed out!
     commEstablished = false;
     wheelChairStop();
  }
  
  if (!commEstablished) {
    updateDisplay();
    if (gotCommand) {
       host.sendInfo("Unexp");
       host.sendCmd('K'); // ack
    }
    return;
  }

  if (waitingForBrakeRelease) {
    if (brakeReleaseTimer.fire()) {
      waitingForBrakeRelease = false;  
      host.sendCmd('K'); // ack
    }
    else {
      return; // still waiting for release
    }
  }
  
  int motorL = 0;
  int motorR = 0;

  if (tankDriveMode) {
    motorL = 25 * tankDriveLeft;
    motorR = 25 * tankDriveRight;
  }
  else {
    motorL = leftEncoder.computeMotorSpeed();
    motorR = rightEncoder.computeMotorSpeed();
  }

  setMotorSpeeds(motorL, motorR);
    
  updateDisplay();

  if (gotCommand) {
    if (encodersChanged || motorL != 0 || motorR != 0) {
      host.sendCmdII('C', leftEncoder.getCount(), rightEncoder.getCount());
      host.sendCmdBB('M', motorL, motorR);
    }    
    host.sendCmd('K'); // ack
  }
}
