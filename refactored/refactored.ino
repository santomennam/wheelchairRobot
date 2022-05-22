#include <string.h>
#include <ArduPID.h>
#include <Encoder.h>
#include <FireTimer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <CmdLink.h>
#include <Adafruit_NeoPixel.h>




enum class BotState {
  tank,
  target,
  idle,
  sleep,
  noConnect,
  estop
};

Adafruit_NeoPixel statePixel(1, 10, NEO_GRB + NEO_KHZ800);

void setStateColor(int r, int g, int b)
{
  statePixel.setPixelColor(0, statePixel.Color(r/2, g/2, b/2));
  statePixel.show();
}

Adafruit_8x16matrix matrix = Adafruit_8x16matrix();

ArduPID PIDControllerL;
ArduPID PIDControllerR;

int32_t encThreshold = 300; // amount of acceptable error (encoder units: 2400/rotation) //was 1200

//int heartbeatTime = 200;  // ping or send motor updates to hindbrain at this rate to keep it awake
int commTimeoutTime = 1000;
//int sleepPulseTimeout = 500;

//FireTimer heartbeatTimer;
//FireTimer commTimeout;
//FireTimer sleepingPulse;

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
  sleep_bmp[] =
  { B00000000,
    B01111110,
    B00000100,
    B00001000,
    B00010000,
    B00100000,
    B01111110,
    B00000000,
  },
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
  tank_bmp[] =
  { 
    B00000000,
    B00000000,
    B00010000,
    B00010000,
    B00010000,
    B00010000,
    B00000000,
    B00000000,
  },
  target_bmp[] =
  { B00111100,
    B01010010,
    B10010001,
    B11111111,
    B10010001,
    B01010010,
    B00111100,
    B00000000
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
   else {
      matrix.drawBitmap(x, 0, tank_bmp, 8, 8, LED_ON);      
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
   // checkEstop();

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

///////////////////////////////////


BotState botState{BotState::noConnect};

float tankDriveLeft = 0;
float tankDriveRight = 0;

CmdLink host(Serial, 115200);
CmdLink hindbrain(Serial2, 19200);

bool setMotorSpeeds(int left, int right)
{
  static int lastLeft  = 0;
  static int lastRight = 0;
  
  if (lastLeft != left || lastRight != right) {
    hindbrain.sendCmdBB('M', left, right);
    lastLeft = left;
    lastRight = right;
    drawArrows(left, right);  
    return true;
  }

  return false;
}

//void setMotorSpeedZero()
//{
//  hindbrain.sendCmdBB('M', 0, 0);
//}


bool needAck = false;
bool invalidCmd = false;

void setup()
{
  pinMode(estop1, OUTPUT);
  digitalWrite(estop1, LOW);
  pinMode(estop2, INPUT_PULLUP);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  matrix.begin(0x70); 
  matrix.setRotation(1);
  matrix.setBrightness(5);
  
  matrix.clear();
  matrix.drawBitmap(4, 0, frown_bmp, 8, 8, LED_ON);
  matrix.writeDisplay();  // write the changes we just made to the display

  host.start();
  hindbrain.start();

  statePixel.begin();

  delay(500);

  hindbrain.sendCmdBI('c','R',21);  

  host.recvTimeoutMS = commTimeoutTime;
  host.sendCmdStr('I',"Forebrain");
  
  matrix.clear();
  matrix.drawBitmap(4, 0, frown_bmp, 8, 8, LED_ON);
  matrix.writeDisplay(); 
}

void forwardEncodersToHost()  // NOTE: this assumes we just got a 'C' command from hindbrain!!!
{
  int32_t v1;
  int32_t v2;
  hindbrain.getParam(v1);
  hindbrain.getParam(v2);
  leftEncoder.refresh(v1);
  rightEncoder.refresh(v2);
  host.sendCmdII('C', leftEncoder.getCount(), rightEncoder.getCount());   
}

void reportUnexpectedHindbrainCmd(char c)
{
  host.sendCmdFmt('I', "Unex: %c", c);
  host.sendCmdStr('I', "FroMHind");
}


void reportUnexpectedHostCmd(char c)
{
  host.sendCmdFmt('I', "Unex: %c", c);
  host.sendCmdStr('I', "FromHost");
}

void forwardInfoToHost()
{
  host.sendCmdStr('I', hindbrain.recvBuffer(), hindbrain.recvLen());
}

BotState handleEStopState()
{
//  if (checkEnterEstop()) {
//    return BotState::estop;
//  }
  if (hindbrain.readCmd()) {
    switch (hindbrain.cmd()) {
      case 'S': // stopped/asleep
        return BotState::sleep;
      case 'E': // estop
        return BotState::estop;
      case 'I': 
        forwardInfoToHost();
        break;
     }
  }
  
  return BotState::estop;  
}

BotState handleDisconnectState()
{
//  if (checkEnterEstop()) {
//    return BotState::estop;
//  }
  if (hindbrain.readCmd()) {
    switch (hindbrain.cmd()) {
      case 'S': // stopped/asleep
        return BotState::sleep;
      case 'E': // estop
        return BotState::estop;
      case 'I': 
        forwardInfoToHost();
        break;
     }
  }

  if (host.readCmd()) {
    needAck = true;
    switch (host.cmd()) {
        case 'Q':
          reportState(botState);
          break;
        default:
          host.sendCmdStr('I',"NoConnect");
          break;
    }
  }
  
  return BotState::noConnect;  
}

BotState handleSleepState()
{
//  if (checkEnterEstop()) {
//    return BotState::estop;
//  }
  if (hindbrain.readCmd()) {

    switch (hindbrain.cmd()) {
      case 'C': 
        forwardEncodersToHost();      
        break;
      case 'S': // stopped/asleep
        break;
      case 'w': // waking
      case 's': // stopping
        break;
      case 'W': // awakened   
        return BotState::idle;
      case 'E': // estop
        return BotState::estop;
      case 'I': 
        forwardInfoToHost();
        break;
      default:
        host.sendCmdStr('I',"WTF1");
        reportUnexpectedHindbrainCmd(hindbrain.cmd());
        break;
     }
  }

  if (host.readCmd()) {
    needAck = true;
    switch (host.cmd()) {
      case 'Q':
        reportState(botState);
        break;
      case 'P':
        break;
      case 'W': // wake request
        hindbrain.sendCmd('W');
        break;
      case 'Z': // reset encoders
        hindbrain.sendCmd('Z'); // reset encoders
        break;    
      default:
        invalidCmd = true;
        reportUnexpectedHostCmd(host.cmd());
        break;      
    }
  }

  if (hindbrain.recvTimeout()) {
    // connection to hindbrain lost
    return BotState::noConnect;
  }

  if (hindbrain.sendTimeout()) {
    hindbrain.sendCmd('P');
  }
  
  return BotState::sleep;  
}

BotState handleTankState()
{
//  if (checkEnterEstop()) {
//    return BotState::estop;
//  }

  if (hindbrain.readCmd()) {
    int32_t v1;
    int32_t v2;
    switch (hindbrain.cmd()) {
      case 'C': // encoder count
        forwardEncodersToHost();      
        break;
      case 'S': // stopped/asleep
      case 's': // stopping
        return BotState::sleep;
      case 'W':
        break;
      case 'E': // estop
        return BotState::estop;
      case 'I': 
        forwardInfoToHost();
        break;
      default:
        host.sendCmdStr('I',"WTF2");
        reportUnexpectedHindbrainCmd(hindbrain.cmd());
        break;       
    }
  }

  char c1;
  char c2;
  int32_t  v1;
  int32_t  v2;

  if (host.readCmd()) {
    needAck = true;
    switch (host.cmd()) {
      case 'Q':
        reportState(botState);
        break;
      case 'P':
        break;
      case 'H': // go idle
        return BotState::idle;
      case 'S': // go to sleep
        hindbrain.sendCmd('S');
        return BotState::sleep;
      case 'D': // tank mode
        host.getParam(c1);
        host.getParam(c2);
        tankDriveLeft  = c1;
        tankDriveRight = c2;
        return BotState::tank;
      case 'T': // target 
        botState =  BotState::target;
        host.getParam(v1);
        host.getParam(v2);
        leftEncoder.setTarget(v1);
        rightEncoder.setTarget(v2);
        return BotState::target;
      default:
        invalidCmd = true;
        reportUnexpectedHostCmd(host.cmd());
        break;      
    }
  }

  if (host.recvTimeout()) {
     hindbrain.sendCmd('S');  // tell hindbrain to sleep
     return BotState::idle;
  }

  int motorL = 25 * tankDriveLeft;
  int motorR = 25 * tankDriveRight;
  
  setMotorSpeeds(motorL, motorR);

  if (hindbrain.sendTimeout()) {
    hindbrain.sendCmd('P');
  }
  
  if (hindbrain.recvTimeout()) {
    // connection to hindbrain lost
    return BotState::noConnect;
  }
  
  return BotState::tank;
}

BotState handleTargetState()
{
//  if (checkEnterEstop()) {
//    return BotState::estop;
//  }

  if (hindbrain.readCmd()) {
    int32_t v1;
    int32_t v2;
    switch (hindbrain.cmd()) {
      case 'C': // encoder count
        forwardEncodersToHost();      
        break;
      case 'I': 
        forwardInfoToHost();
        break;
      case 'S': // stopped/asleep
      case 's': // stopping
        return BotState::sleep;
      case 'W':
        break;
      case 'E': // estop
        return BotState::estop;
      default:
        host.sendCmdStr('I',"WTF3");
        reportUnexpectedHindbrainCmd(hindbrain.cmd());
        break;
    }
  }

  char c1;
  char c2;
  int32_t  v1;
  int32_t  v2;

  if (host.readCmd()) {
    needAck = true;
    switch (host.cmd()) {
      case 'Q':
        reportState(botState);
        break;
      case 'P':
        break;
      case 'H': // go idle
        return BotState::idle;
      case 'S': // go to sleep
        hindbrain.sendCmd('S');
        return BotState::sleep;
      case 'D': // tank mode
        host.getParam(c1);
        host.getParam(c2);
        tankDriveLeft  = c1;
        tankDriveRight = c2;
        return BotState::tank;
      case 'T': // target 
        botState =  BotState::target;
        host.getParam(v1);
        host.getParam(v2);
        leftEncoder.setTarget(v1);
        rightEncoder.setTarget(v2);
        break;
      default:
        invalidCmd = true;
        reportUnexpectedHostCmd(host.cmd());
        break;      
    }
  }

  if (host.recvTimeout()) {
     hindbrain.sendCmd('S');
     return BotState::idle;
  }

  int motorL = leftEncoder.computeMotorSpeed();
  int motorR = rightEncoder.computeMotorSpeed();
  
  setMotorSpeeds(motorL, motorR);

  if (hindbrain.sendTimeout()) {
    hindbrain.sendCmd('P');
  }

  if (hindbrain.recvTimeout()) {
    // connection to hindbrain lost
    return BotState::noConnect;
  }
  
  return BotState::target;
}


BotState handleIdleState()
{
//  if (checkEnterEstop()) {
//    return BotState::estop;
//  }

  if (hindbrain.readCmd()) {
    int32_t v1;
    int32_t v2;
    switch (hindbrain.cmd()) {
      case 'C': // encoder count
        forwardEncodersToHost();      
        break;
      case 'S': // stopped/asleep
      case 's': // stopping
        return BotState::sleep;
      case 'w': // waking
      case 'W': // awake ping
        break;
      case 'E': // estop
        return BotState::estop;
      case 'I': 
        forwardInfoToHost();
        break;
      default:
        host.sendCmdStr('I',"WTF4");
        reportUnexpectedHindbrainCmd(hindbrain.cmd());
        break;
      }
  }

  char c1;
  char c2;
  int32_t  v1;
  int32_t  v2;

  if (host.readCmd()) {
    needAck = true;
    switch (host.cmd()) {
      case 'Q':
        reportState(botState);
        break;
      case 'P': // just a ping to keep awake
        break;
      case 'H': // go (remain) idle
        break;
      case 'Z': // reset encoders
        hindbrain.sendCmd('Z'); // reset encoders
        break;
      case 'S': // go to sleep
        hindbrain.sendCmd('S');
        return BotState::sleep;
      case 'D': // tank mode
        host.getParam(c1);
        host.getParam(c2);
        tankDriveLeft  = c1;
        tankDriveRight = c2;
        return BotState::tank;
      case 'T': // target 
        botState =  BotState::target;
        host.getParam(v1);
        host.getParam(v2);
        leftEncoder.setTarget(v1);
        rightEncoder.setTarget(v2);
        return BotState::target;
      default:
        invalidCmd = true;
        reportUnexpectedHostCmd(host.cmd());
        break;      
    }
  }

  if (host.recvTimeout()) {
    hindbrain.sendCmd('S');
    return BotState::idle;
  }

  if (hindbrain.sendTimeout()) {
    hindbrain.sendCmd('P');
  }

  if (hindbrain.recvTimeout()) {
    // connection to hindbrain lost
    return BotState::noConnect;
  }

  return BotState::idle;
}



BotState lastbotState{BotState::idle};

void reportState(BotState state)
{
   switch (botState) {
    case BotState::noConnect:
      host.sendCmdStr('N',"ModeNC");
      break;
    case BotState::sleep:
      host.sendCmdStr('S',"ModeSleep");
      break;
    case BotState::tank:
      host.sendCmdStr('D',"ModeDrive");
      break;
    case BotState::target:
      host.sendCmdStr('T',"ModeTgt");
      break;
    case BotState::idle:
      host.sendCmdStr('W',"ModeIdle");
      break;
    }
  
}

void loop()
{
 // checkEstop();

  switch (botState) {
    case BotState::estop:
      botState = handleEStopState();
      break;
    case BotState::noConnect:
      botState = handleDisconnectState();
      break;
    case BotState::sleep:
      botState = handleSleepState();
      break;
    case BotState::tank:
      botState = handleTankState();
      break;
    case BotState::target:
      botState = handleTargetState();
      break;
    case BotState::idle:
      botState = handleIdleState();
      break;
  }

  if (host.isCorrupt()) {
    host.sendCmdStr('I', "Corrupt");
    host.sendCmdStr('I', host.getCorruptMsg());
    host.sendCmdStr('I', "FromHost");
  }

  if (hindbrain.isCorrupt()) {
    host.sendCmdStr('I', "Corrupt");
    host.sendCmdStr('I', hindbrain.getCorruptMsg());
    host.sendCmdStr('I', "FromHind");
  }

  if (lastbotState != botState) {
    // bot state changed, update displays
    
    lastbotState =  botState;

    beginDraw();

    reportState(botState);

    switch (botState) {
    case BotState::estop:
      matrix.drawBitmap(4, 0, stop_bmp, 8, 8, LED_ON);      
      setStateColor(255,0,0);
      break;
    case BotState::noConnect:
      matrix.drawBitmap(4, 0, frown_bmp, 8, 8, LED_ON);      
      setStateColor(50,0,0);
      break;
    case BotState::sleep:
      setMotorSpeeds(0,0);
      matrix.drawBitmap(4, 0, sleep_bmp, 8, 8, LED_ON);
      setStateColor(0,0,255);
      break;
    case BotState::tank:
      setStateColor(255,255,0);
      break;
    case BotState::target:
      setStateColor(255,0,255);
      break;
    case BotState::idle:
      setMotorSpeeds(0,0);
      setStateColor(0,255,0);
      matrix.drawBitmap(4, 0, smile_bmp, 8, 8, LED_ON);
      break;
    }
  }
    
  updateDisplay();

  if (needAck) { 
    needAck = false;
    if (invalidCmd) {
      invalidCmd = false;
      host.sendCmdStr('k',"Nack"); // nack
    }
    else {
      host.sendCmdStr('K',"Ack"); // ack
    }
  }
}
