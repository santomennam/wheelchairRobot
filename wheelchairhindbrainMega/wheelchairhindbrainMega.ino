#include <FireTimer.h>
#include <Encoder.h>
#include <Sabertooth.h>
#include <CmdLink.h>
#include <Adafruit_NeoPixel.h>

/*
   From https://www.dimensionengineering.com/datasheets/Sabertooth2x25v2.pdf

  About Ramp

  This adjusts or disables the ramping feature found on the Sabertooth 2x25. This adjustment
  applies to all modes, even R/C and analog mode. Values between 1 and 10 are Fast Ramp;
  values between 11 and 20 are Slow Ramp; values between 21 and 80 are Intermediate Ramp.
  Fast Ramping is a ramp time of 256/(~1000xCommand value). Ramp time is the delay between
  full forward and full reverse speed.
  1: 1/4 second ramp (default)
  2: 1/8 second ramp
  3: 1/12 second ramp
  Slow and Intermediate Ramping are a ramp time of 256/[15.25x(Command value – 10)]
  See Figures 8.1 and 8.2 in the Appendix for a graph of values.

*/

CmdLink forebrain(Serial2, 19200);
CmdLink host(Serial, 115200);

#define SerialSabertooth Serial1

/*
  RJ45 cable goes to two 0.1" connectors (6 pin and 2 pin)

  HindbrainPin  Color           Description                      Notes
  ----------------------------------------------------------------------------------------------------------------------
              White/Orange    5V power                         5V for use by whoever
  15 RX3        Orange          <-------- Serial3 Hindbrain RX   (Future Use)
  14 TX3        White/Green     --------> Serial3 Hindbrain TX   (Future Use)
              Blue            5V Power to estop relay          Provide 5V to this
              White/Blue      Signal to estop relay            High to Drive/Disable Estop
  17 RX2        Green           <-------- Serial2 Hindbrain RX   Control signal from ForeBrain (FOREBRAIN_BAUD)

              Brown           Ground
  16 TX2        White           --------> Serial2 Hindbrain TX   Status to Display/Logger and to ForeBrain (FOREBRAIN_BAUD)


  On the robot, here's a rough guide to the wiring

  Serial1 18 TX1 - White wire to sabertooth

  Serial2 17 RX2 - Blue (of RWB twist) to RJ45   Green                 <------ Control from forebrain
  Serial2 16 TX2 - Yellow to            to RJ4   Brown/White           ------> Data to forebrain

  Serial3 15 RX3 - White (of RWB twist) to RJ45  Orange                <------ future use
  Serial3 14 TX3 - Red (of RWB twist) to RJ45    White/Green           ------> future use

  Digital 41     - Blue -> to yellow Sabertooth EStop on breadboard

  Digital 50     - Yellow to Brake
  Digital 51     - Yellow to Brake

  --------------------

  Breadboard On the robot (Left to right)

  Blue   - EStop -> connected to Red through EStop Relay
  Red    - EStop -> connected to Red through EStop Relay
  White  - Sabertooth Control
  Yellow - Sabertooth EStop (controlled by Blue/Red wires above)   to Pin 41 on arduino
  Yellow - Brake                                                   to Pin 50 on arduino
  Yellow - Brake                                                   to pin 51 on arduino
*/

#define BRAKE_ENABLE_1 50
#define BRAKE_ENABLE_2 51

#define SABER_EMERGENCY_STOP  41

#define LEFT_ENCODER_PIN1  2
#define LEFT_ENCODER_PIN2  3
#define RIGHT_ENCODER_PIN1 20
#define RIGHT_ENCODER_PIN2 21

int MIN_MOTOR_POWER = 5;   // must be in range 1-127                       // commanded power below this threshold is interpreted as a STOP
int MAX_MOTOR_POWER = 40;  // must be in range 1-127 >= MIN_MOTOR_POWER    // commanded power above this threshold is limited to MAX_MOTOR_POWER

const long COMM_TIMEOUT = 300;
const long BRAKE_RELEASE_TIME = 200;  // time between brake release command and brake actually finished releasing
const long BRAKE_ENGAGE_TIME =  200;  // time between brake engage command and brake actually finished engaging

bool emulatorMode   = false;
bool logToHost      = false;
bool logToForebrain = false;
bool softwareReset  = false;

int lastLeftPower   = 0;
int lastRightPower  = 0;

bool forceEstop = false;

int32_t lastEncLeft  = 0;
int32_t lastEncRight = 0;

enum class BotState
{
  asleep,   // motors off, brakes engaged             
  waking,   // motors off, brakes releasing       transition to awake after brake timer expires
  awake,    // brakes off, motors may be running
  stopping, // motors off, brakes engaging        transition to asleep after brake timer expires
  estop,    // estop was triggered, motors off, brakes engaged, can only be brought out of this mode by a reset command
};

BotState botState{BotState::asleep};

Encoder encoderR(RIGHT_ENCODER_PIN1, RIGHT_ENCODER_PIN2); // these colors refer to the colors of the 3d printed wheels on the robot as of Feb 2022
Encoder encoderL(LEFT_ENCODER_PIN2,  LEFT_ENCODER_PIN1);  // black right red left

FireTimer   brakeTimer;
FireTimer   commTimer;

Sabertooth sabertooth(128, SerialSabertooth);

Adafruit_NeoPixel statePixel(1, 10, NEO_GRB + NEO_KHZ800);

void setStateColor(int r, int g, int b)
{
  statePixel.setPixelColor(0, statePixel.Color(r/2, g/2, b/2));
  statePixel.show();
}

void logMessage(const char* msg)
{
  if (logToHost) {
    host.sendCmdStr('I', msg);
  }
  if (logToForebrain) {
   forebrain.sendCmdStr('I', msg);
  }
}

template<typename T>
void logMessage(const char* fmt, T value)
{
  if (logToHost) {
    host.sendCmdFmt('I', fmt, value);
  }
  if (logToForebrain) {
    forebrain.sendCmdFmt('I', fmt, value);
  }  
}

void setMotors(int leftPower, int rightPower)
{
  if (emulatorMode) {
    host.sendCmdBB('M', leftPower, rightPower);
  }
  else {
    sabertooth.motor(1,  leftPower);
    sabertooth.motor(2, -rightPower);
  }
}

void sendEncoders(int32_t leftEnc, int32_t rightEnc)
{
  forebrain.sendCmdII('C', leftEnc, rightEnc);

  if (logToHost) {
    host.sendCmdStr('I', "Encoders");
    host.sendCmdFmt('I', "%d", leftEnc);
    host.sendCmdFmt('I', "%d", rightEnc);
  }
}

void setRamping(int rampVal)
{
  sabertooth.setRamping(rampVal);
}


void setupSabertooth()
{
  SerialSabertooth.begin(9600);
  sabertooth.autobaud(SerialSabertooth);
  setRamping(14);  // 4 second ramp time
}

void brakeOff()
{
  digitalWrite(BRAKE_ENABLE_1, HIGH);
  digitalWrite(BRAKE_ENABLE_2, HIGH);
  digitalWrite(13, LOW);
}

void brakeOn()
{
  digitalWrite(BRAKE_ENABLE_1, LOW);
  digitalWrite(BRAKE_ENABLE_2, LOW);
  digitalWrite(13, HIGH);
}

void resetGlobals()
{
  emulatorMode = false;
  logToHost = false;
  logToForebrain = false;
  lastLeftPower = 0;
  lastRightPower = 0;
  botState = BotState::asleep;
  encoderR.write(0);
  encoderL.write(0);
  brakeOn();
  setRamping(14);
  sabertooth.motor(1,  0);
  sabertooth.motor(2, 0);
  softwareReset  = false;
  forceEstop = false;
  lastEncLeft  = 0;
  lastEncRight = 0;
}

///////////////////////////////////////////////////////////////

void stopBoth()
{
  setMotors(0, 0);
  lastLeftPower  = 0;
  lastRightPower = 0;
}

void stopBothAndBrake()
{
  stopBoth();
  brakeOn();
}

int limitPower(int power)
{
  // deadband
  if ((power <= MIN_MOTOR_POWER) && (power >= -MIN_MOTOR_POWER))
  {
    power = 0;
  }

  power = constrain(power, -MAX_MOTOR_POWER, MAX_MOTOR_POWER);

  return power;
}

bool setBothPower(int leftPower, int rightPower)
{
  bool powerChanged = false;
  leftPower  = limitPower(leftPower);
  rightPower = limitPower(rightPower);

  bool shouldSetMotors = false;

  if (leftPower != lastLeftPower) {
    powerChanged = true;
    lastLeftPower = leftPower;
    shouldSetMotors = true;
  }

  if (rightPower != lastRightPower) {
    powerChanged = true;
    lastRightPower = rightPower;
    shouldSetMotors = true;
  }

  if (shouldSetMotors) {
    setMotors(leftPower, rightPower);
  }

  return powerChanged;
}

void setup()
{
  
  statePixel.begin();

  setStateColor(20,20,20);
  
  host.start();
  forebrain.start();
  
  pinMode(BRAKE_ENABLE_1, LOW);
  pinMode(BRAKE_ENABLE_1, OUTPUT);

  pinMode(BRAKE_ENABLE_2, LOW);
  pinMode(BRAKE_ENABLE_2, OUTPUT);

  pinMode(SABER_EMERGENCY_STOP, INPUT);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
  setupSabertooth();

  setStateColor(50,50,50);

  sendEncoders(0, 0);

  commTimer.begin(COMM_TIMEOUT);

  forebrain.sendInfo("Hindbrain");
  host.sendInfo("Hindbrain");

  botState = BotState::asleep;

}



void updateEncoders(int32_t encLeft, int32_t encRight)
{
  if (encLeft != lastEncLeft || encRight != lastEncRight) {
    lastEncLeft  = encLeft;
    lastEncRight = encRight;
    sendEncoders(encLeft, encRight);
  }
}

void handleConfigCommand(char cmd, int value)
{
  switch (cmd) {
    case 'r': // ramping
      setRamping(value);
      break;
    case 'M': // max power
      if (value >= MIN_MOTOR_POWER && value <= 127) {
        MAX_MOTOR_POWER = value;
        logMessage("Max=%d",value);
      }
      break;
    case 'm': // min power
      if (value >= 0 && value <= MAX_MOTOR_POWER) {
        MIN_MOTOR_POWER = value;
        logMessage("Min=%d",value);
      }
      break;
  }
}

bool checkEnterEstop()
{
  if (forceEstop || digitalRead(SABER_EMERGENCY_STOP) == LOW) {
    logMessage("E Stop");
    stopBothAndBrake();
    forceEstop = false;
    forebrain.sendCmd('S');
    return true;    
  }

  return false;
}


BotState handleSleepState()
{
  if (checkEnterEstop()) {
    return BotState::estop;
  }

  char subCmd;
  int32_t  v1;

  if (host.readCmd()) {
    int32_t encLeft;
    int32_t encRight;
    
    switch (host.cmd()) {
       case 'Q': // query
        host.sendCmdStr('I',"Sleeping");
        break;
      case 'W': // wake
        return BotState::waking;
      case 'R': // softwareReset
        softwareReset = true;
        return BotState::stopping;
      case 'C':
        if (emulatorMode) {
          host.getParam(encLeft);
          host.getParam(encRight);
          sendEncoders(encLeft, encRight);          
        }
        break;
      case 'U': // emulator mode on
        emulatorMode = true;
        host.sendInfo("emulate");
        sabertooth.motor(1, 0);
        sabertooth.motor(2, 0);
        break;
      case 'L': 
        logToHost = true;
        logMessage("LogHost");
        break;
    }
  }  

  if (forebrain.readCmd()) {
    switch (forebrain.cmd()) {
    case 'Q': // query
      forebrain.sendCmdStr('I',"Sleeping");
      break;
    case 'R': // softwareReset
      softwareReset = true;
      return BotState::stopping;
    case 'W': // wake
      return BotState::waking;
    case 'c': // config
      forebrain.getParam(subCmd);
      forebrain.getParam(v1);
      handleConfigCommand(subCmd, v1);
      break;
    case 'L': 
      logToForebrain = true;
      logMessage("LogFB");
      break;
    case 'Z':
      encoderR.write(0);
      encoderL.write(0);
      updateEncoders(0,0);
      break;    
    }
  }

  if (forebrain.sendTimeout())
  { 
     forebrain.sendCmd('S');
     logMessage("Asleep");
  }

  return BotState::asleep;  
}

BotState handleWakingState()
{
  if (checkEnterEstop()) {
    return BotState::estop;
  }

  while (forebrain.readCmd()) 
  {
    switch (forebrain.cmd()) {
    case 'Q': // query
      forebrain.sendCmdStr('I',"Waking");
      break;
    case 'R': // softwareReset
      softwareReset = true;
      return BotState::stopping;
    case 'S': // abort! go back to sleep
      return BotState::stopping;
    case 'Z': // reset encoders
      encoderR.write(0);
      encoderL.write(0);
      updateEncoders(0,0);
      break;
    case 'P': // ping
      break;      
    } 
  }

  if (host.readCmd()) {
    int32_t encLeft;
    int32_t encRight;
    
    switch (host.cmd()) {
      case 'Q': // query
        host.sendCmdStr('I',"Waking");
        break;
      case 'R': // softwareReset
        softwareReset = true;
        return BotState::stopping;
      case 'S': // abort! go back to sleep
        return BotState::stopping;
      case 'C':
        if (emulatorMode) {
          host.getParam(encLeft);
          host.getParam(encRight);
          sendEncoders(encLeft, encRight);          
        }
        break;
    }
  }
  
  if (brakeTimer.fire()) {
    return BotState::awake;
  }

  return BotState::waking;
}

BotState handleAwakeState()
{
  if (checkEnterEstop()) {
    return BotState::estop;
  }

  if (forebrain.recvTimeout())
  {
    return BotState::stopping;
  }

  if (host.readCmd()) {
    int32_t encLeft;
    int32_t encRight;
    
    switch (host.cmd()) {
      case 'Q': // query
        host.sendCmdStr('I',"Awake");
        break;
      case 'R': // softwareReset
        softwareReset = true;
        return BotState::stopping;
      case 'S': // 
        return BotState::stopping;
      case 'C':
        if (emulatorMode) {
          host.getParam(encLeft);
          host.getParam(encRight);
          sendEncoders(encLeft, encRight);          
        }
        break;
      case 'U': // emulator mode on
        emulatorMode = true;
        host.sendInfo("emulate");
        sabertooth.motor(1, 0);
        sabertooth.motor(2, 0);
        break;
    }
  }

  if (forebrain.readCmd()) {
    char m1;
    char m2;

    switch (forebrain.cmd()) {
      case 'Q': // query
        forebrain.sendCmdStr('I',"Awake");
        break;
      case 'P': // keep awake ping
        break;
      case 'R': // softwareReset
        softwareReset = true;
        return BotState::stopping;
      case 'S': 
        return BotState::stopping;
      case 'M': // motor
        forebrain.getParam(m1);
        forebrain.getParam(m2);
        setBothPower(m1, m2);
        break;
      case 'L': 
        logToForebrain = true;
        logMessage("LogFB");
        break;
      case 'Z':
        encoderR.write(0);
        encoderL.write(0);
        updateEncoders(0,0);
        logMessage("EncReset");
        break; 
     }     
  }

  if (forebrain.sendTimeout())
  { 
     forebrain.sendCmd('W');
     logMessage("Awake");
  }

  return BotState::awake;
}

BotState handleStoppingState()
{
  if (checkEnterEstop()) {
    return BotState::estop;
  }
    
  while (forebrain.readCmd()) 
  {
    switch (forebrain.cmd()) {
    case 'Q': // query
      forebrain.sendCmdStr('I',"Stopping");
      break;
    case 'R': // softwareReset
      softwareReset = true;
      break;
    case 'Z': // reset encoders
      encoderR.write(0);
      encoderL.write(0);
      updateEncoders(0,0);
      break;   
    } 
  }

  if (host.readCmd()) {
    int32_t encLeft;
    int32_t encRight;
    
    switch (host.cmd()) {
      case 'Q': // query
        host.sendCmdStr('I',"Stopping");
        break;
      case 'R': // softwareReset
        softwareReset = true;
        break;
      case 'C':
        if (emulatorMode) {
          host.getParam(encLeft);
          host.getParam(encRight);
          sendEncoders(encLeft, encRight);          
        }
        break;
    }
  }

  if (brakeTimer.fire()) {
    return BotState::asleep;
  }

  return BotState::stopping;
}

BotState handleEstopState()
{
  while (host.readCmd()) 
  {
    switch (host.cmd()) {
    case 'Q': // query
      host.sendCmdStr('I',"EStop");
      break;
    case 'R': // softwareReset
      softwareReset = true;
      break;
    }
  }
  
  while (forebrain.readCmd()) 
  {
    switch (forebrain.cmd()) {
    case 'Q': // query
      forebrain.sendCmdStr('I',"EStop");
      break;
    case 'R': // softwareReset
      softwareReset = true;
      break;
    }
  }
  
  if (commTimer.fire())
  { 
     commTimer.begin(COMM_TIMEOUT);          

     if (digitalRead(SABER_EMERGENCY_STOP) != LOW) {
        return BotState::stopping;       
     }
     
     forebrain.sendCmdStr('E', "EStop");
     logMessage("EStop");
  }
  
  return BotState::estop;
}


void loop()
{
  static BotState lastbotState = BotState::estop;
  
  if (!emulatorMode) {
    int32_t encLeft  = encoderL.read();
    int32_t encRight = encoderR.read();
    updateEncoders(encLeft, encRight);
  }

  switch (botState) {
    case BotState::asleep:
      botState = handleSleepState();
      break;
    case BotState::waking:
      botState = handleWakingState();
      break;
    case BotState::awake:
      botState = handleAwakeState();
      break;
    case BotState::stopping:
      botState = handleStoppingState();
      break;
    case BotState::estop:
      botState = handleEstopState();
      break;
  }

  if (lastbotState != botState) {
    lastbotState =  botState;

    switch (botState) {
    case BotState::asleep:
      logMessage("Asleep");
      forebrain.sendCmd('S');
      host.sendCmd('S');
      setStateColor(0,0,255);
      brakeOn();
      commTimer.begin(COMM_TIMEOUT);
      break;
    case BotState::waking:
      logMessage("Waking");
      forebrain.sendCmd('w');
      host.sendCmd('w');
      setStateColor(0,255,255);
      brakeTimer.begin(BRAKE_RELEASE_TIME);
      brakeOff();
      break;
    case BotState::awake:
      logMessage("Awake");
      forebrain.sendCmd('W');
      host.sendCmd('W');
      setStateColor(0,255,0);
      commTimer.begin(COMM_TIMEOUT);
      break;
    case BotState::stopping:
      logMessage("Stopping");
      forebrain.sendCmd('s');
      host.sendCmd('s');
      setStateColor(255,255,0);
      brakeTimer.begin(BRAKE_ENGAGE_TIME);
      stopBoth();
      break;
    case BotState::estop:
      logMessage("Estop");
      forebrain.sendCmd('X');
      host.sendCmd('X');
      setStateColor(255,0,0);
      stopBoth();
      break;
    }
  }

  if (softwareReset && botState == BotState::asleep) {
    resetGlobals();
    softwareReset = false;
    host.sendCmdStr('I',"Reset");
    forebrain.sendCmdStr('I',"Reset");
  }
}
