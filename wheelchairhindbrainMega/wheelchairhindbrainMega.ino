#include <FireTimer.h>
#include <Encoder.h>
#include <Sabertooth.h>
#include <CmdLink.h>

// 4/17/2022

// host <=> hindbrain communications @115200
//
// commands are single char:
//     M  enable mirror mode (all communication between hindbrain/forebrain are echoed to host
//     N  disable mirror mode
//     E  force E-Stop
//     P  ping (just says Hi back)
//   proposed
//     D  disable motors/brakes (allows comms to continue, but nothing is sent to motor drivers)


// forebrain <=> hindbrain communications  @19200
// All commands from host are 4 bytes long with no delimiters or \n's   (except ramp command)
//
//      wake - used to transition from Asleep
//      dbug - recognized but not used?
//      m##x - send motor values (## are left/right motor values as signed bytes, not characters
//      stop - stop motors and go to sleep
//      estp - simulate estop
//      ramp###\n     use this in sleep mode to set ramping (###) is decimal, not bytes like motor values
//      maxp###\n     use this in sleep mode to set max motor power (###) is decimal, not bytes like motor values
//      minp###\n     use this in sleep mode to set min motor power (deadband) (###) is decimal, not bytes like motor values
//
//      values for ramp are kind of odd:
//      ramp1\n is 0.25 sec
//      ramp2\n is 0.125 sec
//      ramp14\n is 4 sec
//      ramp18\n is 2 sec
//      ramp21\n is 1.5 sec
//      ramp26\n is 1 sec
//      ramp31\n is 0.8 sec
//      ramp43\n is 0.5 sec
//      ramp77\n is 0.25 sec


// Status messsages/responses are longer (x bytes max?) and have \n
// Commands from host are ignored/discarded in certain circumstances
//    - when asleep, only wake or dbug are
// Status messages
// On startup,
//    Setup -> Zzzz...  (brake on)
//
// On "wake" command (when Zzzz...)
//    Waking -> BrakeOff  (once BrakeOff is received, motor commands will be processed)
//
// On communication timeout from awake/ready state (BrakeOff)
//    Timeout -> [Stopped] -> BrakeOn -> Hold -> Zzzz...    (Stopped is only sent if motors were on)
//
// On "stop" command (when awake/ready state (BrakeOff)
//    Stop -> [Stopped] -> BrakeOn -> Hold -> Zzzz...
//
// On Bad Command
//    Bad Cmd -> [Stopped] -> BrakeOn -> 'c ### e' -> '1 ### x' -> '2 ### a' -> 'x ### m' -> Hold -> Zzzz...
//
// On Motor command ( m##x )
//   [ ### ###]   new motor values, but only if changed
//
// On real emergency stop signal
//   E Stop -> [ Stopped ] -> BrakeOn -> E Stop 4 -> E Stop 3 -> E Stop 2 -> E Stop 1 -> Hold -> Zzzz...
//
// On "estp" command
//   MEStop -> E Stop -> [ Stopped ] -> BrakeOn -> E Stop 4 -> E Stop 3 -> E Stop 2 -> E Stop 1 -> Hold -> Zzzz...
//
// on "ramp###\n" command
//   Ramp ###   OR  Ramp Bad  (if out of range)
//
// on "maxp###\n" command
//   MaxP ###   OR  MaxP Bad  (if out of range)
//
// on "minp###\n" command
//   MinP ###   OR  MinP Bad  (if out of range)

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
CmdLink logger(Serial3, 19200);

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

  Serial2 17 RX2 - Blue (of RWB twist) to RJ45   Green                 <------ Control Hindbrain through this
  Serial2 16 TX2 - Yellow to ?

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

long COMM_TIMEOUT = 1000;

bool debugMode      = false;
bool mirrorMode     = false;

char tmp[12]; // logger.sendInfo buffer

int         lastLeftPower = 0;
int         lastRightPower = 0;


Encoder encoderR(RIGHT_ENCODER_PIN1, RIGHT_ENCODER_PIN1); // these colors refer to the colors of the 3d printed wheels on the robot as of Feb 2022
Encoder encoderL(LEFT_ENCODER_PIN1,  LEFT_ENCODER_PIN2);  // black right red left

bool emulatorMode   = false;

FireTimer   commTimeout;


Sabertooth sabertooth(128, SerialSabertooth);


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
  if (mirrorMode) {
    host.sendCmdII('C', leftEnc, rightEnc);
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


////////////////////////////////////// Brake Management ///////////////////////////////

enum class BrakeState {
  engaged,
  releasing,
  released,
  engaging
};

const long BRAKE_RELEASE_TIME = 200;  // time between brake release command and brake actually finished releasing
const long BRAKE_ENGAGE_TIME =  200;  // time between brake engage command and brake actually finished engaging

FireTimer   brakeTimer;
BrakeState  brakeState = BrakeState::engaged;

void brakeOff()
{
  digitalWrite(BRAKE_ENABLE_1, HIGH);
  digitalWrite(BRAKE_ENABLE_2, HIGH);

  switch (brakeState) {
    case BrakeState::released:   // already released, do nothing
    case BrakeState::releasing:  // already being released, do nothing
      break;
    case BrakeState::engaged:
    case BrakeState::engaging:
      brakeTimer.begin(BRAKE_RELEASE_TIME);
      brakeState = BrakeState::releasing;
      //logger.sendInfo("Release");
      break;
  }
}

void brakeOn()
{
  digitalWrite(BRAKE_ENABLE_1, LOW);
  digitalWrite(BRAKE_ENABLE_2, LOW);

  switch (brakeState) {
    case BrakeState::released:
    case BrakeState::releasing:
      brakeTimer.begin(BRAKE_ENGAGE_TIME);
      brakeState = BrakeState::engaging;
      logger.sendInfo("BrakeOn");
      break;
    case BrakeState::engaged:
    case BrakeState::engaging:
      break;
  }
}

void updateBrakes()
{
  switch (brakeState) {
    case BrakeState::engaged:
      break;
    case BrakeState::releasing:
      if (brakeTimer.fire()) {
        brakeState = BrakeState::released;
        logger.sendInfo("BrakeOff");
      }
      break;
    case BrakeState::released:
      break;
    case BrakeState::engaging:
      if (brakeTimer.fire()) {
        brakeState = BrakeState::engaged;
        //logger.sendInfo("---engaged");
      }
      break;
  }
}

///////////////////////////////////////////////////////////////


void disableMotors()
{
    setMotors(0, 0);
    digitalWrite(BRAKE_ENABLE_1, LOW);
    digitalWrite(BRAKE_ENABLE_2, LOW);
}

void stopBoth()
{
    setMotors(0, 0);

  if (lastLeftPower != 0 || lastRightPower != 0) {
    logger.sendInfo("Stopped");
  }
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

  if (brakeState != BrakeState::released) {
    power = 0;
  }

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



void printChar(char prefix, int ch)
{
  if ((ch < 32) || (ch > 126)) {
    sprintf(tmp, "%c %3d  ", prefix, ch, ch);
  }
  else {
    sprintf(tmp, "%c %3d %c", prefix, ch, ch);
  }
  logger.sendInfo(tmp);
}


enum class ControlState
{
  asleep,
  awake,
  timeout,
  estop,
  badcmd
};

ControlState state = ControlState::asleep;

void setup()
{
  host.start();
  logger.start();
  forebrain.start();

  
  pinMode(BRAKE_ENABLE_1, LOW);
  pinMode(BRAKE_ENABLE_1, OUTPUT);

  pinMode(BRAKE_ENABLE_2, LOW);
  pinMode(BRAKE_ENABLE_2, OUTPUT);

  pinMode(SABER_EMERGENCY_STOP, INPUT);

  logger.sendInfo("Setup");

#ifdef USE_EMULATOR
  setupEmulator();
#else
  setupSabertooth();
#endif

  sendEncoders(0, 0);

  commTimeout.begin(COMM_TIMEOUT);

  brakeState = BrakeState::engaged;
  state = ControlState::asleep;
  logger.sendInfo("Zzzz...");
  
  forebrain.sendInfo("Hindbrain");
  host.sendInfo("Hindbrain");

}

bool forceEstop = false;

int32_t lastEncLeft  = 0;
int32_t lastEncRight = 0;

void updateEncoders(int32_t encLeft, int32_t encRight)
{
  if (encLeft != lastEncLeft || encRight != lastEncRight) {
    lastEncLeft  = encLeft;
    lastEncRight = encRight;
    sendEncoders(encLeft, encRight);
  }
}




bool processHostCmd()
{
  if (!host.readCmd()) {
    return false;
  }

  int32_t v1;
  int32_t v2;

  switch (host.cmd()) {
    case 'U': // emulator mode on
      emulatorMode = true;
      setMotors(0, 0);
      host.sendInfo("emulate");
      return true;
    case 'I': // mirror mode on
      mirrorMode = true;
      host.sendInfo("mirror");
      return true;
    case 'S': // estop
      forceEstop = true;
      host.sendInfo("estop");
      return true;
    case 'C': // encoders
      host.getParam(v1);
      host.getParam(v2);
      updateEncoders(v1, v2);
      return true;
  }
  host.sendInfo("unknown");
  return false;
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
        logger.sendCmdFmt('I',"Max=%d",value);
      }
      break;
    case 'm': // min power
      if (value >= 0 && value <= MAX_MOTOR_POWER) {
        MIN_MOTOR_POWER = value;
        logger.sendCmdFmt('I',"Min=%d",value);
      }
      break;
  }
}

/*
 s   stop
 S   estop
 W   wake
 M   motor (2 ints) 
 c   config (2 ints)
 */


bool processForebrainCmd()
{
  if (!forebrain.readCmd()) {
    return false;
  }

  if (state == ControlState::estop) {
    return false;
  }

  char subCmd;
  int32_t  v1;
  int32_t  v2;

  switch (forebrain.cmd()) {
    case 's': // stop
      logger.sendInfo("Stop");
      stopBothAndBrake();
      logger.sendInfo("Hold");
      state = ControlState::timeout;
      logger.sendInfo("stop");
      return true;
    case 'S': // estop
      forceEstop = true;
      return true;
  }


  if (state == ControlState::asleep) {
    
    switch (forebrain.cmd()) {
      case 'W': // wake
        logger.sendInfo("Waking");
        state = ControlState::awake;
        brakeOff();
        commTimeout.start();
        return true;
      case 'c': // config
        forebrain.getParam(subCmd);
        forebrain.getParam(v1);
        handleConfigCommand(subCmd, v1);
        return true;
    }

  }

  if (state == ControlState::awake) {

    char m1;
    char m2;

    switch (forebrain.cmd()) {
      case 'M': // motor
        forebrain.getParam(m1);
        forebrain.getParam(m2);
        if (setBothPower(m1, m2)) {
          sprintf(tmp, "%4d%4d", (int)(char)lastLeftPower, (int)(char)lastRightPower);
          if (tmp[0] == ' ') {
            tmp[0] = 'M';
          }
          logger.sendInfo(tmp);
        }
        commTimeout.start();
        return true;
      case 'W': // wake
        stopBoth();
        logger.sendInfo("ReWake!");
        commTimeout.start();
        return true;
    }    
  }
}

void loop()
{
  static int estopCount = 0;
  
  updateBrakes();

  if (!emulatorMode) {
    int32_t encLeft  = encoderL.read();
    int32_t encRight = encoderR.read();
    updateEncoders(encLeft, encRight);
  }

  processHostCmd();
  processForebrainCmd();

  if (state != ControlState::estop && (forceEstop || digitalRead(SABER_EMERGENCY_STOP) == LOW))
  {
    logger.sendInfo("E Stop");
    stopBothAndBrake();
    state = ControlState::estop;
    estopCount = 3;
    commTimeout.start();
    forceEstop = false;
    return;
  }

  switch (state)
  {
    case ControlState::estop:
      if (commTimeout.fire())
      {
        if (forceEstop || digitalRead(SABER_EMERGENCY_STOP) == LOW) {
          logger.sendInfo("E Stop");
          estopCount = 3;
          commTimeout.start();          
        }
        else if (estopCount > 0) {
          logger.sendInfo("E Stop");
          estopCount--;
          commTimeout.start();
        }
        else {
          state = ControlState::asleep;
          logger.sendInfo("Zzzz...");
          commTimeout.start();
        }
      }
      break;
    case ControlState::timeout:
    case ControlState::badcmd:
      if (commTimeout.fire())
      {
        state = ControlState::asleep;
        logger.sendInfo("Zzzz...");
        commTimeout.start();
      }
      break;
    case ControlState::asleep:
      break;
    case ControlState::awake:
      if (commTimeout.fire())
      {
        logger.sendInfo("TimeOut");
        stopBothAndBrake();
        logger.sendInfo("Hold");
        state = ControlState::timeout;
      }
      break;
  }
}
