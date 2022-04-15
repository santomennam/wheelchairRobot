#include <SoftwareSerial.h>
#include <string.h>
#include <ArduPID.h>
#include <Encoder.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

Adafruit_8x16matrix matrix = Adafruit_8x16matrix();

ArduPID PIDControllerL;
ArduPID PIDControllerR;

int encThreshold = 300; // amount of acceptable error (encoder units: 2400/rotation) //was 1200


Encoder black(2, 3); // these colors refer to the colors of the 3d printed wheels on the robot as of Feb 2022
Encoder red(18, 19); // black right red left

//   avoid using pins with LEDs attached for encoders

SoftwareSerial Serial1a(10, 9);
SoftwareSerial foreBrainComm(12, 13); // (COMRX, COMTX); //talk from board to robot

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


void wakeWheelchair()
{
  foreBrainComm.write('w');
  foreBrainComm.write('a');
  foreBrainComm.write('k');
  foreBrainComm.write('e');
}

void enableDebug()
{
  foreBrainComm.write('d');
  foreBrainComm.write('b');
  foreBrainComm.write('u');
  foreBrainComm.write('g');
}

void setMotorSpeeds(int left, int right)
{
  drawArrows(left, right);
  
  char m1 = (char)left;
  char m2 = (char)right;
  foreBrainComm.write('m');
  foreBrainComm.write(m1);
  foreBrainComm.write(m2);
  foreBrainComm.write('x');
}

class SmartEncoder {
  private:
    Encoder& enc;
    ArduPID& pid;
  private:
    long count{0};
    long prevCount{0};
    long target{0};
    bool hasTarget{false};
    bool usingPid{false};
    double pidInput;
    double pidOutput;
    double pidTarget;
    double kP = 0.05;  // 0.05 is pretty close but overshoots a little bit
    double kI = 0.002; // 0.0005;
    double kD = 0.00;
  public:
    SmartEncoder(Encoder& enc, ArduPID& pid) : enc{enc}, pid{pid} {}
    bool refresh();
    void reset();
    long getTarget() {
      return target;
    }
    long getCount() {
      return count;
    }
    void setTarget(long targ);
    void setRelativeTarget(long offset) {
      setTarget(target + offset);
    }
    void clearTarget();
    bool getHasTarget() { return hasTarget; }
    long dist() {
      return target - count;
    }
    long absDist() {
      return abs(target - count);
    }
    bool reachedTarget(long threshold) {
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

bool SmartEncoder::refresh()
{
  count = -enc.read();
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
  enc.write(0);
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
    pid.setOutputLimits(-20, 20);
  }
}

void SmartEncoder::disablePid()
{
  if (usingPid) {
    usingPid = false;
    pid.stop();
  }
}

void SmartEncoder::setTarget(long targ)
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

  const long velModeThreshold = 2000;
  const int motorSpeedBase = 20;

  if (absDist() > velModeThreshold) {
    // we're far enough away that we won't use PID control
    disablePid();
    return fixMotorSpeedSign(motorSpeedBase);
  }

  enablePid();
  pid.compute();
  
  return pidOutput;
}

SmartEncoder leftEncoder(red, PIDControllerL);
SmartEncoder rightEncoder(black, PIDControllerR);


bool refreshEncoders()
{
  bool leftUpdated  = leftEncoder.refresh();
  bool rightUpdated = rightEncoder.refresh();
  return leftUpdated || rightUpdated;  // if you refactor this, be careful about short circuit evaluation... we want both refresh calls to happen!
}
// single digit 0-15 -> 0-F  (capital)
char digitToHexChar(int i)
{
    return i > 9 ? ('A' + i - 10) : '0' + i;
}

// single char 0-F (capital!!) to integer 0-15
int hexCharToDigit(char c)
{
    return c >= 'A' ? (c - 'A' + 10) : c - '0';
}

uint8_t computeCRC8(const uint8_t  *bytes, int len) {
  const uint8_t  generator = 0b00101111;   // polynomial = x^8 + x^5 + x^3 + x^2 + x + 1 (ignore MSB which is always 1)
  uint8_t  crc = 0;

  while (len--)
  {
    crc ^= *bytes++; /* XOR-in the next input byte */

    for (int i = 0; i < 8; i++)
    {
      if ((crc & 0x80) != 0)
      {
        crc = (uint8_t )((crc << 1) ^ generator);
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void computeCRC8(const uint8_t  *bytes, int len, char& c1, char& c2)
{
    int crc = computeCRC8(bytes, len);
    c2 = digitToHexChar(crc & 0x0F);
    c1 = digitToHexChar(crc >> 4);
}

bool validateCRC8(const uint8_t *bytes, int len, char c1, char c2)
{
    int crc = computeCRC8(bytes, len);
    return (c2 == digitToHexChar(crc & 0x0F)) && (c1 == digitToHexChar(crc >> 4));
}

bool stripCRC8(String& str)
{
    if (str.length() < 3) {
        return false;
    }
    bool valid = validateCRC8(reinterpret_cast<const uint8_t*>(str.c_str())+3, str.length()-3, str[0], str[1]);
    if (valid) {
        str = String(str.c_str()+3);
    }
    return valid;
}

void writeDelimited(String str)
{
    char crc[] = "#xx:";
    computeCRC8(reinterpret_cast<const uint8_t*>(str.c_str()), str.length(), crc[1], crc[2]);
    Serial.print(crc);
    Serial.println(str);
}

String readDelimited(bool flushData)
{
  static bool gotStart = false;
  static String data;
  String result;
  static int startBytes = 3;
  static char crc[] = "XX:";

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
      data = ""; 
      crc[0] = 'X';
      crc[1] = 'X';
      crc[2] = ':';
      startBytes = 3;
    }
    else if (dat == '\n') {
      if (gotStart) {
        // end of line

        if (validateCRC8(reinterpret_cast<const uint8_t*>(data.c_str()), data.length(), crc[0], crc[1])) {
          result = data;        
        }
        else {
          // HOW DO WE HANDLE THIS ERROR!!
          writeDelimited(String("E CRC Failed Checksum: ") + crc + "-'" + data + "'");
        }
        data = "";
        gotStart = false;
        return result;
      }
    }
    else if (gotStart) {
      switch (startBytes) {
      case 3: // CRC Nibble 1
        crc[0] = dat;
        startBytes--;
        break;
      case 2: // CRC Nibble 2
        crc[1] = dat;
        startBytes--;
        break;
      case 1: // :
        crc[2] = dat;
        startBytes--; 
        break;
      default:
        data.concat(static_cast<char>(dat));
        break;
      }
      
      if (data.length() > 100) {
        data = "";
        gotStart = false;
      }
    }
    else {
      // ignoring junk, since we haven't seen a #
    }
  }

  return result;
}

void wheelChairReset()
{
  setMotorSpeeds(0, 0);
  wakeWheelchair();
  leftEncoder.reset();
  rightEncoder.reset();
}

void wheelChairStop()
{
  setMotorSpeeds(0,0);
  leftEncoder.clearTarget();
  rightEncoder.clearTarget();
}

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

void checkEstop()
{
  if (digitalRead(estop2) == HIGH)
  {
    setMotorSpeeds(0, 0);
    readDelimited(true);

    bool first = true;

    displayX(20,0,0);
    updateDisplay();
    
    while (true)
    {
      

  
      //in an e-stop holding pattern
      delay(100);

      bool encChanged = refreshEncoders();

      if (first || encChanged) {
        writeDelimited(String("X ESTOP 0 ") + leftEncoder.getCount() + " " + rightEncoder.getCount()); // estop
        first = false;
      }

      String command = readDelimited(false);

      if (command.startsWith("reset"))
      {
        writeDelimited("R Reset Received");// T indicates target data
        wheelChairStop();
        return;
      }
      else if (command.length() > 0) {
        writeDelimited("I In E-Stop so ignoring: " + command);
      }
    }
  }
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

bool processCommands()
{
  String command = readDelimited(false);

  if (command.length() == 0) {
    return false;
  }

  if (command.startsWith("debug"))
  {
    enableDebug();
    writeDelimited("D Debug ON");
    return true;
  }
  else if (command.startsWith("reset"))
  {
    writeDelimited("R Reset Received");// T indicates target data
    wheelChairReset();
    return true;
  }
  else if (command.startsWith("ask")) {
    writeDelimited("A " + String(leftEncoder.getTarget()) + " " + String(rightEncoder.getTarget())); 
    return true;
  }
  else if (command.startsWith("target"))
  {
    command = command.substring(command.indexOf(" ") + 1, -1);
    long leftTarget  = getValue(command, ' ', 0).toInt();
    long rightTarget = getValue(command, ' ', 1).toInt();
    leftEncoder.setTarget(leftTarget);
    rightEncoder.setTarget(rightTarget);
    writeDelimited("T " + String(leftEncoder.getTarget()) + " " + String(rightEncoder.getTarget())); 
    return true;
  }
  else if (command.startsWith("ping")) // keep awake
  {
    return true;
  }
  else {
    writeDelimited("E Invalid command (doing a reset)" + command);
    wheelChairStop();
    return true;
  }
}

void setup()
{
  pinMode(estop1, OUTPUT);
  pinMode(leftModeLED, OUTPUT);
  pinMode(rightModeLED, OUTPUT);
  digitalWrite(estop1, LOW);
  pinMode(estop2, INPUT_PULLUP);

  Serial.begin(19200);
  foreBrainComm.begin(9600);
  Serial1a.begin(115200);

  matrix.begin(0x70); 
  matrix.setRotation(1);
  matrix.setBrightness(5);
  
  matrix.clear();
  matrix.drawBitmap(4, 0, frown_bmp, 8, 8, LED_ON);
  matrix.writeDisplay();  // write the changes we just made to the display
  
  delay(500);
  
  wheelChairReset();

  matrix.clear();
  matrix.drawBitmap(4, 0, smile_bmp, 8, 8, LED_ON);
  matrix.writeDisplay();  // write the changes we just made to the display
}

long lastCmdTime = 0;
long commTimeoutMs = 500;
bool commEstablished = false;

void loop()
{
  checkEstop();


  bool gotCommand = processCommands();

  long currentTime = millis();

  if (gotCommand) {
     lastCmdTime = currentTime;
     if (!commEstablished) {
         // established connection to host!
         commEstablished = true;
         wakeWheelchair();
     }
  }
  else {
     long elapsed = currentTime - lastCmdTime;
     if (commEstablished && elapsed > commTimeoutMs) {
         // communication from host timed out!
         commEstablished = false;
         wheelChairStop();
         writeDelimited("B Byeeee (timeout)"); 
     }
  }

  if (!commEstablished) {
    updateDisplay();
    return;
  }

  bool encodersChanged = refreshEncoders();

  int motorL = leftEncoder.computeMotorSpeed();
  int motorR = rightEncoder.computeMotorSpeed();

  digitalWrite(leftModeLED,  leftEncoder.isUsingPid()  ? HIGH : LOW);
  digitalWrite(rightModeLED, rightEncoder.isUsingPid() ? HIGH : LOW);

  setMotorSpeeds(motorL, motorR);

  updateDisplay();
    
  delay(100);
  
  if (encodersChanged || motorL != 0 || motorR != 0) {
    writeDelimited(String("P 0 ") + leftEncoder.getCount() + " " + rightEncoder.getCount() + " " + motorL + " " + motorR);  // P indicates position data
  }
  else {
    writeDelimited("H"); // heartbeat
  }

}
