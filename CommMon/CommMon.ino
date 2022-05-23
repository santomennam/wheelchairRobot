#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>
#include "CmdLink.h"

enum class BotState {
  awake,
  sleep,
  noConnect,
  estop
};

enum class ForebrainState {
  awake,
  noConnect,
};

LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27 for a 16 chars and 2 line display

int show = -1;


CmdLink forebrain(Serial3, 19200);
CmdLink hindbrain(Serial2, 19200);


void setup()
{
  int error;

  forebrain.start();
  hindbrain.start();

  Serial.begin(115200);
  Serial.println("Monitor");

  // wait on Serial to be available on Leonardo
  while (!Serial)
    ;

  Wire.begin();
  Wire.beginTransmission(0x27);
  error = Wire.endTransmission();

  if (error == 0) {
    Serial.println(": LCD found.");
    show = 0;
    lcd.begin(16, 2); // initialize the lcd

  } else {
    Serial.println(": LCD not found.");
  } // if

  lcd.setBacklight(255);
} // setup()

//
//void loop()
//{
//  if (show == 0) {
//    lcd.setBacklight(255);
//    lcd.home();
//    lcd.clear();
//    lcd.print("Hello LCD");
//    delay(1000);
//
//    lcd.setBacklight(0);
//    delay(400);
//    lcd.setBacklight(255);
//
//  } else if (show == 1) {
//    lcd.clear();
//    lcd.print("Cursor On");
//    lcd.cursor();
//
//  } else if (show == 2) {
//    lcd.clear();
//    lcd.print("Cursor Blink");
//    lcd.blink();
//
//  } else if (show == 3) {
//    lcd.clear();
//    lcd.print("Cursor OFF");
//    lcd.noBlink();
//    lcd.noCursor();
//
//  } else if (show == 4) {
//    lcd.clear();
//    lcd.print("Display Off");
//    lcd.noDisplay();
//
//  } else if (show == 5) {
//    lcd.clear();
//    lcd.print("Display On");
//    lcd.display();
//
//  } else if (show == 7) {
//    lcd.clear();
//    lcd.setCursor(0, 0);
//    lcd.print("*** first line.");
//    lcd.setCursor(0, 1);
//    lcd.print("*** second line.");
//
//  } else if (show == 8) {
//    lcd.scrollDisplayLeft();
//  } else if (show == 9) {
//    lcd.scrollDisplayLeft();
//  } else if (show == 10) {
//    lcd.scrollDisplayLeft();
//  } else if (show == 11) {
//    lcd.scrollDisplayRight();
//
//  } else if (show == 12) {
//    lcd.clear();
//    lcd.print("write-");
//
//  } else if (show > 12) {
//    lcd.print(show - 13);
//  } // if
//
//  delay(1400);
//  show = (show + 1) % 16;
//} // loop()



int commTimeoutTime = 1000;

BotState botState{BotState::noConnect};
BotState lastbotState{BotState::awake};

ForebrainState forebrainState{ForebrainState::noConnect};
ForebrainState lastForebrainState{ForebrainState::awake};

int32_t encL{ -1};
int32_t encR{ -1};
bool encUpdated{true};

int8_t motorL{0};
int8_t motorR{0};
bool motorUpdated{true};

constexpr int bufflen = 17;

char lineOne[bufflen] = "LineOne";
char lineTwo[bufflen] = "LineTwo";
bool needRedraw{true};

void displayLine(int lineNum, const char* message)
{
  snprintf(lineNum ? lineTwo : lineOne, bufflen, message);
  needRedraw = true;
}

template<typename T>
void displayLine(int lineNum, const char* message, T param)
{
  snprintf(lineNum ? lineTwo : lineOne, bufflen, message, param);
  needRedraw = true;
}

template<typename T1, typename T2>
void displayLine(int lineNum, const char* message, T1 p1, T2 p2)
{
  snprintf(lineNum ? lineTwo : lineOne, bufflen, message, p1, p2);
  needRedraw = true;
}

void redrawIfNeeded()
{
  if (needRedraw) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(lineOne);
    lcd.setCursor(0, 1);
    lcd.print(lineTwo);
    needRedraw = false;
  }
}

void handleEncoders()  // NOTE: this assumes we just got a 'C' command from hindbrain!!!
{
  hindbrain.getParam(encL);
  hindbrain.getParam(encR);
  encUpdated = true;
}


void handleMotors()  // NOTE: this assumes we just got a 'M' command from forebrain!!!
{
  forebrain.getParam(motorL);
  forebrain.getParam(motorR);
  motorUpdated = true;
}

void reportState(BotState state)
{
  switch (state) {
    case BotState::noConnect:
      displayLine(0, "Hindbrain N/C");
      break;
    case BotState::sleep:
      displayLine(0, "ZZZzz...");
      break;
    case BotState::awake:
      displayLine(0, "Awake");
      break;
    case BotState::estop:
      displayLine(0, "E-Stop");
      break;
  }
}

void reportForebrainState(ForebrainState state)
{
  switch (state) {
    case ForebrainState::noConnect:
      displayLine(1, "Forebrain N/C");
      break;
    case ForebrainState::awake:
      displayLine(1, "Awake");
      break;
  }
}

bool gotCorruptHindbrain{false};
bool gotCorruptForebrain{false};

void loop()
{
  redrawIfNeeded();
  
  if (hindbrain.readCmd()) {
    switch (hindbrain.cmd()) {
      case 'C':
        handleEncoders();
        break;
      case 'S': // stopped/asleep
      case 's': // stopping
        botState = BotState::sleep;
        break;
      case 'w': // waking
      case 'W': // awake ping
        botState = BotState::awake;
        break;
      case 'E': // estop
        botState = BotState::estop;
        break;
      case 'I':
        break;
      default:
        break;
    }
  }

  if (hindbrain.recvTimeout()) {
    // connection to hindbrain lost
    botState = BotState::noConnect;
  }

  if (forebrain.readCmd()) {
    switch (forebrain.cmd()) {
      case 'M':
        handleMotors();
        break;
      default:
        forebrainState = ForebrainState::awake;
        break;
    }
  }

  if (forebrain.recvTimeout()) {
    forebrainState = ForebrainState::noConnect;
  }

  if (encUpdated) {
    encUpdated = false;
    displayLine(0, "%8ld%8ld", encL, encR);
    needRedraw = true;
  }


  if (motorUpdated) {
    motorUpdated = false;
    displayLine(1, "Motor: %4d %4d", (int)motorL, (int)motorR);
    needRedraw = true;
  }

  if (forebrain.isCorrupt()) {
    gotCorruptForebrain = true;
    displayLine(1, forebrain.getCorruptMsg());
  }

  if (hindbrain.isCorrupt()) {
    gotCorruptHindbrain = true;
    displayLine(0, hindbrain.getCorruptMsg());
  }
//
//  if (gotCorruptHindbrain || gotCorruptForebrain) {
//    return;
//  }

  if (lastbotState != botState) {
    // bot state changed, update displays
    lastbotState =  botState;
    reportState(botState);
  }

  if (lastForebrainState != forebrainState) {
    // bot state changed, update displays
    lastForebrainState =  forebrainState;
    reportForebrainState(forebrainState);
  }
}
