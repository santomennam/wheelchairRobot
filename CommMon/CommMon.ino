#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>
#include "CmdLink.h"

enum class BotState {
  awake,
  sleep,
  noConnect,
  estop
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
//
//void forwardEncodersToHost()  // NOTE: this assumes we just got a 'C' command from hindbrain!!!
//{
//  int32_t v1;
//  int32_t v2;
//  hindbrain.getParam(v1);
//  hindbrain.getParam(v2);
//  leftEncoder.refresh(v1);
//  rightEncoder.refresh(v2);
//  host.sendCmdII('C', leftEncoder.getCount(), rightEncoder.getCount());
//}

BotState handleEStopState()
{
  if (hindbrain.readCmd()) {
    switch (hindbrain.cmd()) {
      case 'S': // stopped/asleep
        return BotState::sleep;
      case 'E': // estop
        return BotState::estop;
      case 'I':
        //forwardInfoToHost();
        break;
    }
  }

  return BotState::estop;
}

BotState handleDisconnectState()
{
  if (hindbrain.readCmd()) {
    switch (hindbrain.cmd()) {
      case 'S': // stopped/asleep
        return BotState::sleep;
      case 'E': // estop
        return BotState::estop;
      case 'I':
        //forwardInfoToHost();
        break;
    }
  }

  return BotState::noConnect;
}

BotState handleSleepState()
{
  if (hindbrain.readCmd()) {
    switch (hindbrain.cmd()) {
      case 'C':
        //        forwardEncodersToHost();
        break;
      case 'S': // stopped/asleep
        break;
      case 'w': // waking
      case 's': // stopping
        break;
      case 'W': // awakened
        return BotState::awake;
      case 'E': // estop
        return BotState::estop;
      case 'I':
        break;
      default:
        break;
    }
  }

  if (hindbrain.recvTimeout()) {
    // connection to hindbrain lost
    return BotState::noConnect;
  }

  return BotState::sleep;
}


BotState handleAwakeState()
{
  if (hindbrain.readCmd()) {
    int32_t v1;
    int32_t v2;
    switch (hindbrain.cmd()) {
      case 'C': // encoder count
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
        break;
      default:
        break;
    }
  }

  if (hindbrain.recvTimeout()) {
    // connection to hindbrain lost
    return BotState::noConnect;
  }

  return BotState::awake;
}

BotState lastbotState{BotState::awake};

void reportState(BotState state)
{
  switch (botState) {
    case BotState::noConnect:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Not Connected");
      //    lcd.setCursor(0, 1);
      //    lcd.print("*** second line.");
      break;
    case BotState::sleep:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Asleep");
      //    lcd.setCursor(0, 1);
      //    lcd.print("*** second line.");
      break;

    case BotState::awake:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Awake");
      break;
    case BotState::estop:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("EStop");
      break;
  }

}

void loop()
{
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
    case BotState::awake:
      botState = handleAwakeState();
      break;
  }

  forebrain.readCmd();

  if (forebrain.isCorrupt()) {
    //    host.sendCmdStr('I', "Corrupt");
    //    host.sendCmdStr('I', host.getCorruptMsg());
    //    host.sendCmdStr('I', "FromHost");
  }

  if (hindbrain.isCorrupt()) {
    //    host.sendCmdStr('I', "Corrupt");
    //    host.sendCmdStr('I', hindbrain.getCorruptMsg());
    //    host.sendCmdStr('I', "FromHind");
  }

  if (lastbotState != botState) {
    // bot state changed, update displays

    lastbotState =  botState;

    reportState(botState);

    switch (botState) {
      case BotState::estop:
        break;
      case BotState::noConnect:
        break;
      case BotState::sleep:
        break;
      case BotState::awake:
        break;
    }
  }
}
