#include "CmdLink.h"

#ifdef ARDUINO
#include <HardwareSerial.h>
#else
#include <cstring>
#include <iostream>
#include <iomanip>
#endif

#define MAX_CMD_SIZE 16

using namespace std;

#ifndef ARDUINO
void dumpAsHex(std::string data)
{
    for (char c : data) {
        if (isprint(c)) {
            cout << setfill(' ') << setw(2) << c << " ";
        }
        else if (c == '\n') {
            cout << "\\n ";
        }
        else {
            cout << "__ ";
        }
    }
    cout << endl;
    for (char c : data) {
        cout << std::hex << setw(2) << setfill('0') << (0xFF & (int)c) << " ";
    }
    cout << std::dec << setfill(' ') << endl;
}
#endif

bool CmdBuffer::verify(int hIdx, int lfIdx)
{
  char digit = get(hIdx+1);
  if (digit >= '0' && digit <= '9') {
    int ndb = digit - '0';
    int expectLF = (hIdx + ndb + 3) % MAX_CMD_SIZE;
    if (expectLF == lfIdx) {
       // found a LF the correct distance from the #, looks like a valid command
       lastCmd = get(hIdx+2);
       dataIdx = (hIdx+3)%MAX_CMD_SIZE;
       numDataBytes = ndb;
       hashIdx = -1;
       return true;
    }
  }
  return false;
}

bool CmdBuffer::push(char c)
{
  if (c == '#') {
    if (hashIdx < 0) {
      hashIdx = pushIdx;
    }
  }
  buffer[pushIdx] = c;
  pushIdx = (pushIdx+1)%MAX_CMD_SIZE;
  if (c == '\n' && hashIdx >= 0) {
    int lfIdx = (pushIdx+MAX_CMD_SIZE-1)%MAX_CMD_SIZE;
    if (verify(hashIdx, lfIdx)) {
         return true;
    }

    // # didn't match see if there is a matching # after the first one we found? ('cause we are
    // out of sync perhaps, due to garbage input)
    int searchIdx = (pushIdx + MAX_CMD_SIZE - 3) % MAX_CMD_SIZE; // start searching for a # three before where we are
    while (searchIdx != hashIdx) {
      //
      if (buffer[searchIdx] == '#' && verify(searchIdx, lfIdx)) {
          return true;
      }
      searchIdx = (searchIdx + MAX_CMD_SIZE - 1) % MAX_CMD_SIZE;  // back-up with wrap
    }
#ifndef ARDUINO
    cout << "Failed Search:\n PushIdx = " << pushIdx << endl;
    searchIdx = (pushIdx + MAX_CMD_SIZE - 3) % MAX_CMD_SIZE; // start searching for a # three before where we are
    cout << " SearchIdx = " << searchIdx << endl;
    dumpAsHex(string(buffer, MAX_CMD_SIZE));
#endif
  }
  return false;
}

void CmdBuffer::copyDataTo(char *dst, int count)
{
  while (count-- > 0) {
    *dst++ = get(dataIdx++);
  }
}

#ifndef ARDUINO
string CmdBuffer::currentCmdBuffer() const
{
    int idx = (dataIdx + MAX_CMD_SIZE - 3) % MAX_CMD_SIZE;
    char tmp[MAX_CMD_SIZE];
    for (int i = 0; i < numDataBytes+4; i++) {
        tmp[i] = get(idx++);
    }
    return string(tmp, numDataBytes+4);
}
#endif

CmdBuilder::CmdBuilder()
{
  buffer[0] = '#';
  numDataBytes = 0;
}

void CmdBuilder::begin(char cmd)
{
  buffer[1] = '0';
  buffer[2] = cmd;
  numDataBytes = 0;
}

void CmdBuilder::pushData(const char* data, int count)
{
   if (numDataBytes + count > 9) {
     // throw or otherwise indicate an error here!
     return;
   }
   char *dst = buffer + 3 + numDataBytes;
   numDataBytes += count;
   while (count-- > 0) {
     *dst++ = *data++;
   }
   buffer[1] = '0' + numDataBytes;
}

char *CmdBuilder::finish()
{
   // #0c\n
   buffer[3+numDataBytes] = '\n';
   return buffer;
}

#ifdef ARDUINO
CmdLink::CmdLink(HardwareSerial& strm, uint32_t baud)
  : stream{strm}
{
  baudRate = baud;
}

void CmdLink::start()
{
    stream.begin(baudRate);
}
#else
CmdLink::CmdLink(std::function<void (const char *, int)> writer, std::function<bool ()> canRead, std::function<char ()> readChar)
    : writer{writer}, canRead{canRead}, readChar{readChar}
{

}
#endif



void CmdLink::sendCmd(char cmd)
{
  builder.begin(cmd);
  send();
}

void CmdLink::sendCmdStr(char cmd, char* str)
{
  builder.begin(cmd);
  builder.pushData(str, strlen(str));
  send();
}

void CmdLink::sendCmdBB(char cmd, char v1, char v2)
{
  builder.begin(cmd);
  builder.pushData(v1);
  builder.pushData(v2);
  send();
}

void CmdLink::sendCmdBI(char cmd, char v1, int32_t v2)
{
  builder.begin(cmd);
  builder.pushData(v1);
  builder.pushData(v2);
  send();
}

void CmdLink::sendCmdII(char cmd, int32_t v1, int32_t v2)
{
  builder.begin(cmd);
  builder.pushData(v1);
  builder.pushData(v2);
  send();
}

void CmdLink::sendCmdI(char cmd, int32_t v1)
{
  builder.begin(cmd);
  builder.pushData(v1);
  send();
}

bool CmdLink::readCmd()
{
#ifdef ARDUINO
   while (stream.available() > 0) {
      if (buffer.push(stream.read())) {
        return true;
      }
   }
#else
   while (canRead()) {
       if (buffer.push(readChar())) {
           if (debug) {
               string msg = buffer.currentCmdBuffer();
               if (msg != "#3KAck\n") {
                   cout << "Received: \n";
                   dumpAsHex(msg);
               }
           }
           return true;
       }
   }
#endif
   return false;
}

#ifndef ARDUINO
std::string CmdLink::getStr()
{
    char buff[10];
    buffer.copyDataTo(buff, buffer.length());
    return std::string(buff, buffer.length());
}
#endif

void CmdLink::send()
{
    char *sendbuffer = builder.finish();
    int sendlen = builder.length();
#ifdef ARDUINO
    stream.write(sendbuffer, sendlen);
#else
    if (debug) {
        string msg = string(sendbuffer, sendlen);
        if (msg != "#0P\n") {
            cout << "Sending:\n";
            dumpAsHex(msg);
        }
    }
    writer(sendbuffer, sendlen);
#endif
}
