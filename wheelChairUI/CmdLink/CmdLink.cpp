#include "CmdLink.h"
#include <HardwareSerial.h>

#define MAX_CMD_SIZE 16

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
    int lfIdx = pushIdx-1;
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
  }
  return false;
}

void CmdBuffer::copyDataTo(char *dst, int count)
{
  while (count-- > 0) {
    *dst++ = get(dataIdx++); 
  }
}

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

CmdLink::CmdLink(HardwareSerial& strm, uint32_t baud)
  : stream{strm}
{
  baudRate = baud;
}

void CmdLink::start()
{
    stream.begin(baudRate);
}

void CmdLink::sendCmd(char cmd)
{
  builder.begin(cmd);
  stream.write(builder.finish(), builder.length());  
}

void CmdLink::sendCmdStr(char cmd, char* str)
{  
  builder.begin(cmd);
  builder.pushData(str, strlen(str));
  stream.write(builder.finish(), builder.length());  
}

void CmdLink::sendCmdBB(char cmd, char v1, char v2)
{
  builder.begin(cmd);
  builder.pushData(v1);
  builder.pushData(v1);
  stream.write(builder.finish(), builder.length());  
}

void CmdLink::sendCmdBI(char cmd, char v1, int16_t v2)
{
  builder.begin(cmd);
  builder.pushData(v1);
  builder.pushData(v1);
  stream.write(builder.finish(), builder.length());  
}

void CmdLink::sendCmdII(char cmd, int16_t v1, int16_t v2)
{
  builder.begin(cmd);
  builder.pushData(v1);
  builder.pushData(v1);
  stream.write(builder.finish(), builder.length());  
}

void CmdLink::sendCmdI(char cmd, int16_t v1)
{
  builder.begin(cmd);
  builder.pushData(v1);
  stream.write(builder.finish(), builder.length());      
}

bool CmdLink::readCmd()
{
   while (stream.available() > 0) {
      if (buffer.push(stream.read())) {
        return true;
      }
   }
   return false;
}