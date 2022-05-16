#ifndef CmdLink_h
#define CmdLink_h

#include <stdint.h>

#ifndef ARDUINO
#include <functional>
#include <string>
#else
#include <FireTimer.h>
#endif

// commands start with '#' followed by 4 bytes (command specific) followed by '\n'  6 bytes total

#define MAX_CMD_SIZE 16

// cmd format    #0a\n 
// or            #1ab\n
// or            #4abbbb\n
// through       #9abbbbbbbbb\n    
// etc
// where a is upper or lower case cmd letter
// 0-9 indicates number of bytes after cmd letter (1 digit only)
// b is any byte

class HardwareSerial;

class CmdBuffer {
  char buffer[MAX_CMD_SIZE];
  int  pushIdx{0};
  int  hashIdx{-1}; // -1 means we haven't seen a # yet
  char lastCmd{0};
  int  numDataBytes{0};
  int  dataIdx{0};
 public:
  bool push(char c);
  char cmd() const { return lastCmd; }
  void eraseCmd();
  int  length() const { return numDataBytes; }
  void copyDataTo(char *dst, int count);
  template<typename T>
  void copyDataTo(T& dst);
#ifndef ARDUINO
  std::string currentCmdBuffer() const;
#endif
 private:
  bool verify(int hashIdx, int lfIdx);
  char get(int idx) const { return buffer[idx%MAX_CMD_SIZE]; }

};

template<typename T>
void CmdBuffer::copyDataTo(T& dst)
{
  copyDataTo(reinterpret_cast<char*>(&dst), sizeof(T));
}


class CmdBuilder {
  char buffer[MAX_CMD_SIZE];
  int  numDataBytes{0};
public:
  CmdBuilder();
  void begin(char cmd);
  int length() { return numDataBytes + 4; }
  void pushData(const char* data, int count);
  template<typename T>
  void pushData(const T& data);
  char *finish();
};


template<typename T>
void CmdBuilder::pushData(const T& data)
{
   pushData(reinterpret_cast<const char*>(&data), sizeof(T));
}

class CmdLink {
#ifdef ARDUINO
  HardwareSerial&    stream;
  uint32_t           baudRate;
  FireTimer          sendTimer;
  FireTimer          recvTimer;
#else
    std::function<void(const char* data, int len)> writer;
    std::function<bool()> canRead;
    std::function<char()> readChar;
#endif
    bool debug{false};
  CmdBuffer  buffer;
  CmdBuilder builder;
 public:
  int                sendTimeoutMS{300};
  int                recvTimeoutMS{400};

#ifdef ARDUINO
  CmdLink(HardwareSerial& strm, uint32_t baud);
#else
  CmdLink(std::function<void(const char* data, int len)> writer,
          std::function<bool()> canRead,
          std::function<char()> readChar
          );
#endif

  void start();

  void sendCmd(char cmd);
  void sendCmdStr(char cmd, const char* str);
  template<typename T>
  void sendCmdFmt(char cmd, const char*fmt, T val);
  void sendCmdBB(char cmd, char v1, char v2);
  void sendCmdII(char cmd, int32_t v1, int32_t v2);
  void sendCmdI(char cmd, int32_t v1);
  void sendCmdBI(char cmd, char v1, int32_t v2);
  void sendInfo(const char* str)  { sendCmdStr('I', str); }
  void sendError(const char* str) { sendCmdStr('E', str); }

  bool readCmd();
  char cmd() { return buffer.cmd(); }
  void eraseCmd() { buffer.eraseCmd(); }

  template<typename T>
  void getParam(T& dst);

#ifdef ARDUINO
  //String getStr();
  bool recvTimeout() { return recvTimer.fire(); }
  bool sendTimeout() { return sendTimer.fire(); }
#else
  std::string getStr();
  void setDebug(bool dbg) { debug = dbg; }
  bool isDebug() const { return debug; }
#endif

private:
  void send();
};

template<typename T>
void CmdLink::sendCmdFmt(char cmd, const char* fmt, T val)
{
  char buff[9];
  snprintf(buff, sizeof(buff), fmt, val);
  sendCmdStr(cmd, buff); 
}

template<typename T>
void CmdLink::getParam(T& dst)
{
  buffer.copyDataTo(dst);
}


#endif