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
void dumpAsHex(std::ostream& strm, std::string data)
{
    strm << "|";
    for (char c : data) {
        if (isprint(c)) {
            strm << setfill(' ') << setw(2) << c << " ";
        }
        else if (c == '\n') {
            strm << "\\n ";
        }
        else {
            strm << "__ ";
        }
    }
    strm << "| |";
    for (char c : data) {
        strm << std::hex << setw(2) << setfill('0') << (0xFF & (int)c) << " ";
    }
    strm << std::dec << setfill(' ') << "|" << endl;
}
#endif


bool CmdBuffer::push(char c)
{
    switch (state) {
    case CmdBufferState::corrupt:     // got unexpected input, waiting for #
    case CmdBufferState::expectHash:  // ready for #
        if (c == '#') {
            state = CmdBufferState::expectSize;
            return false;
        }
        return setCorrupt(c,"1Not #");
    case CmdBufferState::expectSize:   // got the #, waiting for digit
        if (c >= '0' && c <= '9') {
            numDataBytes = c - '0';
            state = CmdBufferState::expectCmd;
            return false;
        }
        return setCorrupt(c,"2NotDigt");
    case CmdBufferState::expectCmd:
        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')) {
            lastCmd = c;
            if (numDataBytes > 0) {
                state = CmdBufferState::expectData;
                numDataRecv = 0;
                return false;
            }
            else {
                state = CmdBufferState::expectEnd;
                return false;
            }
        }
        return setCorrupt(c,"3NotCmd");
    case CmdBufferState::expectData:   // got non-zero size, waiting for data
        if (c == '\\') {
            state = CmdBufferState::expectEsc;
            return false;
        }
        // if (numDataRecv == dataBufferSize) {
        //     throw logic_error("buffer overrun!!");
        // }
        dataBuffer[numDataRecv++] = c;
        if (numDataRecv == numDataBytes) {
            state = CmdBufferState::expectEnd;
            return false;
        }
        // more data to receive
        return false;
    case CmdBufferState::expectEsc:    // got a \, waiting for 0,1,2,3
        switch (c) {
        case '0':
            c = 0;
            break;
        case '1':
            c = '#';
            break;
        case '2':
            c = '\n';
            break;
        case '3':
            c = '\\';
            break;
        default:
            // unexpected escape
            return setCorrupt(c,"4NotEsc");
        }
        // now this is just like normal
        // if (numDataRecv == dataBufferSize) {
        //     throw logic_error("buffer overrun!!");
        // }
        dataBuffer[numDataRecv++] = c;
        if (numDataRecv == numDataBytes) {
            state = CmdBufferState::expectEnd;
            return false;
        }
        // more data to receive
        state = CmdBufferState::expectData;
        return false;
    case CmdBufferState::expectEnd:    // end of data reached, waiting for \n
        if (c == '\n') {
            state = CmdBufferState::expectHash;
            copyDataPos = 0;
            return true; // woohoo!  we got a complete command
        }
        // should have gotten \n :(
        return  setCorrupt(c,"5Not\\n");
    default:
#ifndef ARDUINO
        cout << "BadBufferState: " << static_cast<int>(state) << endl;
        throw logic_error("Bad CmdBufferState!!!");
#endif
        return setCorrupt(c,"6BState"); // buffer overrun??
    }
}

void CmdBuffer::copyDataTo(char *dst, int count)
{
    memcpy(dst, dataBuffer+copyDataPos, count);
    copyDataPos += count;
}

#ifndef ARDUINO
void CmdBuffer::dump(std::ostream& strm)
{
    string stateStr{"CmdStateCorruptUnk"};

    switch (state) {
    case CmdBufferState::corrupt:     // got unexpected input, waiting for #
        stateStr = "CmdStateCorrupt";
        break;
    case CmdBufferState::expectHash:  // ready for #
        stateStr = "Ready";
        break;
    case CmdBufferState::expectSize:   // got the #, waiting for digit
        stateStr = "CmdStateExpectSize";
        break;
    case CmdBufferState::expectCmd:
        stateStr = "CmdStateExpectCmd";
        break;
    case CmdBufferState::expectData:   // got non-zero size, waiting for data
        stateStr = "CmdStateExpectData";
        break;
    case CmdBufferState::expectEsc:    // got a \, waiting for 0,1,2,3
        stateStr = "CmdStateExpectEsc";
        break;
    case CmdBufferState::expectEnd:    // end of data reached, waiting for \n
        stateStr = "CmdStateExpectEnd";
        break;
    }

    strm << stateStr << ": " << lastCmd << " sz: " << numDataBytes << " rc: " << numDataRecv << ": ";
    dumpAsHex(strm, string(dataBuffer, numDataRecv));
}

#endif

bool CmdBuffer::setCorrupt(char c, const char *msg)
{
    state = CmdBufferState::corrupt;
    corruptChar = c;
    corruptMsg = msg;
    return false;
}

CmdBuilder::CmdBuilder()
{
    buffer[0] = '#';
    numDataBytes = 0;
}

void CmdBuilder::begin(char cmd)
{
    buffer[0] = '#';
    buffer[1] = '0';
    buffer[2] = cmd;
    numDataBytes = 0;
    numEsc = 0;
}

void CmdBuilder::pushData(const char* data, int count)
{
    if (numDataBytes + count > 9) {
        // throw or otherwise indicate an error here!
        return;
    }
    // #0a____
    char *dst = buffer + 3 + numDataBytes + numEsc;
    numDataBytes += count;
    while (count-- > 0) {
        char c = *data++;
        switch (c) {
        case 0:
            numEsc++;
            *dst++ = '\\';
            *dst++ = '0';
            break;
        case '#':
            numEsc++;
            *dst++ = '\\';
            *dst++ = '1';
            break;
        case '\n':
            numEsc++;
            *dst++ = '\\';
            *dst++ = '2';
            break;
        case '\\':
            numEsc++;
            *dst++ = '\\';
            *dst++ = '3';
            break;
        default:
            *dst++ = c;
            break;
        }
    }
    buffer[1] = '0' + numDataBytes;
}

char *CmdBuilder::finish()
{
    // #0c\n
    buffer[3+numDataBytes+numEsc] = '\n';
    buffer[3+numDataBytes+numEsc+1] = 0;
    return buffer;
}

#ifdef ARDUINO
CmdLink::CmdLink(HardwareSerial& strm, uint32_t baud)
    : stream{strm}
{
    baudRate = baud;
    sendTimer.begin(sendTimeoutMS);
    recvTimer.begin(recvTimeoutMS);
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

void CmdLink::sendCmdStr(char cmd, const char* str)
{
    builder.begin(cmd);
    builder.pushData(str, strlen(str));
    send();
}

void CmdLink::sendCmdStr(char cmd, const char *str, int len)
{
    builder.begin(cmd);
    builder.pushData(str, len);
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
            recvTimer.begin(recvTimeoutMS);
            return true;
        }
    }
#else
    while (canRead()) {
        if (buffer.push(readChar())) {
            if (debug) {
                (debugStream ? *debugStream : cout) << "Incoming <- ";
                dumpIncoming();
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

void CmdLink::dumpSent()
{
    CmdBuffer tmp;
    const char* buf = builder.getBuffer();
    for (int i = 0; i < builder.length(); i++) {
        tmp.push(buf[i]);
    }
    tmp.dump(debugStream ? *debugStream : std::cout);
}
#endif

void CmdLink::send()
{
    char *sendbuffer = builder.finish();
    int sendlen = builder.length();
#ifdef ARDUINO
    stream.write(sendbuffer, sendlen);
    sendTimer.begin(sendTimeoutMS);
#else

    if (debug) {
        (debugStream ? *debugStream : cout)  << "Sending  -> ";
        dumpSent();
    }
    writer(sendbuffer, sendlen);
#endif
}
