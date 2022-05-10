#include "graphics.h"
#include "lidar.h"
#include <algorithm>
#include <iostream>
#include <iterator>
#include <cctype>
#include <iomanip>
#include <array>
#include "bitrange.h"

#include "rplidar_cmd.h"
#include "sl_crc.h"

constexpr bool verbose = false;

using namespace std;
using namespace mssm;

constexpr int      RESET_TIMEOUT = 1000;
constexpr uint64_t COMM_IDLE_THRESHOLD = 75;
constexpr int      RESPONSE_TIMEOUT = 100;
constexpr int      SEND_TIMEOUT = 100;

// response types
constexpr uint8_t RPLIDAR_RESP_INFO       = 0x04;
constexpr uint8_t RPLIDAR_RESP_HEALTH     = 0x06;
constexpr uint8_t RPLIDAR_RESP_SAMPLERATE = 0x15;
constexpr uint8_t RPLIDAR_RESP_CONFIG     = 0x20;


void displayHex(string data)
{
    if (verbose) {
        bool hasUnprintable = false;
        cout << "Data: ";
        for (size_t i = 0; i < data.size(); i++) {
            if (isprint(static_cast<uint8_t>(data[i]))) {
                cout << data[i];
            }
            else {
                hasUnprintable = true;
            }
        }

        cout << endl;

        if (hasUnprintable) {
            cout << "Hex: ";
            for (size_t i = 0; i < data.size(); i++) {
                cout << hex << static_cast<int>(static_cast<uint8_t>(data[i])) << "(" << (isprint(static_cast<uint8_t>(data[i])) ? data[i] : ' ') << ") ";
            }
            cout << dec <<  endl;
        }
    }
}

template<class _OutIt, class _RanIt>
size_t rplidarCommand(uint8_t cmd, _OutIt _Dest, const _RanIt _First, const _RanIt _Last)
{
    using value_type = typename std::iterator_traits<_OutIt>::value_type;
    using in_value_type = typename std::iterator_traits<_RanIt>::value_type;

    static_assert (sizeof(value_type) == 1, "output iterator value_type must be have sizeof == 1");
    static_assert (sizeof(in_value_type) == 1, "input iterator value_type must be have sizeof == 1");

    uint8_t size = static_cast<uint8_t>(_Last - _First);
    uint8_t checkSum = RPLIDAR_CMD_SYNC_BYTE ^ cmd ^ size;

    *_Dest++ = static_cast<value_type>(RPLIDAR_CMD_SYNC_BYTE);
    *_Dest++ = static_cast<value_type>(cmd);
    *_Dest++ = static_cast<value_type>(size);

    for (_RanIt j = _First; j != _Last; ++j) {
        const auto b = *j;
        *_Dest++ = b;
        checkSum ^= b;
    }

    *_Dest++ = static_cast<value_type>(checkSum);

    return static_cast<size_t>(size+4);  // = size of payload + sync byte + cmd byte + size byte + checksum byte
}

template<class _OutIt> inline
size_t rplidarCommand(uint8_t cmd, _OutIt _Dest)
{
    using value_type = typename std::iterator_traits<_OutIt>::value_type;
    static_assert (sizeof(value_type) == 1, "output iterator value_type must be have sizeof == 1");
    *_Dest++ = static_cast<value_type>(RPLIDAR_CMD_SYNC_BYTE);
    *_Dest++ = static_cast<value_type>(cmd);
    return 2;
}

std::string rplidarCommand(uint8_t cmd, const std::string& payload)
{
    if (payload.empty()) {
        char tmp[2];
        rplidarCommand(cmd, tmp);
        return string(tmp, 2);
    }
    else {
        string s(payload.size()+4,'#');
        rplidarCommand(cmd, s.begin(), payload.begin(), payload.end());
        return s;
    }
}

ResponseParser::ResponseParser()
{
    reset();
}

void ResponseParser::reset()
{
    state.needDescriptor = true;
    state.needData = true;
    state.descriptorSize = 7;  // RPLIDAR descriptor size
    state.hasDescriptor = false;
}

template <typename I>
ResponseParser::ParseResult ResponseParser::parse(I& _First, const I _Last)
{
    using value_type = typename std::iterator_traits<I>::value_type;
    static_assert (sizeof(value_type) == 1, "input iterator value_type must be have sizeof == 1");

    int availableDataSize = _Last - _First;

    if (state.needDescriptor && availableDataSize < state.descriptorSize) {
        return ParseResult::incomplete;  // don't even have enough data for the descriptor
    }

    if (state.needDescriptor) {
        // attempt to read the descriptor

        value_type b1 = *_First++; // byte 1

        if (b1 != static_cast<value_type>(RPLIDAR_ANS_SYNC_BYTE1)) {
            cout << "Bad SYNC_BYTE1: " << std::hex << (int)b1 << dec << endl;
            return ParseResult::error;
        }

        value_type b2 = *_First++; // byte 2

        if (b2 != static_cast<value_type>(RPLIDAR_ANS_SYNC_BYTE2)) {
            cout << "Bad SYNC_BYTE2: " << std::hex << (int)b2 << dec << endl;
            return ParseResult::error;
        }

        int altSize = twiddle::concat<5,0,7,0,7,0,7,0,uint32_t>(_First[3],_First[2],_First[1],_First[0]).val();

        descriptor.size = static_cast<uint32_t>(*_First++);           // byte 3
        descriptor.size |= static_cast<uint32_t>(*_First++) << 8;     // byte 4
        descriptor.size |= static_cast<uint32_t>(*_First++) << 16;    // byte 5

        uint32_t tmp = static_cast<uint32_t>(*_First++);   // byte 6

        descriptor.size |= (tmp & 0x3F) << 24;


        descriptor.type = static_cast<uint8_t>(*_First++);            // byte 7

        descriptor.mode = static_cast<uint8_t>(tmp >> 6);

        availableDataSize -= state.descriptorSize;

        state.needData = descriptor.mode == 0; // mode 0 is Single Response mode, where the descriptor should be followed by exactly one data packet
        state.needDescriptor = false;
        state.hasDescriptor = true;
    }

    if (state.needData &&  static_cast<int>(descriptor.size) > availableDataSize) {
        return ParseResult::incomplete;
    }

    if (state.needData) {
        switch (descriptor.type) {
        case RPLIDAR_RESP_INFO:
            return parseDataInfo(_First);
        case RPLIDAR_RESP_HEALTH:
            return parseDataHealth(_First);
        case RPLIDAR_RESP_SAMPLERATE:
            return parseDataSampleRate(_First);
        case RPLIDAR_RESP_CONFIG:
            return parseConfigResp(_First);
        default:
            cout << "Unknown Descriptor Type: " << (int)descriptor.type << endl;
            return ParseResult::error;
        }
    }

    return ParseResult::complete;
}

// this method assumes that start/end have already been checked to see if there are sufficient bytes of data available
template <typename I>
ResponseParser::ParseResult ResponseParser::parseDataInfo(I& start)
{
    data.devInfo.model = static_cast<uint8_t>(*start++);
    data.devInfo.firmwareMinor = static_cast<uint8_t>(*start++);
    data.devInfo.firmwareMajor = static_cast<uint8_t>(*start++);
    data.devInfo.hardware = static_cast<uint8_t>(*start++);
    for (size_t i = 0; i < sizeof(RespDeviceInfo::serialNumber); i++) {
        data.devInfo.serialNumber[i] = static_cast<char>(*start++);
    }
    return ParseResult::complete;
}

// this method assumes that start/end have already been checked to see if there are sufficient bytes of data available
template <typename I>
ResponseParser::ParseResult ResponseParser::parseDataHealth(I& start)
{
    data.devHealth.status = static_cast<uint8_t>(*start++);
    data.devHealth.errorCode = static_cast<uint16_t>(*start++);
    data.devHealth.errorCode |= (static_cast<uint16_t>(*start++) << 8);
    return ParseResult::complete;
}

// this method assumes that start/end have already been checked to see if there are sufficient bytes of data available
template <typename I>
ResponseParser::ParseResult ResponseParser::parseDataSampleRate(I& start)
{
    data.sampleRate.standard = static_cast<uint16_t>(*start++);
    data.sampleRate.standard |= (static_cast<uint16_t>(*start++) << 8);
    data.sampleRate.express = static_cast<uint16_t>(*start++);
    data.sampleRate.express |= (static_cast<uint16_t>(*start++) << 8);
    return ParseResult::complete;
}

template<typename I>
ResponseParser::ParseResult ResponseParser::parseConfigResp(I &start)
{
    data.configInfo.type = static_cast<uint32_t>(*start++);
    data.configInfo.type |= (static_cast<uint32_t>(*start++) << 8);
    data.configInfo.type |= (static_cast<uint32_t>(*start++) << 16);
    data.configInfo.type |= (static_cast<uint32_t>(*start++) << 24);

    switch (data.configInfo.type) {
    case RPLIDAR_CONF_SCAN_MODE_COUNT:
    case RPLIDAR_CONF_SCAN_MODE_TYPICAL:
        data.configInfo.data1 = static_cast<uint32_t>(*start++);
        data.configInfo.data1 |= (static_cast<uint32_t>(*start++) << 8);
        break;
    case RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE:
    case RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE:
        data.configInfo.data1 = static_cast<uint32_t>(*start++) & 0xFF;
        data.configInfo.data1 |= ((static_cast<uint32_t>(*start++) << 8) & 0xFF00);
        data.configInfo.data1 |= ((static_cast<uint32_t>(*start++) << 16) & 0xFF0000);
        data.configInfo.data1 |= ((static_cast<uint32_t>(*start++) << 24) & 0xFF000000);
        break;
    case RPLIDAR_CONF_SCAN_MODE_ANS_TYPE:
        data.configInfo.data1 = static_cast<uint32_t>(*start++)& 0xFF;;
        break;
    case RPLIDAR_CONF_SCAN_MODE_NAME:
    {
        string s;
        char c = (*start++);
        while (c) {
            s.append(1, c);
            c = (*start++);
        }
        respDataName = s;
        cout << "Got Name: " << s << endl;
    }

        break;
    }

    return ParseResult::complete;
}

template<class _RanIt>
bool rplidarParseScan(_RanIt& _First, const _RanIt _Last, int& quality, double& angle, double& distance, bool& is360start)
{
    using value_type = typename std::iterator_traits<_RanIt>::value_type;
    static_assert (sizeof(value_type) == 1, "input iterator value_type must be have sizeof == 1");
    if (_Last - _First < 5) {
        cout << "Bad size1: " << int(_Last - _First) << endl;
        return false;
    }

    int q = static_cast<uint8_t>(*_First++);

    switch (q & 0x03) {
    case 0:
    case 3:
        // checksum failure!
        cout << "Checksum failure 1" << endl;
        return false;
    case 1:
        // First reading from a 360 scan
        // if (verbose) cout << "New Scan: ";
        is360start = true;
        break;
    case 2:
        // additional data for a 360 scan
        //        if (verbose) cout << "Continued Scan: ";
        is360start = false;
        break;
    }

    quality = q >> 2;

    uint8_t a1 = static_cast<uint8_t>(*_First++);
    uint8_t a2 = static_cast<uint8_t>(*_First++);

    angle = ((a1 | (a2 << 8))>>1) / 64.0;

    if (!(a1 & 1)) {
        // checksum failure!!
        cout << "Checksum failure 2" << endl;
        return false;
    }

    uint8_t d1 = static_cast<uint8_t>(*_First++);
    uint8_t d2 = static_cast<uint8_t>(*_First++);

    distance = (d1 | (d2 << 8)) / 4.0;

    if (verbose) cout << std::dec << quality << " " << angle << " " << distance << endl;

    return true;
}



template<class _RanIt>
bool rplidarParseExpressScan(_RanIt& _First, const _RanIt _Last, double& startAngle, bool& isNew, std::array<ExpressPoint, 32>& packet)
{
    using value_type = typename std::iterator_traits<_RanIt>::value_type;
    static_assert (sizeof(value_type) == 1, "input iterator value_type must be have sizeof == 1");

    if (_Last - _First < 5) {
        cout << "Bad size1: " << int(_Last - _First) << endl;
        return false;
    }

    uint8_t b0 = static_cast<uint8_t>(*_First++);
    uint8_t b1 = static_cast<uint8_t>(*_First++);

    if ((b0 & 0xF0) != 0xA0 || (b1 & 0xF0) != 0x50) {
        cout << "Express packet sync failure" << endl;
        return false;
    }

    int checksum = ((b1 & 0x0F) << 4) | (b0 & 0x0F);

    uint8_t b2 = static_cast<uint8_t>(*_First++);
    uint8_t b3 = static_cast<uint8_t>(*_First++);

    int startAngleI = ((b3 & 0x7F) << 8) | b2;

    startAngle = startAngleI / 64.0;

    isNew = (b3 & 0x80) ? true : false;

    // https://en.wikipedia.org/wiki/Q_(number_format)

    for (size_t i = 0; i < 16; i++) { // read 16 cabins

        uint8_t c0 = static_cast<uint8_t>(*_First++);
        uint8_t c1 = static_cast<uint8_t>(*_First++);
        uint8_t c2 = static_cast<uint8_t>(*_First++);
        uint8_t c3 = static_cast<uint8_t>(*_First++);
        uint8_t c4 = static_cast<uint8_t>(*_First++);

        // |000000|

        packet[i*2].angleComp =  static_cast<uint8_t>(((c0 & 0x03) << 4) | (c4 & 0x0F));
        packet[i*2].distance = static_cast<uint16_t>((c1 << 6) | (c0 >> 2));
        packet[i*2+1].angleComp = static_cast<uint8_t>(((c2 & 0x03) << 4) | (c4 >> 4));
        packet[i*2+1].distance = static_cast<uint16_t>((c3 << 6) | (c2 >> 2));

        //       packet[i*2].angleComp = 0;
        //       packet[i*2+1].angleComp = 0;

    }

    return true;
}

std::string rawString(uint16_t value)
{
    return string(reinterpret_cast<char*>(&value), sizeof(value));
}

std::string rawString(uint32_t value)
{
    return string(reinterpret_cast<char*>(&value), sizeof(value));
}

std::string getConfCmdString(int subCmd, uint16_t data)
{
    char buffer[6] = {0,0,0,0,0,0};
    buffer[0] = reinterpret_cast<char*>(&subCmd)[0];  // lsb
    buffer[4] = reinterpret_cast<char*>(&data)[0];
    buffer[5] = reinterpret_cast<char*>(&data)[1];

    int useLen = 6;

    switch (subCmd) {
    case RPLIDAR_CONF_SCAN_MODE_COUNT:
    case RPLIDAR_CONF_SCAN_MODE_TYPICAL:
        useLen = 4;
        break;
    case RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE:
    case RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE:
    case RPLIDAR_CONF_SCAN_MODE_ANS_TYPE:
    case RPLIDAR_CONF_SCAN_MODE_NAME:
        useLen = 6;
        break;
    }

    return string(buffer, useLen);
}

int packConfCmdToArg1(int subCmd, uint16_t data)
{
    int arg1 = subCmd + (static_cast<int>(data) << 16);
    return arg1;
}

void unpackConfCmdFromArg1(int arg1, int& subCmd, uint16_t& data)
{
    subCmd = arg1 & 0xFFFF;
    data = arg1 >> 16;
}

std::string getConfCmdString(int packedCmd)
{
    int subCmd;
    uint16_t data;
    unpackConfCmdFromArg1(packedCmd, subCmd, data);
    return getConfCmdString(subCmd, data);
}

Lidar::Lidar(const std::string& portName, std::function<void (bool startSweep, const LidarData&)> handler)
    : handler{handler}
{
    port.open(portName, 115200);
    //setState(LidarMode::connect);
    prevTime = std::chrono::steady_clock::now();
    currTime = prevTime;
    lastTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

Lidar::~Lidar()
{
    send(RPLIDAR_CMD_SET_MOTOR_PWM, rawString((uint16_t)0), false);
    send(RPLIDAR_CMD_STOP,"",false);
}


void Lidar::update()
{
    currTime = std::chrono::steady_clock::now();
    auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(currTime - prevTime);
    prevTime = currTime;

    if (port.stillWriting()) {
        cout << "CheckTimeout here?" << endl;
        // discard incoming data ('cause, why are we writing if we are still
        // waiting on more data from a previous command
        while (port.canRead()) {
            port.read();
        }
        return;
    }

    switch (cmdState) {
    case CmdState::none:
        break;
    case CmdState::transmitting:
        commIdleTime = 0;
        timeoutTime = activeCommand.responseTimeout;
        cmdState = CmdState::waitResponse;
        cout << "Finished Cmd Transmit (wait response)" << endl;
        break;
    case CmdState::transmittingNR:
        commIdleTime = 0;
        timeoutTime = activeCommand.responseTimeout;
        cmdState = CmdState::none;
        cout << "Finished Cmd Transmit (no response expected)" << endl;
        if (activeCommand.id != LidarCommandId::reset) {
            activeCommand.id = LidarCommandId::none;
        }
        break;
    case CmdState::waitResponse:
        break;
    }

    switch (state) {
    case LidarState::needReset:
        cmdReset();
        state = LidarState::waitingAfterReset;
        cout << "needReset: Queued Reset Command" << endl;
        break;
    case LidarState::waitingAfterReset:
        while (port.canRead()) {
            port.read();
        }
        if (updateAndCheckTimeout(elapsedMs.count())) {
            state = LidarState::waitingForIdle;
            cout << "waitingForIdle" << endl;
            timeoutTime = 20;
            activeCommand.id = LidarCommandId::none;
        }
        break;
    case LidarState::waitingForIdle:
        if (port.canRead()) {
            cout << "Consuming" << endl;
            timeoutTime = 20; // let's wait 20 ms after last read to make sure there's no more incoming data
            port.read();
        }
        else if (updateAndCheckTimeout(elapsedMs.count())) {
            cout << "got idle, queue some more commands" << endl;
            state = LidarState::waitingOnModeReq;
            cmdMotorSpeed(0);
            cmdGetHealth();
            cmdGetInfo();
            cmdReqTypicalMode();
        }
        break;
    case LidarState::waitingOnModeReq:
        if (updateAndCheckTimeout(elapsedMs.count())) {
            cout << "Timeout!!  resetting" << endl;
            state = LidarState::needReset;
        }
        break;
    case LidarState::stopped:
        break;
    case LidarState::waitingForSpin:
        if (updateAndCheckTimeout(elapsedMs.count())) {
            cout << "Hopefully spinning now" << endl;
            state = LidarState::spinning;
        }
        else {
            cout << timeoutTime << endl;
        }
        break;
    case LidarState::spinning:
        break;
    case LidarState::scanning:
        if (updateAndCheckCommIdle(elapsedMs.count())) {
            cout << "Gone idle while scanning.  Resetting" << endl;
            state = LidarState::needReset;
        }
        break;
    }

    if (port.canRead()) {
        commIdleTime = 0;
        string data = port.read();
        parse(data);
    }

    // see if we need to initiate another command from the queue
    if (activeCommand.id == LidarCommandId::none) {
        if (commands.size() > 0) {
            activeCommand = commands[0];
            commands.erase(commands.begin()); // TODO inefficient!  Use queue
            switch (activeCommand.id) {
            case LidarCommandId::motorSpeed:
                setMotorUpdateState(activeCommand.arg1);
                break;
            case LidarCommandId::reset:
                send(RPLIDAR_CMD_RESET, "", false);
                break;
            case LidarCommandId::beginScan:
                if (isScanning) {
                    cout << "Ignoring redundant scan command" << endl;
                    activeCommand.id = LidarCommandId::none;
                }
                else {
                    send(RPLIDAR_CMD_SCAN, "", true);
                }
                break;
            case LidarCommandId::endScan:
                send(RPLIDAR_CMD_STOP, "", false);
                break;
            case LidarCommandId::getInfo:
                send(RPLIDAR_CMD_GET_DEVICE_INFO, "", true);
                break;
            case LidarCommandId::getHealth:
                send(RPLIDAR_CMD_GET_DEVICE_HEALTH, "", true);
                break;
            case LidarCommandId::getSampleRate:
                send(RPLIDAR_CMD_GET_SAMPLERATE, "", true);
                break;
            case LidarCommandId::beginExpressScan:
            {
                char packet[5] = { static_cast<char>(activeCommand.arg1), 0, 0, 0, 0};
                send(RPLIDAR_CMD_EXPRESS_SCAN, string(packet, 5), true);
            }
                break;
            case LidarCommandId::none:
                cout << "Shouldn't really happen" << endl;
                break;
            case LidarCommandId::getConfig:
                send(RPLIDAR_CMD_GET_LIDAR_CONF, getConfCmdString(activeCommand.arg1), true);
                break;
            }
        }
    }
}



void Lidar::processResponse(const ResponseParser::Descriptor& desc, const ResponseParser::RespData& data)
{
    switch (desc.type) {
    case RPLIDAR_RESP_INFO:
        cout << "Model: " << static_cast<int>(data.devInfo.model) << endl;
        cout << "Hardware Version: " << static_cast<int>(data.devInfo.hardware) << endl;
        cout << "Firmware: " << static_cast<int>(data.devInfo.firmwareMajor) << "." << static_cast<int>(data.devInfo.firmwareMinor) << endl;
        cout << "Serial: ";
        for (size_t i = 0; i < 16; i++) {
            cout << hex << setw(2) << setfill('0') << uppercase << static_cast<unsigned int>(data.devInfo.serialNumber[i]) << dec;
        }
        cout << endl;
        break;
    case RPLIDAR_RESP_HEALTH:
        switch (data.devHealth.status) {
        case 0:
            cout << "Health ok" << endl;
            break;
        case 1:
            cout << "Health warning" << endl;
            cout << "Warning code: " << hex << static_cast<int>(data.devHealth.errorCode) << dec << endl;
            break;
        case 2:
            cout << "Health error! (Protection Stop State)" << endl;
            cout << "Error code: " << hex << static_cast<int>(data.devHealth.errorCode) << dec << endl;
        }
        break;
    case RPLIDAR_RESP_SAMPLERATE:
        cout << "Sample Rates    Standard: " << data.sampleRate.standard << "  Express: " << data.sampleRate.express << endl;
        break;
    case RPLIDAR_RESP_CONFIG:
        cout << "Config data: " << data.configInfo.data1 << " Hex: " << std::hex << data.configInfo.data1 << std::dec << endl;
        switch (data.configInfo.type) {
        case RPLIDAR_CONF_SCAN_MODE_TYPICAL:
            typicalScanMode = data.configInfo.data1;
            if (state == LidarState::waitingOnModeReq) {
                state = LidarState::stopped;
                cout << "Got req mode: entering state stopped" << endl;
            }
            break;
        }
        break;
    default:
  //      cout << "Resp: " << std::hex << (int)desc.type << std::dec << endl;
        break;
    }
}

bool Lidar::updateAndCheckTimeout(int elapsedMs)
{
    if (timeoutTime > 0) {
        timeoutTime -= elapsedMs;
        if (timeoutTime <= 0) {
            timeoutTime = 0;
            cout << "Timeout of some sort" << endl;
            return true;
        }
    }
    return false;
}

bool Lidar::updateAndCheckCommIdle(int elapsedMs)
{
    bool wasIdle = commIdleTime > COMM_IDLE_THRESHOLD;
    if (wasIdle) {
        return false;
    }
    commIdleTime += elapsedMs;
    if (!wasIdle && (commIdleTime > COMM_IDLE_THRESHOLD))
    {
        if (verbose) cout << "Comm has gone idle" << endl;
        return true;
    }
    return false;
}

void Lidar::setMotorUpdateState(int speed)
{
    switch (state) {
    case LidarState::stopped:
        send(RPLIDAR_CMD_SET_MOTOR_PWM, rawString(static_cast<uint16_t>(speed)), false);
        if (speed > 0) {
            state = LidarState::waitingForSpin;
            timeoutTime = 1000;
            cout << "Timeout set to 1000" << endl;
        }
        break;
    case LidarState::spinning:
        send(RPLIDAR_CMD_SET_MOTOR_PWM, rawString(static_cast<uint16_t>(speed)), false);
        if (speed == 0) {
            state = LidarState::stopped;
        }
        break;
    case LidarState::scanning:
        send(RPLIDAR_CMD_SET_MOTOR_PWM, rawString(static_cast<uint16_t>(speed)), false);
        if (speed == 0) {
            state = LidarState::stopped;
        }
        break;
    default:
        cout << "Ignoring motor command: invalid state2" << endl;
 //       send(RPLIDAR_CMD_SET_MOTOR_PWM, rawString(static_cast<uint16_t>(speed)), false);
        break;
    }

}

bool Lidar::send(uint8_t cmd, const std::string& data, bool hasResponse)
{
    if (cmdState != CmdState::none) {
        cout << "Sending Cmd when cmdState != None!!!" << endl;
    }
    responseParser.reset();
    cout << "Sending CMD: " << hex << static_cast<int>(cmd) << dec << endl;
    string dataPacket = rplidarCommand(cmd, data);
    port.write(dataPacket);
    //serialPort.flush();
    timeoutTime = SEND_TIMEOUT;
    cmdState = hasResponse ? CmdState::transmitting : CmdState::transmittingNR;
    return true;
}

double angleDiff(double a, double a1)
{
    if (a <= a1) {
        return a1-a;
    }
    else {
        return 360.0+a1-a;
    }
}

void Lidar::parse(const std::string &str)
{
    if (bufferedData.size() > 0) {
        //cout << "Appending to existing data" << endl;

    }
    bufferedData.append(str);

    if (activeCommand.id == LidarCommandId::reset) {
        //sendEvent(0, 0, 0, bufferedData);
        bufferedData.clear();
        return;
    }

    if (cmdState == CmdState::waitResponse) {
        auto i = bufferedData.begin();
        switch (responseParser.parse(i, bufferedData.end())) {
        case ResponseParser::ParseResult::incomplete:
            cout << "Need a bit more data" << endl;
            break;
        case ResponseParser::ParseResult::complete:
            cout << "Received Response: processing it" << endl;
            timeoutTime = 0;
            processResponse(responseParser.descriptor, responseParser.data);
            isScanning = responseParser.descriptor.mode == 0x01; // multiple response mode
            activeCommand.id = LidarCommandId::none;
            cmdState = CmdState::none;
            break;
        case ResponseParser::ParseResult::error:
            cout << "Parse Failed!" << endl;
            break;
        }
        bufferedData.erase(bufferedData.begin(), i); // erase used data from buffer

        if (responseParser.hasDescriptor()) {
            processor.init(responseParser.descriptor.type, responseParser.descriptor.size);
        }

        return;
    }

    int remaining = 0;

    sl_result ans;

    do {
        ans = processor.processIncoming(bufferedData.size(), reinterpret_cast<uint8_t*>(bufferedData.data()), remaining, handler);

        if (remaining > 0) {
            bufferedData = bufferedData.substr(bufferedData.size()-remaining);
        }
        else {
            bufferedData.clear();
        }

    } while (ans == SL_RESULT_OK && !bufferedData.empty());
}

void Lidar::cmdMotorSpeed(int speed) {
    switch (state) {
    case LidarState::stopped:
    case LidarState::spinning:
    case LidarState::scanning:
        commands.push_back(LidarCommand{LidarCommandId::motorSpeed, speed, 0});
        break;
    default:
        cout << "Ignoring motor command: invalid state: " << getStateString() << endl;
        break;
    }
}

void Lidar::cmdReset()
{
    //if (verbose) cout << "Queueing reset Command" << endl;
    isScanning = false;
    activeCommand.id = LidarCommandId::none;
    commands.clear();
    commands.push_back(LidarCommand{LidarCommandId::reset, 0, RESET_TIMEOUT});
}

void Lidar::cmdBeginScan()
{
    //if (verbose) cout << "Queueing scan Command" << endl;
    commands.push_back(LidarCommand{LidarCommandId::beginScan, 0, RESPONSE_TIMEOUT});
}

void Lidar::cmdEndScan()
{
    //if (verbose) cout << "Queueing scan Command" << endl;
    commands.push_back(LidarCommand{LidarCommandId::endScan, 0, RESPONSE_TIMEOUT});
}

void Lidar::cmdGetInfo()
{
    commands.push_back(LidarCommand{LidarCommandId::getInfo, 0, RESPONSE_TIMEOUT});
}

void Lidar::cmdGetHealth()
{
    commands.push_back(LidarCommand{LidarCommandId::getHealth, 0, RESPONSE_TIMEOUT});
}

void Lidar::cmdGetSampleRate()
{
    commands.push_back(LidarCommand{LidarCommandId::getSampleRate, 0, RESPONSE_TIMEOUT});
}

void Lidar::cmdBeginExpressScan(int expressMode)
{
    commands.push_back(LidarCommand{LidarCommandId::beginExpressScan, expressMode, RESPONSE_TIMEOUT});
}


void Lidar::cmdReqConfModeCount()
{
    commands.push_back(LidarCommand{LidarCommandId::getConfig, packConfCmdToArg1(RPLIDAR_CONF_SCAN_MODE_COUNT, 0), RESPONSE_TIMEOUT});
}

void Lidar::cmdReqConfUsPerSample(int mode)
{
    commands.push_back(LidarCommand{LidarCommandId::getConfig, packConfCmdToArg1(RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE, mode), RESPONSE_TIMEOUT});
}

void Lidar::cmdReqConfAnsType(int mode)
{
    commands.push_back(LidarCommand{LidarCommandId::getConfig, packConfCmdToArg1(RPLIDAR_CONF_SCAN_MODE_ANS_TYPE, mode), RESPONSE_TIMEOUT});

}

void Lidar::cmdReqConfName(int mode)
{
    commands.push_back(LidarCommand{LidarCommandId::getConfig, packConfCmdToArg1(RPLIDAR_CONF_SCAN_MODE_NAME, mode), RESPONSE_TIMEOUT});

}

void Lidar::cmdReqTypicalMode()
{
    commands.push_back(LidarCommand{LidarCommandId::getConfig, packConfCmdToArg1(RPLIDAR_CONF_SCAN_MODE_TYPICAL, 0), RESPONSE_TIMEOUT});

}

Lidar::LidarState Lidar::getState() const
{
    return state;
}

string Lidar::getStateString() const
{
    switch (state) {
    case LidarState::needReset:         return "needReset";
    case LidarState::waitingAfterReset: return "waitingAfterReset";
    case LidarState::waitingForIdle:    return "waitingForIdle";
    case LidarState::waitingOnModeReq:  return "waitingOnModeReq";
    case LidarState::stopped:           return "stopped";
    case LidarState::waitingForSpin:    return "waitingOnSpin";
    case LidarState::spinning:          return "spinning";
    case LidarState::scanning:          return "scanning";
    default:
        return "UNKNOWNSTATE: FIXTHIS";
    }
}



static void convert(const sl_lidar_response_measurement_node_t& from, sl_lidar_response_measurement_node_hq_t& to)
{
    to.angle_z_q14 = (((from.angle_q6_checkbit) >> SL_LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) << 8) / 90;  //transfer to q14 Z-angle
    to.dist_mm_q2 = from.distance_q2;
    to.flag = (from.sync_quality & SL_LIDAR_RESP_MEASUREMENT_SYNCBIT);  // trasfer syncbit to HQ flag field
    to.quality = (from.sync_quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;  //remove the last two bits and then make quality from 0-63 to 0-255
}

static void convert(const sl_lidar_response_measurement_node_hq_t& from, sl_lidar_response_measurement_node_t& to)
{
    to.sync_quality = (from.flag & SL_LIDAR_RESP_MEASUREMENT_SYNCBIT) | ((from.quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
    to.angle_q6_checkbit = 1 | (((from.angle_z_q14 * 90) >> 8) << SL_LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT);
    to.distance_q2 = from.dist_mm_q2 > sl_u16(-1) ? sl_u16(0) : sl_u16(from.dist_mm_q2);
}

static uint32_t _varbitscale_decode(uint32_t scaled, uint32_t & scaleLevel)
{
    static const uint32_t VBS_SCALED_BASE[] = {
        SL_LIDAR_VARBITSCALE_X16_DEST_VAL,
        SL_LIDAR_VARBITSCALE_X8_DEST_VAL,
        SL_LIDAR_VARBITSCALE_X4_DEST_VAL,
        SL_LIDAR_VARBITSCALE_X2_DEST_VAL,
        0,
    };

    static const uint32_t VBS_SCALED_LVL[] = {
        4,
        3,
        2,
        1,
        0,
    };

    static const uint32_t VBS_TARGET_BASE[] = {
        (0x1 << SL_LIDAR_VARBITSCALE_X16_SRC_BIT),
        (0x1 << SL_LIDAR_VARBITSCALE_X8_SRC_BIT),
        (0x1 << SL_LIDAR_VARBITSCALE_X4_SRC_BIT),
        (0x1 << SL_LIDAR_VARBITSCALE_X2_SRC_BIT),
        0,
    };

    for (size_t i = 0; i < _countof(VBS_SCALED_BASE); ++i) {
        int remain = ((int)scaled - (int)VBS_SCALED_BASE[i]);
        if (remain >= 0) {
            scaleLevel = VBS_SCALED_LVL[i];
            return VBS_TARGET_BASE[i] + (remain << scaleLevel);
        }
    }
    return 0;
}

static inline float getAngle(const sl_lidar_response_measurement_node_t& node)
{
    return (node.angle_q6_checkbit >> SL_LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.f;
}

static inline void setAngle(sl_lidar_response_measurement_node_t& node, float v)
{
    sl_u16 checkbit = node.angle_q6_checkbit & SL_LIDAR_RESP_MEASUREMENT_CHECKBIT;
    node.angle_q6_checkbit = (((sl_u16)(v * 64.0f)) << SL_LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) | checkbit;
}

static inline float getAngle(const sl_lidar_response_measurement_node_hq_t& node)
{
    return node.angle_z_q14 * 90.f / 16384.f;
}

static inline void setAngle(sl_lidar_response_measurement_node_hq_t& node, float v)
{
    node.angle_z_q14 = uint32_t(v * 16384.f / 90.f);
}

static inline sl_u16 getDistanceQ2(const sl_lidar_response_measurement_node_t& node)
{
    return node.distance_q2;
}

static inline uint32_t getDistanceQ2(const sl_lidar_response_measurement_node_hq_t& node)
{
    return node.dist_mm_q2;
}

template <class TNode>
static bool angleLessThan(const TNode& a, const TNode& b)
{
    return getAngle(a) < getAngle(b);
}



template < class TNode >
static sl_result ascendScanData_(TNode * nodebuffer, size_t count)
{
    float inc_origin_angle = 360.f / count;
    size_t i = 0;

    //Tune head
    for (i = 0; i < count; i++) {
        if (getDistanceQ2(nodebuffer[i]) == 0) {
            continue;
        }
        else {
            while (i != 0) {
                i--;
                float expect_angle = getAngle(nodebuffer[i + 1]) - inc_origin_angle;
                if (expect_angle < 0.0f) expect_angle = 0.0f;
                setAngle(nodebuffer[i], expect_angle);
            }
            break;
        }
    }

    // all the data is invalid
    if (i == count) return SL_RESULT_OPERATION_FAIL;

    //Tune tail
    for (i = count - 1; i >= 0; i--) {
        if (getDistanceQ2(nodebuffer[i]) == 0) {
            continue;
        }
        else {
            while (i != (count - 1)) {
                i++;
                float expect_angle = getAngle(nodebuffer[i - 1]) + inc_origin_angle;
                if (expect_angle > 360.0f) expect_angle -= 360.0f;
                setAngle(nodebuffer[i], expect_angle);
            }
            break;
        }
    }

    //Fill invalid angle in the scan
    float frontAngle = getAngle(nodebuffer[0]);
    for (i = 1; i < count; i++) {
        if (getDistanceQ2(nodebuffer[i]) == 0) {
            float expect_angle = frontAngle + i * inc_origin_angle;
            if (expect_angle > 360.0f) expect_angle -= 360.0f;
            setAngle(nodebuffer[i], expect_angle);
        }
    }

    // Reorder the scan according to the angle value
    std::sort(nodebuffer, nodebuffer + count, &angleLessThan<TNode>);

    return SL_RESULT_OK;
}

#define  MAX_SCAN_NODES  (8192)

class StdProcessor : public LidarProcessor {
    int  recvPos = 0;

    bool _isScanning{false};
    sl_lidar_response_measurement_node_t node;

    virtual sl_result processIncoming(int recvSize, uint8_t* recvData, int& remainData, std::function<void(bool startSweep, const LidarData& point)> handler) override;
    //  sl_result _waitScanData(int recvSize, uint8_t* recvData, int& remainData);
    sl_result _waitNode(int recvSize, uint8_t* recvData, int& remainData);

};

sl_result StdProcessor::_waitNode(int recvSize, uint8_t* recvData, int& remainData)
{
    uint8_t *nodeBuffer = (uint8_t*)&node;

    size_t remainSize = sizeof(sl_lidar_response_measurement_node_t) - recvPos;

    if (recvSize > remainSize) {
        remainData = recvSize-remainSize;
        recvSize = remainSize;
    }
    else {
        remainData = 0;
    }

    for (size_t pos = 0; pos < recvSize; ++pos) {
        uint8_t currentByte = recvData[pos];
        switch (recvPos) {
        case 0: // expect the sync bit and its reverse in this byte
        {
            uint8_t tmp = (currentByte >> 1);
            if ((tmp ^ currentByte) & 0x1) {
                // pass
            }
            else {
                continue;
            }

        }
            break;
        case 1: // expect the highest bit to be 1
        {
            if (currentByte & SL_LIDAR_RESP_MEASUREMENT_CHECKBIT) {
                // pass
            }
            else {
                recvPos = 0;
                continue;
            }
        }
            break;
        }
        nodeBuffer[recvPos++] = currentByte;

        if (recvPos == sizeof(sl_lidar_response_measurement_node_t)) {
            recvPos = 0;
            return SL_RESULT_OK;
        }
    }

    return SL_RESULT_OPERATION_TIMEOUT;  // use timeout to indicate waiting for more data

}


//sl_result StdProcessor::_waitScanData(int recvSize, uint8_t* recvData, int& remainData)
//{
//    if (!_isConnected) {
//        count = 0;
//        return SL_RESULT_OPERATION_FAIL;
//    }

//    size_t   recvNodeCount = 0;
//    uint32_t     startTs = getms();
//    uint32_t     waitTime;
//    Result<nullptr_t> ans = SL_RESULT_OK;

//    while ((waitTime = getms() - startTs) <= timeout && recvNodeCount < count) {
//        sl_lidar_response_measurement_node_t node;
//        ans = _waitNode(&node, timeout - waitTime);
//        if (!ans) return ans;

//        nodebuffer[recvNodeCount++] = node;

//        if (recvNodeCount == count) return SL_RESULT_OK;
//    }
//    count = recvNodeCount;
//    return SL_RESULT_OPERATION_TIMEOUT;
//}


sl_result StdProcessor::processIncoming(int recvSize, uint8_t* recvData, int& remainData, std::function<void(bool startSweep, const LidarData& point)> handler)
{
    sl_result ans = _waitNode(recvSize, recvData, remainData);

    if (ans != SL_RESULT_OK) {
        return ans;
    }

    //bool syncBit = node.sync_quality & SL_LIDAR_RESP_MEASUREMENT_SYNCBIT;

    sl_lidar_response_measurement_node_hq_t nodeHq;
    convert(node, nodeHq);

    LidarData data;
    data.angle = getAngle(nodeHq);
    data.distance = getDistanceQ2(nodeHq)/4;
    data.quality = nodeHq.quality;

    if (data.isOk() || (nodeHq.flag & 0x01)) {
        handler(nodeHq.flag & 0x01, data);
    }

    return SL_RESULT_OK;
}


class HQDataProcessor  : public LidarProcessor {
    constexpr static size_t maxCount{256};
    sl_lidar_response_measurement_node_hq_t  local_buf[maxCount];
    size_t                                   count = maxCount;

    int  recvPos = 0;

    bool _isScanning{false};

    sl_lidar_response_hq_capsule_measurement_nodes_t    hq_node;

    sl_lidar_response_measurement_node_hq_t   _cached_scan_node_hq_buf[8192];
    size_t                                   _cached_scan_node_hq_count;    sl_lidar_response_hq_capsule_measurement_nodes_t _cached_previous_Hqdata;

    bool                                         _is_previous_HqdataRdy;

    //    sl_lidar_response_measurement_node_hq_t   _cached_scan_node_hq_buf_for_interval_retrieve[8192];
    //    size_t                                   _cached_scan_node_hq_count_for_interval_retrieve;

    void _HqToNormal(const sl_lidar_response_hq_capsule_measurement_nodes_t & node_hq, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount);
    sl_result _waitHqNode(int recvSize, uint8_t* recvData, int& remainData);
    sl_result processIncoming(int recvSize, uint8_t* recvData, int& remainData, std::function<void(bool startSweep, const LidarData& point)> handler);

};

sl_result HQDataProcessor::_waitHqNode(int recvSize, uint8_t* recvData, int& remainData)
//sl_lidar_response_hq_capsule_measurement_nodes_t & node, uint32_t timeout)
{
    uint8_t *nodeBuffer = (uint8_t*)&hq_node;

    size_t remainSize = sizeof(sl_lidar_response_hq_capsule_measurement_nodes_t) - recvPos;

    if (recvSize > remainSize) {
        remainData = recvSize-remainSize;
        recvSize = remainSize;
    }
    else {
        remainData = 0;
    }

    for (size_t pos = 0; pos < recvSize; ++pos) {
        uint8_t currentByte = recvData[pos];
        switch (recvPos) {
        case 0: // expect the sync byte
        {
            uint8_t tmp = (currentByte);
            if (tmp == SL_LIDAR_RESP_MEASUREMENT_HQ_SYNC) {
                // pass
            }
            else {
                recvPos = 0;
                _is_previous_HqdataRdy = false;
                continue;
            }
        }
            break;
        case sizeof(sl_lidar_response_hq_capsule_measurement_nodes_t) - 1 - 4:
        {

        }
            break;
        case sizeof(sl_lidar_response_hq_capsule_measurement_nodes_t) - 1:
        {

        }
            break;
        }
        nodeBuffer[recvPos++] = currentByte;

        if (recvPos == sizeof(sl_lidar_response_hq_capsule_measurement_nodes_t)) {
            uint32_t crcCalc2 = sl::crc32::getResult(nodeBuffer, sizeof(sl_lidar_response_hq_capsule_measurement_nodes_t) - 4);

            if (crcCalc2 == hq_node.crc32) {
                _is_previous_HqdataRdy = true;
                recvPos = 0;
                return SL_RESULT_OK;
            }
            else {
                _is_previous_HqdataRdy = false;
                recvPos = 0;
                return SL_RESULT_INVALID_DATA;
            }

        }
    }

    _is_previous_HqdataRdy = false;
    return SL_RESULT_OPERATION_TIMEOUT;  // use timeout to indicate waiting for more data
}


sl_result HQDataProcessor::processIncoming(int recvSize, uint8_t *recvData, int &remainData, std::function<void(bool startSweep, const LidarData& point)> handler)
{
    sl_result ans = _waitHqNode(recvSize, recvData, remainData);

    if (ans != SL_RESULT_OK) {
        count = 0;
        return ans;
    }

    count = maxCount;

    _HqToNormal(hq_node, local_buf, count);

    //cout << count << " HQ: data in local_buf.  What to do with it?" << endl;

    for (int i = 0; i < count; i++) {
        LidarData data;
        data.angle = getAngle(local_buf[i]);
        data.distance = getDistanceQ2(local_buf[i])/4;
        data.quality = local_buf[i].quality;
        if (data.isOk()  || (local_buf[i].flag & 0x01)) {
            handler(local_buf[i].flag & 0x01, data);
        }
    }

    return SL_RESULT_OK;
}

void HQDataProcessor::_HqToNormal(const sl_lidar_response_hq_capsule_measurement_nodes_t & node_hq, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount)
{
    nodeCount = 0;
    if (_is_previous_HqdataRdy) {
        for (size_t pos = 0; pos < _countof(_cached_previous_Hqdata.node_hq); ++pos) {
            nodebuffer[nodeCount++] = node_hq.node_hq[pos];
        }
    }
    _cached_previous_Hqdata = node_hq;
    _is_previous_HqdataRdy = true;

}

class UltraCapsuleProcessor  : public LidarProcessor {
    constexpr static size_t maxCount{256};
    sl_lidar_response_measurement_node_hq_t  local_buf[maxCount];
    size_t                                   count = maxCount;

    int  recvPos = 0;
    sl_lidar_response_ultra_capsule_measurement_nodes_t    ultra_capsule_node;

    bool _isScanning{false};
    sl_lidar_response_ultra_capsule_measurement_nodes_t _cached_previous_ultracapsuledata;
    bool                                         _is_previous_capsuledataRdy;

    void _ultraCapsuleToNormal(const sl_lidar_response_ultra_capsule_measurement_nodes_t & capsule, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount);
    sl_result _waitUltraCapsuledNode(int recvSize, uint8_t* recvData, int& remainData);

    sl_result processIncoming(int recvSize, uint8_t* recvData, int& remainData, std::function<void(bool startSweep, const LidarData& point)> handler);
};

sl_result UltraCapsuleProcessor::_waitUltraCapsuledNode(int recvSize, uint8_t* recvData, int& remainData)
{
    uint8_t *nodeBuffer = (uint8_t*)&ultra_capsule_node;

    size_t remainSize = sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t) - recvPos;

    if (recvSize > remainSize) {
        remainData = recvSize-remainSize;
        recvSize = remainSize;
    }
    else {
        remainData = 0;
    }

    for (size_t pos = 0; pos < recvSize; ++pos) {
        uint8_t currentByte = recvData[pos];
        switch (recvPos) {
        case 0: // expect the sync bit 1
        {
            uint8_t tmp = (currentByte >> 4);
            if (tmp == SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_1) {
                // pass
            }
            else {
                _is_previous_capsuledataRdy = false;
                continue;
            }
        }
            break;
        case 1: // expect the sync bit 2
        {
            uint8_t tmp = (currentByte >> 4);
            if (tmp == SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
                // pass
            }
            else {
                recvPos = 0;
                _is_previous_capsuledataRdy = false;
                continue;
            }
        }
            break;
        }

        nodeBuffer[recvPos++] = currentByte;

        if (recvPos == sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t)) {
            // calc the checksum ...
            uint8_t checksum = 0;
            uint8_t recvChecksum = ((ultra_capsule_node.s_checksum_1 & 0xF) | (ultra_capsule_node.s_checksum_2 << 4));

            for (size_t cpos = offsetof(sl_lidar_response_ultra_capsule_measurement_nodes_t, start_angle_sync_q6);
                 cpos < sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t); ++cpos)
            {
                checksum ^= nodeBuffer[cpos];
            }

            if (recvChecksum == checksum) {
                // only consider vaild if the checksum matches...
                if (ultra_capsule_node.start_angle_sync_q6 & SL_LIDAR_RESP_MEASUREMENT_EXP_SYNCBIT) {
                    // this is the first capsule frame in logic, discard the previous cached data...
                    _is_previous_capsuledataRdy = false;
                    recvPos = 0;
                    return SL_RESULT_OK;
                }
                recvPos = 0;
                return SL_RESULT_OK;
            }
            _is_previous_capsuledataRdy = false;
            recvPos = 0;
            return SL_RESULT_INVALID_DATA;
        }
    }

    // need more data

    _is_previous_capsuledataRdy = false;
    return SL_RESULT_OPERATION_TIMEOUT;  // use timeout to indicate waiting for more data

}

sl_result UltraCapsuleProcessor::processIncoming(int recvSize, uint8_t *recvData, int &remainData, std::function<void(bool startSweep, const LidarData& point)> handler)
{


    //    sl_lidar_response_measurement_node_hq_t   local_scan[MAX_SCAN_NODES];
    //size_t                                   scan_count = 0;
    //    Result<nullptr_t>                        ans = SL_RESULT_OK;
    //    memset(local_scan, 0, sizeof(local_scan));

    sl_result ans = _waitUltraCapsuledNode(recvSize, recvData, remainData);

    if (ans != SL_RESULT_OK) {
        return ans;
    }

    count = maxCount;

    _ultraCapsuleToNormal(ultra_capsule_node, local_buf, count);

    //cout << count << " Ultra data in local_buf.  What to do with it?" << endl;

    for (int i = 0; i < count; i++) {
        LidarData data;
        data.angle = getAngle(local_buf[i]);
        data.distance = getDistanceQ2(local_buf[i])/4;
        data.quality = local_buf[i].quality;
        if (data.isOk() || (local_buf[i].flag & 0x01)) {
            handler(local_buf[i].flag & 0x01, data);
        }
    }

    return SL_RESULT_OK;
}

void UltraCapsuleProcessor::_ultraCapsuleToNormal(const sl_lidar_response_ultra_capsule_measurement_nodes_t & capsule, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount)
{
    nodeCount = 0;
    if (_is_previous_capsuledataRdy) {
        int diffAngle_q8;
        int currentStartAngle_q8 = ((capsule.start_angle_sync_q6 & 0x7FFF) << 2);
        int prevStartAngle_q8 = ((_cached_previous_ultracapsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

        diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
        if (prevStartAngle_q8 > currentStartAngle_q8) {
            diffAngle_q8 += (360 << 8);
        }

        int angleInc_q16 = (diffAngle_q8 << 3) / 3;
        int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for (size_t pos = 0; pos < _countof(_cached_previous_ultracapsuledata.ultra_cabins); ++pos) {
            int dist_q2[3];
            int angle_q6[3];
            int syncBit[3];


            uint32_t combined_x3 = _cached_previous_ultracapsuledata.ultra_cabins[pos].combined_x3;

            // unpack ...
            int dist_major = (combined_x3 & 0xFFF);

            // signed partical integer, using the magic shift here
            // DO NOT TOUCH

            int dist_predict1 = (((int)(combined_x3 << 10)) >> 22);
            int dist_predict2 = (((int)combined_x3) >> 22);

            int dist_major2;

            uint32_t scalelvl1, scalelvl2;

            // prefetch next ...
            if (pos == _countof(_cached_previous_ultracapsuledata.ultra_cabins) - 1) {
                dist_major2 = (capsule.ultra_cabins[0].combined_x3 & 0xFFF);
            }
            else {
                dist_major2 = (_cached_previous_ultracapsuledata.ultra_cabins[pos + 1].combined_x3 & 0xFFF);
            }

            // decode with the var bit scale ...
            dist_major = _varbitscale_decode(dist_major, scalelvl1);
            dist_major2 = _varbitscale_decode(dist_major2, scalelvl2);


            int dist_base1 = dist_major;
            int dist_base2 = dist_major2;

            if ((!dist_major) && dist_major2) {
                dist_base1 = dist_major2;
                scalelvl1 = scalelvl2;
            }


            dist_q2[0] = (dist_major << 2);
            if ((dist_predict1 == 0xFFFFFE00) || (dist_predict1 == 0x1FF)) {
                dist_q2[1] = 0;
            }
            else {
                dist_predict1 = (dist_predict1 << scalelvl1);
                dist_q2[1] = (dist_predict1 + dist_base1) << 2;

            }

            if ((dist_predict2 == 0xFFFFFE00) || (dist_predict2 == 0x1FF)) {
                dist_q2[2] = 0;
            }
            else {
                dist_predict2 = (dist_predict2 << scalelvl2);
                dist_q2[2] = (dist_predict2 + dist_base2) << 2;
            }


            for (int cpos = 0; cpos < 3; ++cpos) {
                syncBit[cpos] = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16) ? 1 : 0;

                int offsetAngleMean_q16 = (int)(7.5 * 3.1415926535 * (1 << 16) / 180.0);

                if (dist_q2[cpos] >= (50 * 4))
                {
                    const int k1 = 98361;
                    const int k2 = int(k1 / dist_q2[cpos]);

                    offsetAngleMean_q16 = (int)(8 * 3.1415926535 * (1 << 16) / 180) - (k2 << 6) - (k2 * k2 * k2) / 98304;
                }

                angle_q6[cpos] = ((currentAngle_raw_q16 - int(offsetAngleMean_q16 * 180 / 3.14159265)) >> 10);
                currentAngle_raw_q16 += angleInc_q16;

                if (angle_q6[cpos] < 0) angle_q6[cpos] += (360 << 6);
                if (angle_q6[cpos] >= (360 << 6)) angle_q6[cpos] -= (360 << 6);

                sl_lidar_response_measurement_node_hq_t node;

                node.flag = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
                node.quality = dist_q2[cpos] ? (0x2F << SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
                node.angle_z_q14 = sl_u16((angle_q6[cpos] << 8) / 90);
                node.dist_mm_q2 = dist_q2[cpos];

                nodebuffer[nodeCount++] = node;
            }

        }
    }

    _cached_previous_ultracapsuledata = capsule;
    _is_previous_capsuledataRdy = true;
}

class CapsuleProcessor  : public LidarProcessor {


public:
    enum class CapsuleType {
        NORMAL_CAPSULE = 0,
        DENSE_CAPSULE = 1,
    };


private:
    constexpr static size_t maxCount{256};
    sl_lidar_response_measurement_node_hq_t  local_buf[maxCount];
    size_t                                   count = maxCount;

    int  recvPos = 0;
    sl_lidar_response_capsule_measurement_nodes_t       capsule_node;

    bool                                                _is_previous_capsuledataRdy{false};
    sl_lidar_response_capsule_measurement_nodes_t       _cached_previous_capsuledata;
    sl_lidar_response_dense_capsule_measurement_nodes_t _cached_previous_dense_capsuledata;
    bool                                                _scan_node_synced{false};
    bool _isScanning{false};
    CapsuleType _cached_capsule_flag{0};

    void _capsuleToNormal(const sl_lidar_response_capsule_measurement_nodes_t & capsule, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount);
    void _dense_capsuleToNormal(const sl_lidar_response_capsule_measurement_nodes_t & capsule, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount);

    sl_result _waitCapsuledNode(int recvSize, uint8_t* recvData, int& remainData);

public:
    CapsuleProcessor(CapsuleType capsuleFlag) { _cached_capsule_flag = capsuleFlag; }
    sl_result processIncoming(int recvSize, uint8_t* recvData, int& remainData, std::function<void(bool startSweep, const LidarData& point)> handler);
};


sl_result CapsuleProcessor::processIncoming(int recvSize, uint8_t *recvData, int &remainData, std::function<void(bool startSweep, const LidarData& point)> handler)
{
    sl_result ans = _waitCapsuledNode(recvSize, recvData, remainData); // // always discard the first data since it may be incomplete

    if (ans != SL_RESULT_OK) {
        count = 0;
        return ans;
    }

    count = maxCount;

    switch (_cached_capsule_flag) {
    case CapsuleType::NORMAL_CAPSULE:
        _capsuleToNormal(capsule_node, local_buf, count);
        break;
    case CapsuleType::DENSE_CAPSULE:
        _dense_capsuleToNormal(capsule_node, local_buf, count);
        break;
    }

    //cout << count << " Capule: data in local_buf.  What to do with it?" << endl;

    for (int i = 0; i < count; i++) {
        LidarData data;
        data.angle = getAngle(local_buf[i]);
        data.distance = getDistanceQ2(local_buf[i])/4;
        data.quality = local_buf[i].quality;
        if (data.isOk() || (local_buf[i].flag & 0x01)) {
            handler(local_buf[i].flag & 0x01, data);
        }
    }

    return SL_RESULT_OK;
}

sl_result CapsuleProcessor::_waitCapsuledNode(int recvSize, uint8_t *recvData, int& remainData)
{
    uint8_t *nodeBuffer = (uint8_t*)&capsule_node;

    size_t remainSize = sizeof(sl_lidar_response_capsule_measurement_nodes_t) - recvPos;

    if (recvSize > remainSize) {
        remainData = recvSize-remainSize;
        recvSize = remainSize;
    }
    else {
        remainData = 0;
    }

    for (size_t pos = 0; pos < recvSize; ++pos) {
        uint8_t currentByte = recvData[pos];

        switch (recvPos) {
        case 0: // expect the sync bit 1
        {
            uint8_t tmp = (currentByte >> 4);
            if (tmp == SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_1) {
                // pass
            }
            else {
                _is_previous_capsuledataRdy = false;
                continue;
            }

        }
            break;
        case 1: // expect the sync bit 2
        {
            uint8_t tmp = (currentByte >> 4);
            if (tmp == SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
                // pass
            }
            else {
                recvPos = 0;
                _is_previous_capsuledataRdy = false;
                continue;
            }
        }
            break;
        }

        nodeBuffer[recvPos++] = currentByte;

        if (recvPos == sizeof(sl_lidar_response_capsule_measurement_nodes_t)) {
            // calc the checksum ...
            uint8_t checksum = 0;
            uint8_t recvChecksum = ((capsule_node.s_checksum_1 & 0xF) | (capsule_node.s_checksum_2 << 4));
            for (size_t cpos = offsetof(sl_lidar_response_capsule_measurement_nodes_t, start_angle_sync_q6);
                 cpos < sizeof(sl_lidar_response_capsule_measurement_nodes_t); ++cpos)
            {
                checksum ^= nodeBuffer[cpos];
            }
            if (recvChecksum == checksum) {
                // only consider vaild if the checksum matches...
                if (capsule_node.start_angle_sync_q6 & SL_LIDAR_RESP_MEASUREMENT_EXP_SYNCBIT) {
                    // this is the first capsule frame in logic, discard the previous cached data...
                    _scan_node_synced = false;
                    _is_previous_capsuledataRdy = false;
                }
                recvPos = 0;
                return SL_RESULT_OK;
            }
            // bad checksum
            _is_previous_capsuledataRdy = false;
            recvPos = 0;
            return SL_RESULT_INVALID_DATA;
        }
    }

    // need more data

    _is_previous_capsuledataRdy = false;
    return SL_RESULT_OPERATION_TIMEOUT;  // use timeout to indicate waiting for more data
}

void CapsuleProcessor::_capsuleToNormal(const sl_lidar_response_capsule_measurement_nodes_t & capsule, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount)
{
    nodeCount = 0;
    if (_is_previous_capsuledataRdy) {
        int diffAngle_q8;
        int currentStartAngle_q8 = ((capsule.start_angle_sync_q6 & 0x7FFF) << 2);
        int prevStartAngle_q8 = ((_cached_previous_capsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

        diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
        if (prevStartAngle_q8 > currentStartAngle_q8) {
            diffAngle_q8 += (360 << 8);
        }

        int angleInc_q16 = (diffAngle_q8 << 3);
        int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for (size_t pos = 0; pos < _countof(_cached_previous_capsuledata.cabins); ++pos) {
            int dist_q2[2];
            int angle_q6[2];
            int syncBit[2];

            dist_q2[0] = (_cached_previous_capsuledata.cabins[pos].distance_angle_1 & 0xFFFC);
            dist_q2[1] = (_cached_previous_capsuledata.cabins[pos].distance_angle_2 & 0xFFFC);

            int angle_offset1_q3 = ((_cached_previous_capsuledata.cabins[pos].offset_angles_q3 & 0xF) | ((_cached_previous_capsuledata.cabins[pos].distance_angle_1 & 0x3) << 4));
            int angle_offset2_q3 = ((_cached_previous_capsuledata.cabins[pos].offset_angles_q3 >> 4) | ((_cached_previous_capsuledata.cabins[pos].distance_angle_2 & 0x3) << 4));

            angle_q6[0] = ((currentAngle_raw_q16 - (angle_offset1_q3 << 13)) >> 10);
            syncBit[0] = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16) ? 1 : 0;
            currentAngle_raw_q16 += angleInc_q16;


            angle_q6[1] = ((currentAngle_raw_q16 - (angle_offset2_q3 << 13)) >> 10);
            syncBit[1] = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16) ? 1 : 0;
            currentAngle_raw_q16 += angleInc_q16;

            for (int cpos = 0; cpos < 2; ++cpos) {

                if (angle_q6[cpos] < 0) angle_q6[cpos] += (360 << 6);
                if (angle_q6[cpos] >= (360 << 6)) angle_q6[cpos] -= (360 << 6);

                sl_lidar_response_measurement_node_hq_t node;

                node.angle_z_q14 = sl_u16((angle_q6[cpos] << 8) / 90);
                node.flag = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
                node.quality = dist_q2[cpos] ? (0x2f << SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
                node.dist_mm_q2 = dist_q2[cpos];

                nodebuffer[nodeCount++] = node;
            }

        }
    }

    _cached_previous_capsuledata = capsule;
    _is_previous_capsuledataRdy = true;
}

void CapsuleProcessor::_dense_capsuleToNormal(const sl_lidar_response_capsule_measurement_nodes_t & capsule, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount)
{
    static int lastNodeSyncBit = 0;
    const sl_lidar_response_dense_capsule_measurement_nodes_t *dense_capsule = reinterpret_cast<const sl_lidar_response_dense_capsule_measurement_nodes_t*>(&capsule);
    nodeCount = 0;
    if (_is_previous_capsuledataRdy) {
        int diffAngle_q8;
        int currentStartAngle_q8 = ((dense_capsule->start_angle_sync_q6 & 0x7FFF) << 2);
        int prevStartAngle_q8 = ((_cached_previous_dense_capsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

        diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
        if (prevStartAngle_q8 > currentStartAngle_q8) {
            diffAngle_q8 += (360 << 8);
        }

        int angleInc_q16 = (diffAngle_q8 << 8) / 40;
        int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for (size_t pos = 0; pos < _countof(_cached_previous_dense_capsuledata.cabins); ++pos) {
            int dist_q2;
            int angle_q6;
            int syncBit;
            const int dist = static_cast<const int>(_cached_previous_dense_capsuledata.cabins[pos].distance);
            dist_q2 = dist << 2;
            angle_q6 = (currentAngle_raw_q16 >> 10);

            syncBit = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < (angleInc_q16<<1)) ? 1 : 0;
            syncBit = (syncBit^ lastNodeSyncBit)&syncBit;//Ensure that syncBit is exactly detected
            if (syncBit) {
                _scan_node_synced = true;
            }

            currentAngle_raw_q16 += angleInc_q16;

            if (angle_q6 < 0) angle_q6 += (360 << 6);
            if (angle_q6 >= (360 << 6)) angle_q6 -= (360 << 6);


            sl_lidar_response_measurement_node_hq_t node;

            node.angle_z_q14 = sl_u16((angle_q6 << 8) / 90);
            node.flag = (syncBit | ((!syncBit) << 1));
            node.quality = dist_q2 ? (0x2f << SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
            node.dist_mm_q2 = dist_q2;
            if(_scan_node_synced)
                nodebuffer[nodeCount++] = node;
            lastNodeSyncBit = syncBit;
        }
    }
    else {
        _scan_node_synced = false;
    }

    _cached_previous_dense_capsuledata = *dense_capsule;
    _is_previous_capsuledataRdy = true;
}

sl_result LidarDataProcessor::init(uint8_t scanAnsType, uint32_t header_size)
{
    if (scanAnsType == SL_LIDAR_ANS_TYPE_MEASUREMENT) {
        if (header_size < sizeof(sl_lidar_response_measurement_node_t)) {
            return SL_RESULT_INVALID_DATA;
        }
        proc = make_unique<StdProcessor>();
        return SL_RESULT_OK;
    }
    else if (scanAnsType == SL_LIDAR_ANS_TYPE_MEASUREMENT_CAPSULED) {
        if (header_size < sizeof(sl_lidar_response_capsule_measurement_nodes_t)) {
            return SL_RESULT_INVALID_DATA;
        }
        proc = make_unique<CapsuleProcessor>(CapsuleProcessor::CapsuleType::NORMAL_CAPSULE);
        return SL_RESULT_OK;
    }
    else if (scanAnsType == SL_LIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED) {
        if (header_size < sizeof(sl_lidar_response_capsule_measurement_nodes_t)) {
            return SL_RESULT_INVALID_DATA;
        }
        proc = make_unique<CapsuleProcessor>(CapsuleProcessor::CapsuleType::DENSE_CAPSULE);
        return SL_RESULT_OK;
    }
    else if (scanAnsType == SL_LIDAR_ANS_TYPE_MEASUREMENT_HQ) {
        if (header_size < sizeof(sl_lidar_response_hq_capsule_measurement_nodes_t)) {
            return SL_RESULT_INVALID_DATA;
        }
        proc = make_unique<HQDataProcessor>();
        return SL_RESULT_OK;
    }
    else {
        if (header_size < sizeof(sl_lidar_response_ultra_capsule_measurement_nodes_t)) {
            return SL_RESULT_INVALID_DATA;
        }
        proc = make_unique<UltraCapsuleProcessor>();
        return SL_RESULT_OK;
    }
}

sl_result LidarDataProcessor::processIncoming(int recvSize, uint8_t *recvData, int &remainData, std::function<void(bool startSweep, const LidarData& point)> handler)
{
    if (!proc) {
        // discard data
        remainData = 0;
        return SL_RESULT_INVALID_DATA;
    }
    return proc->processIncoming(recvSize, recvData, remainData, handler);
}
