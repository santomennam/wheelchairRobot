#include "graphics.h"
#include "lidar.h"
#include <algorithm>
#include <iostream>
#include <iterator>
#include <cctype>
#include <iomanip>
#include <array>
#include "bitrange.h"

constexpr bool verbose = true;

using namespace std;
using namespace mssm;

constexpr int      RESET_TIMEOUT = 1000;
constexpr uint64_t COMM_IDLE_THRESHOLD = 75;
constexpr int      RESPONSE_TIMEOUT = 100;
constexpr int      SEND_TIMEOUT = 100;

constexpr uint8_t RPLIDAR_CMD_SYNC_BYTE = 0xA5;
constexpr uint8_t RPLIDAR_ANS_SYNC_BYTE1 = 0xA5;
constexpr uint8_t RPLIDAR_ANS_SYNC_BYTE2 = 0x5A;

//constexpr uint8_t RPLIDAR_CMDFLAG_HAS_PAYLOAD = 0x80;
//constexpr uint8_t RPLIDAR_ANS_PKTFLAG_LOOP = 0x1;

// response types
constexpr uint8_t RPLIDAR_RESP_INFO       = 0x04;
constexpr uint8_t RPLIDAR_RESP_HEALTH     = 0x06;
constexpr uint8_t RPLIDAR_RESP_SAMPLERATE = 0x15;


constexpr uint8_t RPLIDAR_CMD_STOP        = 0x25;
constexpr uint8_t RPLIDAR_CMD_SCAN        = 0x20;
constexpr uint8_t RPLIDAR_CMD_FORCE_SCAN  = 0x21;
constexpr uint8_t RPLIDAR_CMD_RESET       = 0x40;


// Commands without payload but have response
constexpr uint8_t RPLIDAR_CMD_GET_DEVICE_INFO     = 0x50;
constexpr uint8_t RPLIDAR_CMD_GET_DEVICE_HEALTH   = 0x52;
constexpr uint8_t RPLIDAR_CMD_GET_SAMPLERATE      = 0x59; //added in fw 1.17

constexpr uint8_t RPLIDAR_CMD_HQ_MOTOR_SPEED_CTRL = 0xA8;

// Commands with payload and have response
constexpr uint8_t RPLIDAR_CMD_EXPRESS_SCAN   = 0x82; //added in fw 1.17
constexpr uint8_t RPLIDAR_CMD_HQ_SCAN        = 0x83; //added in fw 1.24
constexpr uint8_t RPLIDAR_CMD_GET_LIDAR_CONF = 0x84; //added in fw 1.24
constexpr uint8_t RPLIDAR_CMD_SET_LIDAR_CONF = 0x85; //added in fw 1.24

//add for A2 to set RPLIDAR motor pwm when using accessory board
constexpr uint8_t RPLIDAR_CMD_SET_MOTOR_PWM      = 0xF0;
constexpr uint8_t RPLIDAR_CMD_GET_ACC_BOARD_FLAG = 0xFF;


#define RPLIDAR_ANS_HEADER_SIZE_MASK        0x3FFFFFFF
#define RPLIDAR_ANS_HEADER_SUBTYPE_SHIFT    (30)

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

        descriptor.size = static_cast<uint32_t>(*_First++);           // byte 3
        descriptor.size |= static_cast<uint32_t>(*_First++) << 8;     // byte 4
        descriptor.size |= static_cast<uint32_t>(*_First++) << 16;    // byte 5

        uint32_t tmp = static_cast<uint32_t>(*_First++);   // byte 6

        descriptor.size |= (tmp & 0x03) << 24;

        descriptor.type = static_cast<uint8_t>(*_First++);            // byte 7

        descriptor.mode = static_cast<uint8_t>(tmp >> 6);

        availableDataSize -= state.descriptorSize;

        state.needData = descriptor.mode == 0; // mode 0 is Single Response mode, where the descriptor should be followed by exactly one data packet
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

template<class _RanIt>
bool rplidarParseScan(_RanIt& _First, const _RanIt _Last, int& quality, double& angle, double& distance, bool& hasMoreData)
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
        // new scan
        if (verbose) cout << "New Scan: ";
        hasMoreData = true;
        break;
    case 2:
        // continued scan
        if (verbose) cout << "Continued Scan: ";
        hasMoreData = false;
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

Lidar::Lidar(const std::string& portName, std::function<void (const std::vector<LidarData> &)> handler)
    : handler{handler}
{
    port.open(portName, 115200);
    //setState(LidarMode::connect);
    lastTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

Lidar::~Lidar()
{
    send(RPLIDAR_CMD_SET_MOTOR_PWM, rawString(0), false);
    send(RPLIDAR_CMD_STOP,"",false);
}


void Lidar::update()
{
    auto currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    if (expectSent && !port.stillWriting()) {
        if (verbose) cout << "Serial port reports cmd sent" << endl;
        commIdleTime = 0;
        updateState(ModeUpdate::cmdSent);
    }

    if (port.canRead()) {
        commIdleTime = 0;
        string data = port.read();
        //        if (verbose) cout << "Parsing " << readData.length() << " bytes" << endl;
        //displayHex(data);
        parse(data);
        //        readData.clear();
    }

    updateTime(currentTime);

    if (activeCommand.id == LidarCommandId::none) {
        if (commands.size() > 0) {
            activeCommand = commands[0];
            commands.erase(commands.begin()); // TODO inefficient!  Use queue
            switch (activeCommand.id) {
            case LidarCommandId::motorSpeed:
                send(RPLIDAR_CMD_SET_MOTOR_PWM, rawString(static_cast<uint16_t>(activeCommand.arg1)), false);
                break;
            case LidarCommandId::reset:
                if (isScanning) {
                    cout << "Cannot reset while scanning!  (Stopping first)" << endl;
                    activeCommand.id = LidarCommandId::none;

                    LidarCommand resetCmd{LidarCommandId::reset, 0, RESET_TIMEOUT};
                    LidarCommand stopCmd{LidarCommandId::endScan, 0, RESPONSE_TIMEOUT};

                    commands.insert(commands.begin(), resetCmd);
                    commands.insert(commands.begin(), stopCmd);
                }
                else {
                    send(RPLIDAR_CMD_RESET, "", true);
                }
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
                send(RPLIDAR_CMD_EXPRESS_SCAN, string(5, '\0'), true);
                break;
            case LidarCommandId::none:
                cout << "Shouldn't really happen" << endl;
                break;
            }
        }
    }
}

void Lidar::updateTime(std::chrono::milliseconds::rep currentTime)
{
    int ms = static_cast<int>(currentTime - lastTime);
    if (ms < 0) {
        ms = 0;
    }
    bool commIdle = commIdleTime > COMM_IDLE_THRESHOLD;
    commIdleTime += static_cast<uint64_t>(ms);
    if (!commIdle && (commIdleTime > COMM_IDLE_THRESHOLD))
    {
        if (verbose) cout << "Comm has gone idle" << endl;
        updateState(ModeUpdate::commIdle);
    }

    lastTime = currentTime;
    // cout << "MS: " << ms << endl;
    if (timeoutTime > 0) {
        if (verbose) cout << "TO: " << timeoutTime << endl;
        timeoutTime -= ms;
        if (timeoutTime <= 0) {
            timeoutTime = 0;
            if (verbose) cout << "updateTime sending timeout" << endl;
            updateState(ModeUpdate::timeout);
        }
    }
}

void Lidar::updateState(ModeUpdate reason)
{
    ModeTransition transition;

    switch (reason) {
    case ModeUpdate::commIdle:
        //        if (isScanning) {
        switch (activeCommand.id) {
        case LidarCommandId::endScan:
            //            case LidarCommandId::reset:
            isScanning = false;
            responseParser.reset();
            if (verbose) cout << "Scanning is really done" << endl;
            timeoutTime = 0;
            transition = ModeTransition::nextState;
            break;
        default:
            // cout << "IS THIS OK???" << endl;
            transition = ModeTransition::skip;
            break;
        }
        //        }
        //        else {
        //            transition = ModeTransition::skip;
        //        }
        break;
    case ModeUpdate::timeout:
        if (expectSent) {
            cout << "Timeout waiting for cmd sent" << endl;
            transition = ModeTransition::error;
        }
        else if (expectResponse) {
            cout << "Timeout waiting for cmd response " << lastTime << endl;
            transition = ModeTransition::error;
        }
        else {
            cout << "Timeout while in unknown state" << endl;
            transition = ModeTransition::error;
        }
        break;
    case ModeUpdate::cmdSent:
        if (expectSent) {
            if (verbose) cout << "We did expect cmdSent" << endl;
            expectSent = false;
            if (expectResponse) {
                // now wait for a response
                if (verbose) cout << "Now we should wait for a response" << endl;
                timeoutTime = activeCommand.responseTimeout;
                transition = ModeTransition::skip;
            }
            else {
                timeoutTime = activeCommand.responseTimeout;
                if (timeoutTime == 0) {
                    transition = ModeTransition::nextState;
                }
                else {
                    transition = ModeTransition::skip;
                }
            }
        }
        else {
            cout << "Unexpected cmdSent" << endl;
            transition = ModeTransition::error;
        }
        break;
    case ModeUpdate::gotResponse:
        if (expectSent) {
            cout << "Unexpected gotResponse waiting for sent" << endl;
            transition = ModeTransition::error;
        }
        else if (expectResponse) {
            expectResponse = false;
            timeoutTime = 0;
            transition = ModeTransition::nextState;
        }
        else {
            cout << "Unexpected gotResponse" << endl;
            transition = ModeTransition::error;
        }
        break;
    }

    switch (transition) {
    case ModeTransition::skip:
        if (verbose) cout << "Transition: skip" << endl;
        break;
    case ModeTransition::error:
        cout << "Transition: error" << endl;
        activeCommand.id = LidarCommandId::none;
        break;
    case ModeTransition::nextState:
        if (verbose) cout << "Command complete" << endl;
        processResponse(responseParser.descriptor, responseParser.data);
        activeCommand.id = LidarCommandId::none;
        break;
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
    }
}

bool Lidar::send(uint8_t cmd, const std::string& data, bool hasResponse)
{
    cout << "Sending CMD: " << hex << static_cast<int>(cmd) << dec << endl;
    string dataPacket = rplidarCommand(cmd, data);
    port.write(dataPacket);
    //serialPort.flush();
    timeoutTime = SEND_TIMEOUT;
    expectResponse = hasResponse;
    expectSent = true;
    return true;
}


//std::array<ExpressPoint,32> packet1;
//std::array<ExpressPoint,32> packet2;
//bool parsePacket1{true};

double angleDiff(double a, double a1)
{
    if (a <= a1) {
        return a1-a;
    }
    else {
        return 360.0+a1-a;
    }
}

template <typename I>
bool ExpressScanProcessor::parse(I& start, const I end, std::function<void(const std::vector<LidarData>& values)> transmit)
{
    size_t prevPacketIdx = nextPacketIdx ? 0 : 1;
    size_t currPacketIdx = nextPacketIdx;
    nextPacketIdx = prevPacketIdx;

    if (!rplidarParseExpressScan(start, end, startAngle[currPacketIdx], isNew[currPacketIdx], packet[currPacketIdx])) {
        cout << "Error scanning: flush data" << endl;
        start = end;
        packetValid[currPacketIdx] = false;
        return false;
    }

    packetValid[currPacketIdx] = true;

    if (isNew[currPacketIdx]) {
        cout << "Can't process new packet" << endl;
        return true;
    }

    bool prevPacketValid = packetValid[prevPacketIdx];

    if (!prevPacketValid) {
        cout << "Prev Packet Invalid???   can't process this packet";
        return true;
    }

    std::array<ExpressPoint,32>& prevPacket = packet[prevPacketIdx];
    //std::array<ExpressPoint,32>& currPacket = packet[currPacketIdx];

    double prevStartAngle = startAngle[prevPacketIdx];
    double currStartAngle = startAngle[currPacketIdx];

    double da = angleDiff(prevStartAngle, currStartAngle);

    //  cout << "DA: " << da << endl;

    //  cout << "Angle: " << prevStartAngle << "           "  << currStartAngle << endl;


    for (size_t i = 0; i < 32; i++) {
        // cout << prevPacket[i].deltaAngle() << endl;

        double angle = prevStartAngle + (da/32.0)*i; // - prevPacket[i].deltaAngle();
        if (angle > 360.0) {
            angle -= 360.0;
        }
        double distance = prevPacket[i].distance;
        int quality = distance > 0 ? 1 : 0;
        if (!data.empty() && data.back().angle > 350 && angle < 10) {
            transmit(data);
            data.clear();
        }
        data.push_back(LidarData{quality, angle, distance});
        //        if (prevPacket[i].distance != 0) {
        //            cout << angle << " " << prevPacket[i].distance << endl;
        //        }
    }

    return true;
}


void Lidar::parse(const std::string &str)
{
    if (bufferedData.size() > 0) {
        cout << "Appending to existing data" << endl;
    }
    bufferedData.append(str);

    if (activeCommand.id == LidarCommandId::reset) {
        //sendEvent(0, 0, 0, bufferedData);
        bufferedData.clear();
        updateState(ModeUpdate::gotResponse);
        return;
    }

    if (expectResponse) {
        auto i = bufferedData.begin();
        switch (responseParser.parse(i, bufferedData.end())) {
        case ResponseParser::ParseResult::incomplete:
            cout << "Need a bit more data" << endl;
            break;
        case ResponseParser::ParseResult::complete:
            updateState(ModeUpdate::gotResponse);
            isScanning = responseParser.descriptor.mode == 0x01; // multiple response mode
            break;
        case ResponseParser::ParseResult::error:
            cout << "Parse Failed!" << endl;
            break;
        }
        bufferedData.erase(bufferedData.begin(), i); // erase used data from buffer
        return;
    }

    while (!bufferedData.empty() && bufferedData.size() >= responseParser.descriptor.size) {

        auto i = bufferedData.begin();

        switch (responseParser.descriptor.size) {
        case 5:
            int quality;
            double angle;
            double distance;
            bool moreData;
            //cout << "Parsing: " << endl;

            if (!rplidarParseScan(i, bufferedData.end(), quality, angle, distance, moreData)) {
                cout << "Parse Failed" << endl;
            }
            else {
                cout << "FIX THIS" << endl;
                //                data.push_back(LidarData{quality, angle, distance});
            }
            break;
        case 84:
            if (!expressScanProcessor.parse(i, bufferedData.end(), [this](std::vector<LidarData> data) { transmit(data); }))
            {
                cout << "Error scanning: flush data" << endl;
                i = bufferedData.end();
            }
            break;
        default:
            cout << "Unexpected response size: " << responseParser.descriptor.size << endl;
            break; // what to do... what to do...

        }

        bufferedData.erase(bufferedData.begin(), i);  // get rid of what we used
    }

    if (bufferedData.size() > 0) {
        if (verbose) cout << "Leaving " << bufferedData.size() << " in buffer... hope we get more data" << endl;
    }
}

void Lidar::transmit(const std::vector<LidarData>& data)
{
    handler(data);
}


void Lidar::cmdMotorSpeed(int speed) {
    commands.push_back(LidarCommand{LidarCommandId::motorSpeed, speed, 0});
}

void Lidar::cmdReset()
{
    //if (verbose) cout << "Queueing reset Command" << endl;
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

void Lidar::cmdBeginExpressScan()
{
    commands.push_back(LidarCommand{LidarCommandId::beginExpressScan, 0, RESPONSE_TIMEOUT});
}



//[distance_sync flags]
#define RPLIDAR_RESP_MEASUREMENT_EXP_ANGLE_MASK           (0x3)
#define RPLIDAR_RESP_MEASUREMENT_EXP_DISTANCE_MASK        (0xFC)

typedef struct _rplidar_response_cabin_nodes_t {
    uint16_t   distance_angle_1; // see [distance_sync flags]
    uint16_t   distance_angle_2; // see [distance_sync flags]
    uint8_t    offset_angles_q3;
}  rplidar_response_cabin_nodes_t;


#define RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1               0xA
#define RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2               0x5

#define RPLIDAR_RESP_MEASUREMENT_HQ_SYNC                  0xA5

#define RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT              (0x1<<15)

typedef struct _rplidar_response_capsule_measurement_nodes_t {
    uint8_t                             s_checksum_1; // see [s_checksum_1]
    uint8_t                             s_checksum_2; // see [s_checksum_1]
    uint16_t                            start_angle_sync_q6;
    rplidar_response_cabin_nodes_t  cabins[16];
}  rplidar_response_capsule_measurement_nodes_t;
// ext1 : x2 boost mode

#define RPLIDAR_RESP_MEASUREMENT_EXP_ULTRA_MAJOR_BITS     12
#define RPLIDAR_RESP_MEASUREMENT_EXP_ULTRA_PREDICT_BITS   10

typedef struct _rplidar_response_ultra_cabin_nodes_t {
    // 31                                              0
    // | predict2 10bit | predict1 10bit | major 12bit |
    uint32_t combined_x3;
}  rplidar_response_ultra_cabin_nodes_t;

typedef struct _rplidar_response_ultra_capsule_measurement_nodes_t {
    uint8_t                             s_checksum_1; // see [s_checksum_1]
    uint8_t                             s_checksum_2; // see [s_checksum_1]
    uint16_t                            start_angle_sync_q6;
    rplidar_response_ultra_cabin_nodes_t  ultra_cabins[32];
}  rplidar_response_ultra_capsule_measurement_nodes_t;

typedef struct rplidar_response_measurement_node_hq_t {
    uint16_t   angle_z_q14;
    uint32_t   dist_mm_q2;
    uint8_t    quality;
    uint8_t    flag;
}  rplidar_response_measurement_node_hq_t;

typedef struct _rplidar_response_hq_capsule_measurement_nodes_t{
    uint8_t sync_byte;
    uint64_t time_stamp;
    rplidar_response_measurement_node_hq_t node_hq[16];
    uint32_t  crc32;
} rplidar_response_hq_capsule_measurement_nodes_t;
