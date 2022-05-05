#ifndef LIDAR_H
#define LIDAR_H

#include "serialport.h"
#include <vector>
#include <array>
#include <functional>
#include <chrono>

class ResponseParser {
public:

    struct Descriptor {
        uint8_t  type;
        uint8_t  mode;
        uint32_t size;
    };

    struct RespDeviceInfo {
        uint8_t model;
        uint8_t firmwareMinor;
        uint8_t firmwareMajor;
        uint8_t hardware;
        uint8_t serialNumber[16];
    };

    struct RespDeviceHealth {
        uint8_t status;
        uint16_t errorCode;
    };

    struct RespSampleRate {
        uint16_t standard;
        uint16_t express;
    };

    struct RespConfigInfo {
        uint32_t type;
        uint32_t data1;
    };

    //UNION!
    union RespData {
        RespDeviceInfo devInfo;
        RespDeviceHealth devHealth;
        RespSampleRate sampleRate;
        RespConfigInfo configInfo;
    };

private:
    struct ParseState {
        int  descriptorSize;
        bool needDescriptor;
        bool needData;
    };

    ParseState state;
public:
    Descriptor descriptor;
    RespData   data;

    enum class ParseResult {
        incomplete,
        complete,
        error
    };

public:
    ResponseParser();

    void reset();

    template <typename I>
    ParseResult parse(I& start, const I end);

private:

    int dataPacketSize();

    template <typename I> ParseResult parseDataInfo(I& start);
    template <typename I> ParseResult parseDataHealth(I& start);
    template <typename I> ParseResult parseDataSampleRate(I& start);
    template <typename I> ParseResult parseConfigResp(I& start);
};

struct ExpressPoint {
    uint8_t angleComp; // in 1/8ths of a degree, high bit of 6 bit number is sign.  +- 32/8
    uint16_t distance;

    double deltaAngle() { return static_cast<int8_t>(angleComp << 2)/32.0; } //  return (angleComp & 0x20) ? (-static_cast<double>(angleComp & 0x1F)/8.0) : (static_cast<double>(angleComp & 0x1F)/8.0); }
};

struct LidarData {
    int quality;
    double angle;
    double distance;
};

class ExpressScanProcessor {
private:
    std::array<ExpressPoint,32> cabin[2];
    double startAngle[2];
    bool isNew[2];
    bool packetValid[2];
    size_t nextCabinIdx{0};
    int cabinNumber{-1};
public:
    template <typename I>
    bool parse(I& start, const I end, std::function<void(bool startScan, const LidarData& point)> handler);
};

class Lidar
{
    SerialPort port;

    enum class LidarCommandId {
        none,
        reset,
        motorSpeed,
        beginScan,
        endScan,
        getInfo,
        getHealth,
        getSampleRate,
        beginExpressScan,
        getConfig
    };

    struct LidarCommand {
        LidarCommandId id;
        int arg1;
        int responseTimeout;
    };

private:

    enum class ModeUpdate {
        commIdle,
        timeout,
        cmdSent,
        gotResponse,
    };

    enum class ModeTransition {
        skip,
        nextState,
        error,
    };

private:

    ResponseParser responseParser;
    ExpressScanProcessor expressScanProcessor;

    std::vector<LidarCommand> commands;

    LidarCommand activeCommand{LidarCommandId::none, 0, 0};

    uint64_t commIdleTime{0};
    int  timeoutTime{0};

    bool expectSent{false};
    bool expectResponse{false};
    bool isScanning{false};
    bool inStartup{true};

    std::function<void(bool startSweep, const LidarData& point)> handler;

    std::chrono::milliseconds::rep lastTime;
    std::string bufferedData;
public:
    Lidar(const std::string& portName, std::function<void(bool startSweep, const LidarData& point)> handler);
    virtual ~Lidar();
public:
    void updateTime(std::chrono::milliseconds::rep currentTime);

    void update();
    bool send(uint8_t cmd, const std::string& data, bool hasResponse);

    void parse(const std::string& str);

    void updateState(ModeUpdate reason);

    void processResponse(const ResponseParser::Descriptor& desc, const ResponseParser::RespData& data);

    void cmdMotorSpeed(int speed);
    void cmdReset();
    void cmdBeginScan();
    void cmdEndScan();
    void cmdGetInfo();
    void cmdGetHealth();
    void cmdGetSampleRate();
    void cmdBeginExpressScan();
    void cmdReqConfModeCount();
    void cmdReqConfUsPerSample(int mode);
    void cmdReqConfAnsType(int mode);

};

#endif // LIDAR_H
