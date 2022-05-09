#ifndef LIDAR_H
#define LIDAR_H

#include "serialport.h"
#include <vector>
#include <array>
#include <functional>
#include <chrono>
#include <memory>

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

    std::string respDataName;
private:
    struct ParseState {
        int  descriptorSize;
        bool needDescriptor;
        bool needData;
        bool hasDescriptor{false};
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
    bool hasDescriptor() const { return state.hasDescriptor; }

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
    uint8_t angleComp; // q3.2 format in 1/4ths of a degree, high bit of 6 bit number is sign.  +- 32/8
    uint16_t distance;

    double deltaAngle() { return (angleComp & 0x20) ? (-static_cast<double>(angleComp & 0x1F)/4.0) : (static_cast<double>(angleComp & 0x1F)/4.0); }
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
    double lastAngle{360};
public:
    template <typename I>
    bool parse(I& start, const I end, std::function<void(bool startScan, const LidarData& point)> handler);
};

typedef uint32_t sl_result;

class LidarProcessor {
public:
    virtual sl_result processIncoming(int recvSize, uint8_t* recvData, int& remainData, std::function<void(bool startSweep, const LidarData& point)> handler) = 0;
    void dispatch(double angle, double distance, int quality);
};

class LidarDataProcessor {
private:
    std::unique_ptr<LidarProcessor> proc;
public:
    sl_result init(uint8_t scanAnsType, uint32_t header_size);
    sl_result processIncoming(int recvSize, uint8_t* recvData, int& remainData, std::function<void(bool startSweep, const LidarData& point)> handler);

};

class Lidar
{
    SerialPort port;

    LidarDataProcessor processor;

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
    void cmdBeginExpressScan(int expressMode);
    void cmdReqConfModeCount();
    void cmdReqConfUsPerSample(int mode);
    void cmdReqConfAnsType(int mode);
    void cmdReqConfName(int mode);
    void cmdReqTypicalMode();
};



#endif // LIDAR_H
