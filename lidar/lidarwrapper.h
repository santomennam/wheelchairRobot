#ifndef LIDARWRAPPER_H
#define LIDARWRAPPER_H
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>

#include "nanovg.h"
#include "serialport.h"
#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include "graphics.h"
#include <rplidar.h>

#ifndef _MSC_VER
#pragma GCC diagnostic ignored "-Wsign-compare"
#endif
class LidarWrapper
{
public:
    LidarWrapper();
    ~LidarWrapper();
    void setMotorSpeed(int speed = -1);
    bool checkLidarHealth(sl::ILidarDriver * drv);
    int scan();
    std::vector<Vec2d> points;
    std::vector<Vec2d> polarData;

public:
    sl::IChannel* _channel;
    sl::Result<sl::IChannel*> channel = sl::createSerialPortChannel("COM8", 115200);
    sl::ILidarDriver * lidar = *sl::createLidarDriver();
    sl_result res = (lidar)->connect(*channel);
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    void updatePoints();
};

#endif // LIDARWRAPPER_H
