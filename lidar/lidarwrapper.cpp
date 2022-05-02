#include "lidarwrapper.h"
#include <iostream>
using namespace sl;
using namespace std;
using namespace mssm; //oh yeah codependency moment

LidarWrapper::LidarWrapper()
{

}

LidarWrapper::~LidarWrapper()
{
    delete lidar;
    lidar = NULL;
}

void LidarWrapper::setMotorSpeed(int speed)
{
    if(speed != -1)
    {
        atDefaultSpeed = false;
        lidar->setMotorSpeed(speed);
    }
    else{
        if(!atDefaultSpeed){
            atDefaultSpeed = true;
            lidar->setMotorSpeed();
        }
    }
}

bool LidarWrapper::checkLidarHealth(sl::ILidarDriver *drv)
{
    sl_result op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preferred way to judge whether the operation succeeded.
        printf("Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stdout, "Error, lidar internal error detected. Rebooting.\n");
            drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

int LidarWrapper::scan()
{
    sl_result op_result;
    if(SL_IS_OK(res)){ 
        setMotorSpeed();
        lidar->startScan(0,1);
        //originally a while true here
        nodeCount = _countof(nodes);
        op_result = lidar->grabScanDataHq(nodes, nodeCount); //did operation succeed? //use a promise?

        if (SL_IS_OK(op_result)) {
//            lidar->ascendScanData(nodes, nodeCount);
            updatePoints();
            return 1;
        }
    }
    else{
        fprintf(stdout, "Failed to connect to LIDAR %08x\r\n", res);
    }

    return 0;
}


double mmToInch(double dist)
{
    return dist/25.4;
}

void LidarWrapper::updatePoints()
{
    points.clear();
    polarData.clear();
    for(int i = 0; i < nodeCount; i++){
        if(nodes[i].quality != 0)
        {
            double dist = mmToInch(nodes[i].dist_mm_q2);
            double angle = nvgDegToRad((nodes[i].angle_z_q14* 90.f) / 16384.f);
            polarData.push_back({dist,angle});
            Vec2d point = {dist,0};
            point.rotate(angle);
            points.push_back(point);
        }
    }
}


