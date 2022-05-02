#include "lidarwrapper.h"

using namespace sl;
using namespace std;
using namespace mssm; //oh yeah codependency moment

LidarWrapper::LidarWrapper()
{

}

void LidarWrapper::setMotorSpeed(int speed)
{
    if(speed != -1)
    {
        lidar->setMotorSpeed(speed);
    }
    else{
        lidar->setMotorSpeed();
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
    fprintf(stdout, "Scanning", res);
    sl_result op_result;
    if(SL_IS_OK(res)){
        lidar->setMotorSpeed();
        // start scan...
        lidar->startScan(0,1);
        //originally a while true here
            size_t count = _countof(nodes);
            op_result = lidar->grabScanDataHq(nodes, count); //did operation succeed?

            if (SL_IS_OK(op_result)) {
                lidar->ascendScanData(nodes, count);
                for (int pos = 0; pos < (int)count ; ++pos) {
                    if(nodes[pos].quality != 0){
                    printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
                           (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ",
                           (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
                           nodes[pos].dist_mm_q2/4.0f,
                           nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
                    }
                }
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
    points = {};
    polarData = {};
    for(auto node: nodes){
        if(node.quality != 0)
        {
            double dist = mmToInch(node.dist_mm_q2);
            double angle = nvgDegToRad(node.angle_z_q14);
            polarData.push_back({dist,angle});
            Vec2d point = {dist,0};
            point = point.rotated(angle);
        }
    }
}
