#include <iostream>
#include "graphics.h"
#include <iomanip>
#include "serialport.h"
#include "CmdLink.h"

using namespace std;
using namespace mssm;

#ifndef _MSC_VER
#pragma GCC diagnostic ignored "-Wsign-compare"
#endif

// Here are some of the basic drawing commands:

//void   line(Vec2d p1, Vec2d p2, Color c = WHITE);
//void   ellipse(Vec2d center, double w, double h, Color c = WHITE, Color f = TRANSPARENT);
//void   arc(Vec2d center, double w, double h, double a, double alen, Color c = WHITE);
//void   chord(Vec2d center, double w, double h, double a, double alen, Color c = WHITE, Color f = TRANSPARENT);
//void   pie(Vec2d center, double w, double h, double a, double alen, Color c = WHITE, Color f = TRANSPARENT);
//void   rect(Vec2d corner, double w, double h, Color c = WHITE, Color f = TRANSPARENT);
//void   polygon(std::vector<Vec2d> pts, Color border, Color fill = TRANSPARENT);
//void   polyline(std::vector<Vec2d> pts, Color color);
//void   text(Vec2d pos, double size, const std::string& str, Color textColor = WHITE, HAlign hAlign = HAlign::left, VAlign vAlign = VAlign::baseline);



#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include <rplidar.h>
using namespace std;
using namespace sl;
int main()
{
    ///  Create a communication channel instance
       IChannel* _channel;
       Result<IChannel*> channel = createSerialPortChannel("COM8", 115200);
       ///  Create a LIDAR driver instance
       ILidarDriver * lidar = *createLidarDriver();
       auto res = (lidar)->connect(*channel);
       if(SL_IS_OK(res)){
           sl_lidar_response_device_info_t deviceInfo;
           res = (lidar)->getDeviceInfo(deviceInfo);
           if(SL_IS_OK(res)){
               printf("Model: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
               deviceInfo.model,
               deviceInfo.firmware_version >> 8, deviceInfo.firmware_version & 0xffu,
               deviceInfo.hardware_version);
           }else{
               fprintf(stdout, "Failed to get device information from LIDAR %08x\r\n", res);
           }
       }else{
           fprintf(stdout, "Failed to connect to LIDAR %08x\r\n", res);
       }
       // TODO

       /// Delete Lidar Driver and channel Instance
       delete lidar;
       delete *channel;
    Graphics g("MyProgram", 1024, 768);


    while (g.draw()) {
        if (g.isKeyPressed(Key::ESC)) {
            break;
        }


        for (const Event& e : g.events()) {
            switch (e.evtType) {
            case EvtType::KeyPress:
                break;
            case EvtType::KeyRelease:
                break;
            case EvtType::MouseMove:
                break;
            case EvtType::MousePress:
                break;
            case EvtType::MouseRelease:
                break;
            default:
                break;
            }
        }
    }

    return 0;
}


