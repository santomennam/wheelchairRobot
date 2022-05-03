#include <iostream>
#include "graphics.h"
#include <iomanip>
#include "serialport.h"
#include "CmdLink.h"
#include "lidar.h"

using namespace std;
using namespace mssm;

#ifndef _MSC_VER
#pragma GCC diagnostic ignored "-Wsign-compare"
#endif

// Note:  Mac Users who are using Qt Creator...
//        if the program compiles and runs but a window doesn't open up
//        try un-checking "run in terminal"


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


int main()
{
    Graphics g("LidarThing", 1024, 768);



    Lidar lidar("COM10", [&g](const std::vector<LidarData>& data) {
        vector<Vec2d> pts;
        for (auto& sp : data) {
            Vec2d c{g.width()/2, g.height()/2};
            Vec2d pt{sp.distance, 0};
            pts.push_back(c+pt.rotated(sp.angle));

        }
        g.points(pts, WHITE);
    });

    while (g.draw()) {

        lidar.update();

        for (const Event& e : g.events()) {
            switch (e.evtType) {
            case EvtType::KeyPress:
                switch (e.arg) {
                case 'I':
                    lidar.cmdGetInfo();
                    break;
                case 'M':
                    lidar.cmdMotorSpeed(300);
                    break;
                case 'S':
                    lidar.cmdMotorSpeed(0);
                    break;
                case 'G':
                    lidar.cmdBeginScan();
                    break;
                case 'L':
                    break;
                case 'W':
                    break;
                case 'C':
                    break;
                case 'F':
                    break;
                }

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


