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

    vector<vector<Vec2d>> points;
    vector<Vec2d> incomingPoints;

    Lidar lidar("COM10", [&g, &points, &incomingPoints](bool startSweep, const LidarData& point) {
        if (startSweep) {
            points.push_back(std::move(incomingPoints));
            incomingPoints.clear();
            for (auto& p : points.back()) {
                p = p + Vec2d{g.width()/2, g.height()/2};
            }
            if (points.size() > 10) {
                points.erase(points.begin());
            }
        }
        if (point.quality < 15) {
            return;
        }
        if (point.distance < 10) {
            return;
        }

        Vec2d pt{point.distance/3, 0};
        incomingPoints.push_back(pt.rotated(M_PI * point.angle / 180.0));
    });

    while (g.draw()) {

        lidar.update();

        Color c = YELLOW;

        double color_h;
        double color_s;
        double color_v;

        rgb2hsv(c, color_h, color_s, color_v);

        color_v = 0.2;

        for (const auto& pts : points) {
            c = hsv2rgb(color_h, color_s, color_v);
            g.points(pts, c);
            color_v += 0.08;
        }

        for (const Event& e : g.events()) {
            switch (e.evtType) {
            case EvtType::KeyPress:
                switch (e.arg) {
                case 'I':
                    lidar.cmdGetInfo();
                    break;
                case 'M':
                    lidar.cmdMotorSpeed(600);
                    break;
                case 'S':
                    lidar.cmdMotorSpeed(0);
                    break;
                case 'G':
                    lidar.cmdBeginScan();
                    break;
                case 'R':
                    lidar.cmdReset();
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


