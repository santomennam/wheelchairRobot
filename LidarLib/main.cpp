#include <iostream>
#include "graphics.h"
#include <iomanip>
#include "serialport.h"
#include "CmdLink.h"
#include "lidar.h"
#include <fstream>

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

void write(ostream& strm, vector<Vec2d> pts)
{
    strm << "0, 0, 0" << endl;
    for (auto& p : pts) {
        strm << p.x << ", " << p.y << ", " << endl;
    }
}

void write(ostream& strm, bool isStartScan, const LidarData& data)
{
    strm << (isStartScan ? 1 : 0) << ", " << data.quality << ", " << data.angle << ", " << data.distance << endl;
}

#include "bitrange.h"
using namespace twiddle;

string chooseSerialPort(Graphics& g)
{
    string portName;

    SerialPorts ports;

    while (g.draw() && portName.empty()) {

        g.cout << "Press a number key to choose the port: " << endl;
        g.cout << "Escape to cancel" << endl << endl;
        for (int i = 0; i < ports.size(); i++) {
            g.cout << "Port " << i << ": " << ports[i] << endl;
        }

        for (const Event& e : g.events()) {
            switch (e.evtType) {
            case EvtType::KeyPress:
                if (e.arg >= '0' && e.arg < '0'+ports.size()) {
                    portName = ports[e.arg - '0'];
                }
                if (e.arg == static_cast<int>(Key::ESC)) {
                    return "";
                }
                break;
            default:
                break;
            }
        }
    }

    return portName;

}


int main()
{
    Graphics g("LidarThing", 1000, 1000);

    ofstream out_raw;
    ofstream out_pts;


    string portName = chooseSerialPort(g);

    bool useImg = false;

    double scale = 1.0;

    Image img(g, 1000, 1000, BLACK, true);

    vector<vector<Vec2d>> points;
    vector<Vec2d> incomingPoints;

    int scansToKeep = 10;

    Lidar lidar(portName, [&](bool startSweep, const LidarData& point) {

        if (out_raw.is_open()) {
            write(out_raw, startSweep, point);
        }

        if (startSweep) {
            if (out_pts.is_open()) {
                write(out_pts, incomingPoints);
            }
            points.push_back(std::move(incomingPoints));
            incomingPoints.clear();
            if (points.size() > scansToKeep) {
                points.erase(points.begin());
            }
        }
        if (point.quality < 5) {
            return;
        }

        Vec2d pt{point.distance, 0};
        pt.rotate(M_PI * point.angle / 180.0);
        incomingPoints.push_back(pt);

        if (useImg) {

            int x = incomingPoints.back().x* (1.0/scale)+img.width()/2;
            int y = incomingPoints.back().y* (1.0/scale)+img.height()/2;
            if (x >= 0 && x < img.width() && y >= 0 && y < img.height()) {
                img.setPixel(x, y, YELLOW);
            }
        }

    });

    while (g.draw()) {

        lidar.update();

        g.cout << lidar.getStateString() << endl;
        g.cout << "Timeout: " << lidar.currentTimeoutVal() << endl;
        g.cout << "RotFreq: " << lidar.desRotFreq() << endl;
        g.cout << "IdleTime: " << lidar.currentIdleTime() << endl;

        Color c = YELLOW;

        double color_h;
        double color_s;
        double color_v;

        rgb2hsv(c, color_h, color_s, color_v);


        if (useImg) {
            img.updatePixels();
            g.image({0,0}, img);
        }
        else {
            for (int i = 0; i < points.size(); i++) {
                vector<Vec2d> pts = points[i];
                color_v = (1.0/points.size())*(i+1);
                c = hsv2rgb(color_h, color_s, color_v);
                for (auto& p : pts) {
                    p = p * (1.0/scale);
                    p = p + Vec2d{g.width()/2, g.height()/2};
                }
                g.points(pts, c);
            }
        }

        g.ellipse({g.width()/2, g.height()/2}, 75/scale, 75/scale, GREEN, GREEN);

        for (const Event& e : g.events()) {
            switch (e.evtType) {
            case EvtType::KeyPress:
                switch (e.arg) {
//                case 'I':
//                    lidar.cmdGetInfo();
//                    break;
                case 'M':
                    lidar.cmdMotorSpeed(lidar.desRotFreq());
                    break;
                case 'S':
                    lidar.cmdMotorSpeed(0);
                    break;
                case 'G':
                    lidar.cmdBeginScan();
                    break;
                case 'E':
                    lidar.cmdBeginExpressScan(3);
                    break;
                case 'R':
                    lidar.cmdReset();
                    break;
                case 'I':
                    useImg = !useImg;
                    if (useImg) {
                        img.set(g.width(), g.height(), BLACK, true);
                    }
                    else {
                        img.set(10,10,BLACK,true);
                    }
                    break;
                case 'W':
                    break;
                case 'C':
                    out_raw.close();
                    out_pts.close();
                    break;
                case 'F':
                    out_raw.open("Raw.csv");
                    out_pts.open("Pts.csv");
                    break;
                case 'T':
                    lidar.cmdReqTypicalMode();
                    break;
                case '0':
                    lidar.cmdReqConfModeCount();
                    break;
                case '1':
                    lidar.cmdReqConfAnsType(0);
                    lidar.cmdReqConfUsPerSample(0);
                    lidar.cmdReqConfName(0);
                    break;
                case '2':
                    lidar.cmdReqConfAnsType(1);
                    lidar.cmdReqConfUsPerSample(1);
                    lidar.cmdReqConfName(1);
                    break;
                case '3':
                    lidar.cmdReqConfAnsType(2);
                    lidar.cmdReqConfUsPerSample(2);
                    lidar.cmdReqConfName(2);
                    break;
                case '4':
                    lidar.cmdReqConfAnsType(3);
                    lidar.cmdReqConfUsPerSample(3);
                    lidar.cmdReqConfName(3);
                    break;

                }

                break;
            case EvtType::MouseWheel:
                if (e.arg > 0) {
                    scale = std::min(20.0, scale + 1);
                }
                else {
                    scale = std::max(1.0, scale - 1);
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


