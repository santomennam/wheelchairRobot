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
    Graphics g("MyProgram", 1024, 768);


    while (g.draw()) {
        if (g.isKeyPressed(Key::ESC)) {
            break;
        }

//        if (g.isKeyPressed('Q')) {
//            m1.setVoltage(10);
//        }
//        else if (g.isKeyPressed('A')) {
//            m1.setVoltage(5);
//        }
//        else if (g.isKeyPressed('Z')) {
//            m1.setVoltage(-5);
//        }
//        else {
//            m1.setVoltage(0);
//        }

//        if (g.isKeyPressed('U')) {
//            m2.setVoltage(10);
//        }
//        else if (g.isKeyPressed('J')) {
//            m2.setVoltage(5);
//        }
//        else if (g.isKeyPressed('M')) {
//            m2.setVoltage(-5);
//        }
//        else {
//            m2.setVoltage(0);
//        }

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


