#include <iostream>
#include "graphics.h"
#include <iomanip>
#include "CmdLink.h"
#include <cmath>
#include "lidarwrapper.h"

using namespace std;
using namespace mssm;

#ifndef _MSC_VER
#pragma GCC diagnostic ignored "-Wsign-compare"
#endif

using namespace std;
using namespace sl;


void draw(Graphics& g, LidarWrapper lidar){
    g.clear();
    Vec2d center = {g.width()/2,g.width()};
    for (auto point : lidar.points)
    {
        g.point((point+center),WHITE);
    }
    g.draw();
}


void graphicsMain(Graphics& g)
{
    LidarWrapper lidar;

    while (g.draw()) {
        if (g.isKeyPressed(Key::ESC)) {
            break;
        }


        for (const Event& e : g.events()) {
            switch (e.evtType) {
            case EvtType::KeyPress:
                switch(e.arg)
                {
                    case 'S':
                        if(lidar.scan()){
                            draw(g,lidar);
                        }
                        break;
                    case 'K':
                        lidar.setMotorSpeed(0);
                        break;
                    default:
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
}

int main()
{
    Graphics g("LIDAR", 800, 600,graphicsMain);
}
//







