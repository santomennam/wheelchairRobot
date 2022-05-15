#include <iostream>
#include <memory>
#include "graphics.h"
#include <cmath>
#include <functional>
#include "segment.h"
#include "vec2d.h"
#include <vector>
#include "area.h"
#include "areatree.h"
#include "world.h"
#include "serialportreader.h"
#include "networkplugin.h"
#include <iomanip>
#include <strstream>
#include <fstream>
#include "viewport.h"
#include "physics.h"
#include<windows.h>
#include "botconnection.h"
#include "botcommserial.h"
#include "botcommmonitor.h"

#include "Lidar/lidar.h"

#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wnarrowing"

using namespace std;
using namespace mssm;
using namespace ClipperLib;


void resetDestination(BotConnection& bot, World& world, Vec2d destination)
{
    world.path = vector<Vec2d>{destination};//,{100,-50},{0,0}};
    world.findEncPath(world.path);
    for(int i = 0; i < world.targets.size(); i++)
    {
        if(i > 1){
            cout<<"Pos "<<world.targets[i-1].pos<<" and ";
        }
        cout<<"dest "<<world.targets[i].pos<<" with angle "<<to_string(world.targets[i].angle*(180/M_PI)) <<" degrees and enc readings "<<world.targets[i].encoderReadings <<". this is " << (world.targets[i].turn ? "a turn." : "not a turn.") <<endl;
    }
    cout<<"Initial target size: "<<to_string(world.targets.size())<<endl;
}

vector<Vec2d> parser(istream& file)
{
    vector<Vec2d> pts;
    string line;
    while(getline(file,line))
    {
        stringstream ss(line);
        Vec2d pt;
        ss>>pt.x;
        ss.ignore(1);
        ss>>pt.y;
        pts.push_back(pt);
    }
    return pts;
}


class SerialPort : public ISerialPort
{
public:
    Graphics& g;
    int pluginId{-1};
    string incoming;
    // ISerialPort interface
public:
    SerialPort(Graphics& g) : g{g} {}

    void open(std::string port);

    bool canRead() const override;
    string read() override;
    void write(string data) override;
    bool stillWriting() override;

    void addIncoming(string str) { incoming.append(str); }
};

void SerialPort::open(string connectionName)
{
    pluginId = g.registerPlugin([connectionName](QObject* parent) {
          return new SerialPortReader(parent, connectionName, QSerialPort::Baud115200);
    });
}

bool SerialPort::canRead() const
{
    return !incoming.empty();
}

string SerialPort::read()
{
    string tmp{std::move(incoming)};
    incoming.clear();
    return tmp;
}

void SerialPort::write(string data)
{
    g.callPlugin(pluginId,static_cast<int>(SerialPortReader::Command::send),0, data);

}

bool SerialPort::stillWriting()
{
    return false;
}

void graphicsMain(Graphics& g)
{
    Vec2d destination = {0,0};

    bool targetMode = false;

    bool drawAdj = false;

    physics phys;

    Vec2d previous;
    bool recording = false;


    BotCommSerial botSerial(g);

    BotCommMonitor botMonitor(&botSerial);

    BotConnection bot(&botMonitor);


    botSerial.connect("COM5");

    ofstream file;
    ifstream input;
    bool playback = false;
    // std::chrono::time_point<std::chrono::steady_clock> queriedTime;
    Vec2d trackingPair;
    bool clicked = false;
    Vec2d point;
    vector<Vec2d> pathPoints;
    double width = 1000;
    double height = 1000;


    World world({0,0},{{100,100}},width,height);

    // send messages from bot to the world
    bot.setOnEncoderUpdateHandler([&world](Vec2d encoders) { world.botEncoderUpdated(encoders); });
    bot.setOnTargetUpdateHandler([&world](Vec2d targets) { world.botTargetUpdated(targets); });
    bot.setOnMotorUpdateHandler([&world](Vec2d motors) { world.botMotorsUpdated(motors); });

    phys.botWidth = world.tree.botWidth;
    //string incomingData;

    vector<Vec2d> midpoints;
    double stepx = width/5;
    double stepy = height/10;

    for (double x = (stepx/2)-width; x < width; x += stepx) {
        for (double y = (stepy/2)-height; y < height; y += stepy) {
            midpoints.push_back({x,y});
        }
    }

    world.worldScale = 0.04;
    world.view.pan(Vec2d{g.width()/2, g.height()/2});

    string lastLine;

    //resetDestination(bot, world, {0, 0});

    bool arrowPressed = false;
    bool resetRequested = false;
    bool targetRequested = false;

    Vec2d target;

    vector<double> leftMotorSpeed;
    vector<double> rightMotorSpeed;

    SerialPort port(g);
    port.open("COM10");


    ifstream lidarSimPoints(R"(C:\Users\Marcello Santomenna\wheelchairRobot\navBrain\data\Pts.csv)");
    vector<Vec2d> lidarPoints = parser(lidarSimPoints);


    Lidar lidar(&port, [](bool startSweep, const LidarData& point) {
        cout << "Data!!!" << endl;
    });


    while (g.draw()) {
        g.clear();

        lidar.update();

        bot.update();

        Vec2d mot = bot.getMotors();

        leftMotorSpeed.push_back(mot.x);
        rightMotorSpeed.push_back(mot.y);

        while (leftMotorSpeed.size() > 300) {
            leftMotorSpeed.erase(leftMotorSpeed.begin());
            rightMotorSpeed.erase(rightMotorSpeed.begin());
        }

        int textY = g.height();

        if (bot.inDriveableState()) {
            if (bot.readyForNextCommand()) {
                g.text({10,textY -= 25}, 20, "Ready", GREEN);
            }
            else if (bot.elapsedSinceLastResponse() > 1) {
                g.text({10,textY -= 25}, 20, "Waiting", RED);
            }
            else {
                g.text({10,textY -= 25}, 20, "Waiting", YELLOW);
            }
        }

        //g.text({10,textY -= 25}, 20, "incomingResponse: " + world.incomingData, GREEN);
        g.text({10,textY -= 25}, 20, "Last Command:    " + bot.lastCommand(), GREEN);
        g.text({10,textY -= 25}, 20, "receivedResponse: " + bot.getReceivedCommand(), GREEN);
        if (!bot.getReceivedInfo().empty()) {
            g.text({10,textY -= 25}, 20, "INFO:     " + bot.getReceivedInfo(), YELLOW);
        }
        if (!bot.getReceivedError().empty()) {
            g.text({10,textY -= 25}, 20, "ERROR:    " + bot.getReceivedError(), RED);
        }

        //        std::chrono::duration<double> diff = std::chrono::steady_clock::now() - world.lastTime;
        //        if(diff.count() >= 1)
        //        {
        //            g.rect(g.width()/2-g.width()*0.04,g.height()*0.05,60,10,RED,RED);
        //            g.text(g.width()/2-g.width()*0.04,g.height()*0.05,10,"TIMED OUT",WHITE);
        //        }


        g.text({300,g.height()-30}, 20, bot.stateStr());


        if(drawAdj)
        {
            world.tree.showAdjacents(g,world.view);
        }

        g.ellipseC(world.view.worldToScreen(destination),10,10,RED,WHITE);
        world.draw(g);
        if(recording)
        {
            g.rect(g.width()-20,g.height()-20,10,10,RED,RED);
        }

        if(playback)
        {
            g.rect(g.width()-20,g.height()-20,10,10,GREEN,GREEN);
        }

        vector<Vec2d> copynav = world.path;
        for(auto& n : copynav)
        {
            n=world.view.worldToScreen(n);
        }

        world.updateTargets();

        if(pathPoints.size())
        {
            vector<Vec2d> tmp = pathPoints;
            for(auto& n : tmp)
            {
                n=world.view.worldToScreen(n);
            }
            g.polyline(tmp,GREEN);
        }

        if(world.targetsChanged && world.targets.size())
        {
            world.targetsChanged = false;
            Vec2d target = world.targets[0].encoderReadings;
            trackingPair = target;

            //           sendTarget(g, target,boardPluginID);
        }
        else if (world.targetsChanged) {
            g.rect(g.width()/2-g.width()*0.04,g.height()*0.05,60,10,GREEN,GREEN);
            g.text(g.width()/2-g.width()*0.04,g.height()*0.1,10,"Reached Dest",WHITE);
        }

        //draw info
        if(world.diagnostics){
            g.text(10,130,20,"Current encoder counts: " + world.posTracker.encoderReadings.toIntString(),WHITE);

            g.polyline(copynav,GREEN);

            g.text(g.width()-200,g.height()-20,20,"Targets");
            for(int i = 0; i < world.targets.size(); i++)
            {
                g.text(g.width()-200,g.height()-(40+20*i),20,world.targets[i].pos.toIntString());
            }

            //g.text(10,10,20, "Scale: "+to_string(world.view.scale));

            //g.text(10,155,20,"Target encoder counts: "+trackingPair.toIntString());

            //g.text(g.width() - 400, 105, 20, "Velocities: " + to_string(world.vel1) +" " + to_string(world.vel2));

        }

        int arrows =
                (g.isKeyPressed(Key::Up)    ? 0x01 : 0x00) |
                (g.isKeyPressed(Key::Down)  ? 0x02 : 0x00) |
                (g.isKeyPressed(Key::Left)  ? 0x04 : 0x00) |
                (g.isKeyPressed(Key::Right) ? 0x08 : 0x00);

        if (arrowPressed && !arrows) {
            // just released arrows
            if (bot.readyForNextCommand()) {
                arrowPressed = false;
                bot.tankSteer(0,0);
            }
        }

        if (arrows) {
            arrowPressed = true;

            if (!bot.inDriveableState()) {
                g.text({300,g.height()-80}, 20, "Cannot Drive in this mode!", RED);
            }

            if (bot.inDriveableState() && bot.readyForNextCommand()) {
                switch (arrows) {
                case 0x01: // up only
                    bot.tankSteer(1,1);
                    break;
                case 0x02: // down only
                    bot.tankSteer(-1,-1);
                    break;
                case 0x04: // left only
                    bot.tankSteer(-1,1);
                    break;
                case 0x08: // right only
                    bot.tankSteer(1,-1);
                    break;
                case 0x05: // left && up
                    bot.tankSteer(0,1);
                    break;
                case 0x09: // right && up
                    bot.tankSteer(1,0);
                    break;
                case 0x06: // left && down
                    bot.tankSteer(-1,0);
                    break;
                case 0x0A: // right && down
                    bot.tankSteer(0,-1);
                    break;
                }
                //h
            }
            else {
                //cout << "Not ready for arrows" << endl;
            }
        }

        for (const Event& e : g.events())
        {
            switch (e.evtType)
            {
            default:
                // cout << e << endl;
                break;
            case EvtType::MousePress:
                if(e.arg == 1)
                {
                    if(clicked){
                        pathPoints = world.tree.navigation(world.view.screenToWorld(point),world.view.screenToWorld(g.mousePos()),g,world.view);
                        clicked = false;
                    }
                    else{
                        clicked = true;
                        point = g.mousePos();
                    }
                }
                previous = g.mousePos();
                break;
            case EvtType::MouseRelease:
                break;
            case EvtType::MouseWheel:
                if(e.arg > 10)
                {
                    world.view.zoom(g.mousePos(),1.1);
                }
                else{
                    world.view.zoom(g.mousePos(),0.9);
                }
                break;
            case EvtType::MouseMove:
                if(e.arg == 2)
                {
                    world.view.pan(g.mousePos()-previous);
                    previous=g.mousePos();
                }
                break;
            case EvtType::PluginMessage:
                if(e.pluginId == botSerial.getPluginId())
                {
                    //cout << "RAWRAW: " << e.data << endl;
                    botSerial.handleRawData(e.data);
                }
                else if (e.pluginId == port.pluginId) {
                    port.addIncoming(e.data);
                }
                break;

            case EvtType::KeyPress:
                switch(e.arg)
                {
                case static_cast<int>(Key::ESC):
                    resetRequested = true;
                    world.targets.clear();
                    break;
                case '>':
                    targetRequested = true;
                    target = {4000,4000};
                    // resetDestination(g, world, {10, 0},boardPluginID);
                    break;
                case '<':
                    targetRequested = true;
                    target = {-4000,-4000};
                    //  resetDestination(g, world, {-10, 0},boardPluginID);
                    break;
                case '.':
                    targetRequested = true;
                    target = {2000,2000};
                    // resetDestination(g, world, {10, 0},boardPluginID);
                    break;
                case ',':
                    targetRequested = true;
                    target = {-2000,-2000};
                    //  resetDestination(g, world, {-10, 0},boardPluginID);
                    break;
                case 'D':
                    // world.diagnostics = !world.diagnostics;
                    bot.setDebug(true);
                    break;
                case 'U':
                    targetMode = !targetMode;
                    if(targetMode)
                    {
                        world.masterNav({600,800},g);
                        world.findEncPath(world.path);
                        for(int i = 0; i < world.targets.size(); i++)
                        {
                            if(i > 1){
                                cout<<"Pos "<<world.targets[i-1].pos<<" and ";
                            }
                            cout<<"dest "<<world.targets[i].pos<<" with angle "<<to_string(world.targets[i].angle*(180/M_PI)) <<" degrees and enc readings "<<world.targets[i].encoderReadings <<". this is " << (world.targets[i].turn ? "a turn." : "not a turn.") <<endl;
                        }
                    }
                    break;
                case 'X':
                {
                    packet p = world.tree.findClosestNode(world.view.screenToWorld(g.mousePos()));
                    if (p.node) {
                        for (int i = 0; i < p.node->boundaries.size(); i++) {
                            segment& s = p.node->boundaries[i];
                            s.open = false;
                            for (auto& sa : p.node->adjacents[i]->sharedSegsP(p.node)) {
                                sa->open = false;
                            }
                        }
                    }

                }break;
                case 'A':
                    drawAdj = !drawAdj;
                    break;
                case ' ':
                    world.showObstacle = !world.showObstacle;
                    break;
                case 'B':
                    world.showBeam = !world.showBeam;
                    break;
                case 'O':
                    world.placeObstaclesFromList(vector<Vec2d>(lidarPoints.begin(),lidarPoints.begin()+50));
                    lidarPoints.erase(lidarPoints.begin(),lidarPoints.begin()+50);
//                   world.createRandomObstacles(g);
                    break;
                }
                break;
            }
        }

        if (resetRequested && (bot.readyForNextCommand() || bot.elapsedSinceLastSend() >= 0.2)) {
            bot.resetBot();
            resetRequested = false;
            targetRequested = false;
        }
        else if (targetRequested && bot.readyForNextCommand()) {
            bot.sendTarget(target);
            targetRequested = false;
        }
        else if(bot.elapsedSinceLastSend() >= 0.2 && bot.readyForNextCommand())
        {
            bot.keepBotAlive();
        }

        if (targetRequested && !bot.inDriveableState()) {
            g.text({300,g.height()-55}, 20, "Cannot Target in this mode!", RED);
        }
    }

    cout << "Application closing: reset bot" << endl;
    bot.resetBot();

}

int main()
{
    // main should be empty except for the following line:
    Graphics g("Navigation", 800, 600, graphicsMain);
}


