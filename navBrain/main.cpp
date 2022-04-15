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

#include "simplecrc.h"

#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wnarrowing"
using namespace std;
using namespace mssm;

string lastCommandSent;
tp lastCommandTime;
Vec2d lastSentTarget;

void sendCommand(Graphics& g, int boardPluginId, string cmd, bool debug)
{
    cmd = wrapDelimitedCRC8(cmd);
    if (debug) {
        cout << ">>>>> Sending Command to Bot: " << cmd << endl;
    }
    g.callPlugin(boardPluginId,static_cast<int>(SerialPortReader::Command::send),0,cmd);
    lastCommandSent = cmd;
    lastCommandTime = std::chrono::steady_clock::now();
}

void resetBot(Graphics& g,int boardPluginID)
{
    sendCommand(g, boardPluginID, "reset", true);

}

void ask(Graphics& g,int boardPluginID)
{
    sendCommand(g, boardPluginID, "ask", true);
}

void setDebugMode(Graphics& g,int boardPluginID)
{
    sendCommand(g, boardPluginID, "debug", true);
}

void keepBotAlive(Graphics& g,int boardPluginID)
{
    sendCommand(g, boardPluginID, "ping", false);
}

void sendTarget(Graphics& g, Vec2d encoderTarget,int boardPluginID)
{
    lastSentTarget = encoderTarget;
    stringstream ss;
    ss<<"target "<<encoderTarget.x<<" "<<encoderTarget.y;
    sendCommand(g, boardPluginID, ss.str(), true);
}

void resetDestination(Graphics& g, World& world, Vec2d destination,int boardPluginID)
{
    cout << "resetDestination calling resetBot" << endl;
    resetBot(g,boardPluginID);
    //setDebugMode(g,boardPluginID);
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


void graphicsMain(Graphics& g)
{
    Vec2d destination = {0,0};

    bool drawAdj = false;

    physics phys;

    Vec2d previous;
    bool recording = false;
    int boardPluginID = g.registerPlugin([](QObject* parent) { return new SerialPortReader(parent, "COM5",QSerialPort::Baud19200); });

    ofstream file;
    ifstream input;
    bool playback = false;
    std::chrono::time_point<std::chrono::steady_clock> queriedTime;
    Vec2d trackingPair;
    bool clicked = false;
    Vec2d point;
    vector<Vec2d> points;
    double width = 10000;
    double height = 10000;
    World world({0,0},{{100,100}},width,height);
    phys.botWidth = world.tree.botWidth;
    string incomingData;

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

    resetDestination(g,world,{10, 0},boardPluginID);

    bool arrowPressed = false;

    while (g.draw()) {
        g.clear();

        int textY = g.height();

        //g.text({10,textY -= 25}, 20, "incomingResponse: " + world.incomingData, GREEN);
        g.text({10,textY -= 25}, 20, "Last Command:    " + lastCommandSent, GREEN);
        g.text({10,textY -= 25}, 20, "receivedResponse: " + world.receivedCommand, GREEN);
        if (!world.receivedInfo.empty()) {
            g.text({10,textY -= 25}, 20, "ERROR:    " + world.receivedInfo, YELLOW);
        }
        if (!world.receivedError.empty()) {
            g.text({10,textY -= 25}, 20, "ERROR:    " + world.receivedError, RED);
        }

        std::chrono::duration<double> diff = std::chrono::steady_clock::now() - world.lastTime;
        if(diff.count() >= 1)
        {
            g.rect(g.width()/2-g.width()*0.04,g.height()*0.05,60,10,RED,RED);
            g.text(g.width()/2-g.width()*0.04,g.height()*0.05,10,"TIMED OUT",WHITE);
        }





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

            g.text(10,10,20, "Scale: "+to_string(world.view.scale));

            g.text(10,155,20,"Target encoder counts: "+trackingPair.toIntString());
            if(world.queried)
            {
                std::chrono::duration<double> diff = std::chrono::steady_clock::now() - queriedTime;
                g.text(g.width() - 400, 155, 20, "Last queried target: "+ world.queriedTarget.toIntString());
                g.text(g.width() - 400,125,10,to_string(diff.count()));
            }
            g.text(g.width() - 400, 105, 20, "Velocities: " + to_string(world.vel1) +" " + to_string(world.vel2));

        }

        int arrows =
                (g.isKeyPressed(Key::Up)    ? 0x01 : 0x00) |
                (g.isKeyPressed(Key::Down)  ? 0x02 : 0x00) |
                (g.isKeyPressed(Key::Left)  ? 0x04 : 0x00) |
                (g.isKeyPressed(Key::Right) ? 0x08 : 0x00);

        if (arrowPressed && !arrows) {
            // just released arrows
            arrowPressed = false;
            //sendTarget(g, world.posTracker.encoderReadings, boardPluginID);
        }
        if (arrows) {
            arrowPressed = true;
        }

        if ((world.posTracker.encoderReadings-lastSentTarget).magnitude() < 1000) {
            switch (arrows) {
            case 0x01: // up only
                sendTarget(g, world.posTracker.encoderReadings + Vec2d{ 2000, 2000 }, boardPluginID);
                break;
            case 0x02: // down only
                sendTarget(g, world.posTracker.encoderReadings + Vec2d{ -2000, -2000 }, boardPluginID);
                break;
            case 0x04: // left only
                sendTarget(g, world.posTracker.encoderReadings + Vec2d{ -1500, 1500 }, boardPluginID);
                break;
            case 0x08: // right only
                sendTarget(g, world.posTracker.encoderReadings + Vec2d{ 1500, -1500 }, boardPluginID);
                break;
            }
        }

        for (const Event& e : g.events())
        {
            switch (e.evtType)
            {
            case EvtType::MousePress:
                if(e.arg == 1)
                {
                    if(clicked){
                        points = world.tree.navigation(world.view.screenToWorld(point),world.view.screenToWorld(g.mousePos()),g,world.view);
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
                if(e.pluginId == boardPluginID)
                {
                    //cout << "----------\n" << e << "----------" << endl;
                    if(recording)
                    {
                        file<<e.data;
                        file.flush();
                        cout<<"recording"<<endl;
                    }
                    if(!playback&&!recording){
                        //                        cout<<e.data<<endl;
                        world.dataInterp(e.data);
                    }
                }
                break;

            case EvtType::KeyPress:
                switch(e.arg)
                {


                case static_cast<int>(Key::ESC):
                    resetBot(g,boardPluginID);
                    world.targets.clear();
                    break;
                case '>':
                    sendTarget(g, {4000,4000}, boardPluginID);
                    // resetDestination(g, world, {10, 0},boardPluginID);
                    break;
                case '<':
                    sendTarget(g, {-4000,-4000}, boardPluginID);
                    //  resetDestination(g, world, {-10, 0},boardPluginID);
                    break;
                case '.':
                    sendTarget(g, {2000,2000}, boardPluginID);
                    // resetDestination(g, world, {10, 0},boardPluginID);
                    break;
                case ',':
                    sendTarget(g, {-2000,-2000}, boardPluginID);
                    //  resetDestination(g, world, {-10, 0},boardPluginID);
                    break;
                case 'T':
                    ask(g,boardPluginID);
                    queriedTime = std::chrono::steady_clock::now();
                    break;
                case 'D':
                    world.diagnostics = !world.diagnostics;
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
                    world.createObstacles(g);
                    break;
                }
                break;
            }
        }


        std::chrono::duration<double> elapsedSinceSentCommand = std::chrono::steady_clock::now() - lastCommandTime;
        if(elapsedSinceSentCommand.count() >= 0.2)
        {
            keepBotAlive(g, boardPluginID);
        }
    }

    cout << "Application closing: reset bot" << endl;
    resetBot(g,boardPluginID);

}

int main()
{
    // main should be empty except for the following line:
    Graphics g("Navigation", 800, 600, graphicsMain);
}
