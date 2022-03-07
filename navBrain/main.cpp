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
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wnarrowing"
using namespace std;
using namespace mssm;

//void draw(Graphics&g, World& world)//(trueCoords coordinatePlane,Robot robot,Graphics&g)
//{
//    world.robot.update();
//    world.sensorCoords();

//    vector<Vec2d> tempPoints = world.robot.pointsToDraw;
//    for (Vec2d& temp:tempPoints)
//    {
//        temp.rotate(world.robot.angle);
//        temp.x += world.robot.position.x;
//        temp.y += world.robot.position.y;
//    }
//    cout<<"draw"<<endl;
//    if(world.showBeam){
//        //for(int i = 0; i<world.robot.measuredPoints.size(); i++)
//        //{
//        g.polyline(world.robot.measuredPoints,WHITE);
//        //}
//    }

//    if(world.showObstacle)
//    {
//        for(int i = 0; i<world.obstacles.size();i++){
//            for(int j =0; j<world.obstacles[i].pts.size();j++)
//            {
//                g.point(world.obstacles[i].pts[j], RED);
//            }
//        }
//    }
//    g.polygon(tempPoints,WHITE,WHITE);
//    //g.polyline(obstacle.pts,RED);
//    for(int i = 0; i<world.sensedCoords.size();i++)
//    {
//        g.point(world.sensedCoords[i],BLUE);
//    }
//}

void graphicsMain(Graphics& g)
{

    bool drawAdj = false;

    //    todo:
    //        -write tests on carefully placed points to make sure we continuously do things right
    //        -fix the uses of sharedseg
    //          -run under debugger, try to plot path through the node formed after pressing T a couple times - notice how it doesnt splitEdge, instead it makes a new, thin node
    physics phys;

    Vec2d previous;
    bool recording = false;
    int boardPluginID = g.registerPlugin([](QObject* parent) { return new SerialPortReader(parent, "COM4",QSerialPort::Baud9600); });
//    int dataPluginID = g.registerPlugin([](QObject* parent) { return new SerialPortReader(parent, "COM5",QSerialPort::Baud115200); });

    NetworkClientPlugin simWorldConnection{g, 1237, "localhost"};

    //string data;
    ofstream file;
    ifstream input;
    bool playback = false;
    Vec2d merge2;
    bool merge = false;
    bool clicked = false;
    Vec2d point;
    vector<Vec2d> points;
    double width = 10000;
    double height = 10000;
    World world({g.width()/2, g.height()/2},{{100,100}},width,height);
    phys.botWidth = world.tree.botWidth;
    string incomingData;
    long long lastSendTime = g.time();
    //world.createObstacles(g);
    //  Node* closestNode = nullptr;

    vector<Vec2d> midpoints;

    double stepx = width/5;
    double stepy = height/10;

    for (double x = (stepx/2)-width; x < width; x += stepx) {
        for (double y = (stepy/2)-height; y < height; y += stepy) {
            midpoints.push_back({x,y});
        }
    }

    world.worldScale = 0.04;
    world.panOffset = {500, 500};

    string lastLine;
    stringstream ss;

    ss<<"debug"<<'\n';
    g.callPlugin(boardPluginID,static_cast<int>(SerialPortReader::Command::send),0,ss.str()); //TEMPORARY

    Vec2d destination{world.robot.position.x+10,world.robot.position.y+10};
    while (g.draw()) {

        //know circumferences, each wheel travels half of the section of the circumference

//        world.masterNav(destination,g); // COMMENTED OUT ON 1/26/22

        g.clear();
        if(drawAdj)
        {
            world.tree.showAdjacents(g,world.view);
        }
        Vec2d front = {world.phys.position.x+20,world.phys.position.y};
        front.rotate(world.phys.angle);
        g.line(world.view.worldToScreen(world.phys.position),world.view.worldToScreen(front));
        g.ellipseC(world.view.worldToScreen(destination),10,10,RED,RED);
        world.phys.draw(g,world.view);
        //        g.ellipseC({g.height(),g.width()},50,50,RED);
        world.draw(g);
        //        while (midpoints.size()>0) {
        //            world.tree.placer(midpoints.back());
        //            midpoints.pop_back();
        //        }
        if(recording)
        {
            g.rect(g.width()-20,g.height()-20,10,10,RED,RED);
        }

        if(playback)
        {
            g.rect(g.width()-20,g.height()-20,10,10,GREEN,GREEN);
        }
        //  world.tree.resetColor(g);
        //        packet pack = world.tree.findClosestNode((g.mousePos()));
        //        closestNode = pack.node;
        //        //    closestNode->color = YELLOW;
        //        g.point(pack.closestOnNode,RED);

        //  draw(g,world);

        g.text(10,10,20, "Scale: "+to_string(world.view.scale));

        vector<Vec2d> copynav = world.path;
        for(auto& n : copynav)
        {
            n=world.view.worldToScreen(n);
        }

        //cout<<"bruh"<<endl;

//        if(g.time()-lastSendTime>100) //milliseconds
//        {
//            lastSendTime = g.time();
//            stringstream ss;
//            ss<<"power "<<setw(4)<<static_cast<int>(world.phys.leftPower)<<" "<<setw(4)<<static_cast<int>(world.phys.rightPower)<<'\n';
//            cout<<ss.str()<<endl;
//            if(abs(world.phys.leftPower) > 127 || abs(world.phys.rightPower) > 127)
//            {
//                cout<<"motor power too large"<<endl;
//            }
//            //string dataSend = "power " + to_string(static_cast<int>(world.phys.leftPower)) + " " + to_string(static_cast<int>(world.phys.rightPower))+"\n";
//            g.callPlugin(boardPluginID,static_cast<int>(SerialPortReader::Command::send),0,ss.str());
//        }
        g.text(10,130,20,"power "+to_string(world.phys.leftPower)+", "+to_string(world.phys.rightPower),WHITE);
        g.polyline(copynav,GREEN);

        if(playback)
        {
            string line;
            if(getline(input,line)){
                world.dataInterp(line);
                cout<<line<<endl;
            }
        }
        world.phys.processMovement(20);
        for (const Event& e : g.events())
        {

         //SIMWORLD STUFF

//            NetworkSocketEvent socketEvent;
//            string socketData;

//            if (simWorldConnection.handleEvent(e, socketEvent, socketData)) {
//                // got a network event of some sort
//                switch (socketEvent)
//                {
//                case NetworkSocketEvent::connected:
//                    cout  << "Socket Connected to: " << socketData.c_str() << endl;
//                    simWorldConnection.send("Boop\n");
//                    break;
//                case NetworkSocketEvent::disconnected:
//                    cout  << "Socket Disconnected " << socketData.c_str() << endl;
//                    //connection.reConnect(1234, "localhost");
//                    //gameServer.closePlugin();
//                    break;
//                case NetworkSocketEvent::error:
//                    cout  << "Socket Network Error " << socketData.c_str() << endl;
//                    break;
//                case NetworkSocketEvent::other:
//                    cout  << "Socket Network Other " << socketData.c_str() << endl;
//                    break;
//                case NetworkSocketEvent::data:
//                    cout  << "Socket Data Packet1: " << socketData.c_str() << endl;
//                    world.dataInterp(socketData.c_str()); // this is where we handle data from the sim world

//                    // bot has gotten some data from the server... handle it
//                    //bot.handleReceivedData(g, socketData);
//                    break;
//                }

//                continue;
//            }

            switch (e.evtType)
            {

            case EvtType::MousePress:
                if(e.arg == 1)
                {
                    if(clicked){
                        points = world.tree.navigation(world.view.screenToWorld(point),world.view.screenToWorld(g.mousePos()),g,world.view);
                        //                        cout<<world.tree.doesSegmentCollide(world.view.screenToWorld(point),world.view.screenToWorld(g.mousePos()))<<endl;
                        //                        cout<<"nav"<<endl;
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

                    int find = e.data.find("\r\n");
                    if(find != -1 && e.data != "ok\r\n"){
                    cout<<e.data<<endl;
                    }
                }
                if(e.pluginId == boardPluginID)
                {
                    if(recording)
                    {
                        file<<e.data;
                        file.flush();
                        cout<<"recording"<<endl;
                    }
                    if(!playback&&!recording){
                        world.dataInterp(e.data);
                    }
                }
                break;

            case EvtType::KeyPress:
                switch(e.arg)
                {
                case '8':
                    cout << "Try to say hello back" << endl;
                    simWorldConnection.send("Hello!!!!\n");
                    break;
                case 'S':
                    world.save("loadTestOne");
                    break;
                case 'L':
                    world.load("loadTestOne");
                    break;
                case 'P':
                    recording = false;
                    playback = !playback;
                    if(playback){
                        input.open("serialData.txt");
                    }
                    break;
                case 'T':
                    cout<<"sending target"<<endl;
                    ss<<"target R 4000"<<'\n';
                    g.callPlugin(boardPluginID,static_cast<int>(SerialPortReader::Command::send),0,ss.str()); //TEMPORARY
                    ss<<"target L 4000"<<'\n';
                    g.callPlugin(boardPluginID,static_cast<int>(SerialPortReader::Command::send),0,ss.str());
                    //                    if (!midpoints.empty()) {
                    //                        world.tree.placer(midpoints.back());
                    //                        midpoints.pop_back();
                    //                    }
//                    destination = world.view.screenToWorld(g.mousePos());

                    break;
                case 'D':
                   g.callPlugin(boardPluginID,static_cast<int>(SerialPortReader::Command::send),0,"debug");
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
                case 'R':
                    recording = !recording;
                    playback = false;
                    if(recording)
                    {
                        file = ofstream("12112020.txt");
                    }
                    else{
                        file.close();
                    }
                    break;
                case 'C':
                    world.tree.clear();
                    break;
                case 'M':
                    if(merge)
                    {
                        world.tree.combineNodes(g.mousePos(),merge2);
                        merge = false;
                    }
                    else{
                        merge = true;
                        merge2 = g.mousePos();
                    }
                    break;
                case static_cast<int>(Key::ESC):
                {
                    world.view.reset(world.robot.position);
                    break;
                }
                case 'W':
                    world.robot.speed =10;
                    break;
                    //                case 'S':
                    //                    world.robot.speed =-10;
                    //                    break;
                case 'A':
                    drawAdj = !drawAdj;
                    break;
                    //                case 'D':
                    //                    world.robot.angle -= 0.1;
                    //                    break;
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
            case EvtType::KeyRelease:
                switch(e.arg)
                {
                case 'S':
                case 'W':
                    world.robot.speed = 0;
                    break;
                case 'D':
                case 'A':
                    world.robot.angularVelocity = 0;
                    break;
                default:
                    break;
                }
            }
        }
    }
}

int main()
{
    // main should be empty except for the following line:
    Graphics g("Drawing", 800, 600, graphicsMain);
}
