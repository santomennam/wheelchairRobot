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
    int boardPluginID = g.registerPlugin([](QObject* parent) { return new SerialPortReader(parent, "COM5",QSerialPort::Baud115200); });

    NetworkClientPlugin simWorldConnection{g, 1237, "localhost"};

    //string data;
    ofstream file;
    ifstream input;
    bool playback = false;
    Vec2d queriedLoc;
    std::chrono::time_point<std::chrono::steady_clock> queriedTime;
    Vec2d merge2;
    Vec2d trackingPair;
    bool merge = false;
    bool clicked = false;
    Vec2d point;
    vector<Vec2d> points;
    double width = 10000;
    double height = 10000;
    World world({0,0},{{100,100}},width,height);
    phys.botWidth = world.tree.botWidth;
    string incomingData;
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
    world.view.pan(Vec2d{g.width()/2, g.height()/2});

    string lastLine;
    stringstream ss;

    ss<<"debug"<<'\n';
    g.callPlugin(boardPluginID,static_cast<int>(SerialPortReader::Command::send),0,ss.str()); //TEMPORARY


    //TESTING THE PATH NAV SYSTEM
    Vec2d destination{100,0};
    //    world.masterNav(destination,g);
    world.path = vector<Vec2d>{{50,0}};
    //    vector<Vec2d> path = {{300,200},{600,800}};
    world.findEncPath(world.path);
    for(int i = 0; i < world.targets.size(); i++)
    {
        if(i > 1){
            cout<<"Pos "<<world.targets[i-1].pos<<" and ";
        }
        cout<<"dest "<<world.targets[i].pos<<" with angle "<<to_string(world.targets[i].angle*(180/M_PI)) <<" degrees and enc readings "<<world.targets[i].encoderReadings <<". this is " << (world.targets[i].turn ? "a turn." : "not a turn.") <<endl;
    }
    cout<<"Initial target size: "<<to_string(world.targets.size())<<endl;

    while (g.draw()) {
        //        cout<<"Target size: "<<to_string(world.targets.size())<<endl;
        std::chrono::duration<double> diff = std::chrono::steady_clock::now() - world.lastTime;
        if(diff.count() >= 1)
        {
            g.rect(g.width()/2-g.width()*0.04,g.height()*0.05,60,10,RED,RED);
            g.text(g.width()/2-g.width()*0.04,g.height()*0.05,10,"TIMED OUT",WHITE);
        }

        //know circumferences, each wheel travels half of the section of the circumference

        //        world.masterNav(destination,g); // COMMENTED OUT ON 1/26/22



        g.clear();
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
            cout<<"sending target "<<target<<endl;
            ss<<"target "<<target.x<<" "<<target.y<<'\n';
            g.callPlugin(boardPluginID,static_cast<int>(SerialPortReader::Command::send),0,ss.str());
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
                g.text(g.width() - 300, 155, 20, "Last queried target: "+ world.queriedTarget.toIntString());
                g.text(g.width() - 300,125,10,to_string(diff.count()));
            }
        }


        if(playback)
        {
            string line;
            if(getline(input,line)){
                world.dataInterp(line);
                cout<<line<<endl;
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
                        cout<<e.data<<endl;
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
                    cout<<"ask"<<endl;
                    ss<<"ask"<<'\n';
                    g.callPlugin(boardPluginID,static_cast<int>(SerialPortReader::Command::send),0,ss.str());
                    queriedTime = std::chrono::steady_clock::now();
                    break;
                case 'D':
                    world.diagnostics = !world.diagnostics;
//                    g.callPlugin(boardPluginID,static_cast<int>(SerialPortReader::Command::send),0,"debug");
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
                //            case EvtType::KeyRelease:
                //                switch(e.arg)
                //                {
                //                case 'S':
                //                case 'W':
                //                    world.robot.speed = 0;
                //                    break;
                //                case 'D':
                //                case 'A':
                //                    world.robot.angularVelocity = 0;
                //                    break;
                //                default:
                //                    break;
                //                }
            }
        }
    }
}

int main()
{
    // main should be empty except for the following line:
    Graphics g("Drawing", 800, 600, graphicsMain);
}
