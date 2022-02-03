#ifndef WORLD_H
#define WORLD_H
#include <vector>
#include "graphics.h"
#include "robot.h"
#include "areatree.h"
#include "obstacle.h"
#include "segment.h"
#include "viewport.h"
#include <ostream>
#include <fstream>
#include "physics.h"
class World
{
public:
    Viewport view;
    Vec2d panOffset {0,0};
    std::ofstream strm;
    double worldScale = 0.5;
    Robot robot;
    AreaTree tree;
    std::vector<Obstacle> obstacles;    
    bool paused = false;
    bool showObstacle = false;
    std::vector<Vec2d> sensedCoords;
    bool showBeam = false;
    std::vector<Vec2d> newlyDetected;
    std::vector<Vec2d> alreadyDetected;
    std::string incomingData;
    double olda = 0;
    double oldb = 0;
    double leftEnc;
    double rightEnc;
    bool gotFirstReading = false;
    bool drawBoundaries = true;
    physics phys;
    std::vector<Vec2d> path;
    bool navigated = false;
    Vec2d destination{0,0}; //will fuck up if first move is to 0,0
public:
    void save(std::ostream& strm);
    void save(std::string filename);
    void load(std::istream& strm);
    void load(std::string filename);
    World(Vec2d botStart,std::vector<Vec2d> obstaclePoints, double width, double height);
    void draw(mssm::Graphics&g);
    void update();
    void dataInterp(std::string data);
    void sensorCoords();
    void createObstacles(mssm::Graphics&g);
    void makeNewArea();
    bool followPath(mssm::Graphics&g);
    bool hasWorldChanged(); //implement this
    void callNav(mssm::Graphics&g, Vec2d dest);
    bool masterNav(Vec2d dest,mssm::Graphics&g); //
};
#endif // WORLD_H
