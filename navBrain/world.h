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
#include "absolutepostracker.h"

using tp = std::chrono::time_point<std::chrono::steady_clock>;

class World
{
public:
    double vel1;
    double vel2;
    Viewport view;
    Vec2d panOffset {0,0};
    Vec2d queriedTarget;
    bool queried = false;
    std::ofstream strm;
    double worldScale = 0.5;
    Robot robot;
    AreaTree tree;
    bool diagnostics = true;
    tp lastTime;
    std::vector<Obstacle> obstacles;    
    bool paused = false;
    bool showObstacle = false;
    std::vector<Vec2d> sensedCoords;
    bool showBeam = false;
    std::vector<Vec2d> newlyDetected;
    std::vector<Vec2d> alreadyDetected;
    std::string incomingData;
    bool timedOut = false;
//    double olda = 0;
//    double oldb = 0;
//    double leftEnc;
//    double rightEnc;
    bool gotFirstReading = false;
    bool drawBoundaries = true;
    physics phys;
    std::vector<Vec2d> path;
    bool navigated = false;
    Vec2d destination{0,0}; //will fuck up if first move is to 0,0
    AbsolutePosTracker posTracker;
    std::vector<double> anglesAfterWaypoints;
    std::vector<navPoint> targets;
    double acceptableError = 600; //in encs //was 1200
    bool targetsChanged = true;
public:
    int encoders(double distance);
    double inches(int encs);
    void updateTargets();
    void navToPoint(Vec2d start, Vec2d dest, double currentAngle);
    void findEncPath(std::vector<Vec2d> path);
    float rads(int degrees);
    float radToArc(float rads);
    Vec2d generateTurn(int degrees);
    double distanceToPoint(Vec2d point, Vec2d closest);
    navPoint encsForTurn(double currentAngle, Vec2d inchPos, Vec2d inchDest); //returns encoder offset when turning between two points that are in inches units
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
