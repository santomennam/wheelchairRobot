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
#include "polyops.h"
#include "Lidar/lidar.h"

using tp = std::chrono::time_point<std::chrono::steady_clock>;

class LidarPoint {
public:
    bool   valid;
    double distance;
    double localAngle;
    double worldAngle;
    Vec2d  worldPos;
};

class World
{
public:
    PolyOps polyOp;
    double vel1;
    double vel2;
    Viewport view;
    Vec2d panOffset {0,0};

    std::ofstream strm;
    double worldScale = 0.5;
    Robot robot;
    AreaTree tree;
    bool diagnostics = true;

    std::vector<Obstacle> obstacles;    
    bool paused = false;
    bool showObstacle = true;
    std::vector<Vec2d> sensedCoords;
    bool showBeam = false;
    //std::vector<Vec2d> newlyDetected;
    std::vector<Vec2d> alreadyDetected;

    std::vector<LidarPoint> newLidar;
    LidarPoint lastLidarPoint;

//    std::string incomingData;
//    std::string receivedCommand;
//    std::string receivedError;
//    std::string receivedInfo;

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
    int obstacleRadius = 5; //inches around the detected point to build the polygon

public:

    // used by BotConnection:
    void botTargetUpdated(Vec2d targets);
    void botEncoderUpdated(Vec2d encoders);
    void botMotorsUpdated(Vec2d motors);

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
    void sensorCoords();
    void createRandomObstacles(mssm::Graphics&g);
    void placeObstacle(Vec2d point);
   // void placeObstaclesFromList(std::vector<Vec2d> list);
    void addLidarPoint(LidarPoint& pnt) { newLidar.push_back(pnt); }
    int numNewLidarPoint() { return newLidar.size(); }
    void processLidarData(mssm::Graphics& g);
    bool followPath(mssm::Graphics&g);
    bool hasWorldChanged(); //implement this
    void callNav(mssm::Graphics&g, Vec2d dest);
    bool masterNav(Vec2d dest,mssm::Graphics&g); //
};
#endif // WORLD_H
