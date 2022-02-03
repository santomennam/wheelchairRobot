#ifndef ROBOT_H
#define ROBOT_H
#include "graphics.h"
#include "obstacle.h"
#include "physics.h"

class Robot
{
public:
    double angle = 0; //RADIANS
    Vec2d velocity;
    double speed;//t to turn, move to 0,0, rotate, and then move back out. in draw, connect with lines. use desmoms to find it, start in the center when drawing
    std::vector<Vec2d> measuredPoints;
public:
    Robot(Vec2d startPosition);
    double length;
    bool moved = false;
    double distanceRead = 0;
    double width = 100;
    Vec2d position; // pissy pants
    double angularVelocity = 0;
    std::vector<Vec2d> pointsToDraw{{-40,20},{-40,-20},{40,-20},{40,20}}; // put a vector of points here. when wan
    void navigation();
    void measureStepOne();
    void turnLeft(double degrees);
    void turnRight(double degrees);
    Vec2d navPoint();
    void forward(double speed);
    void update();
    double measureStep2(std::vector<Obstacle> obstacles); //should this go in world? i want robot to be independent enough to do this by itself
    void actuateTurn(double degrees);
    void power(double left, double right);

};


#endif // ROBOT_H
