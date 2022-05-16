#include "world.h"
#include <iostream>
#include <algorithm>
#include <ostream>
#include "simplecrc.h"

using namespace std;
using namespace mssm;

using tp = std::chrono::time_point<std::chrono::steady_clock>;

void World::botTargetUpdated(Vec2d targets)
{

}

void World::botEncoderUpdated(Vec2d encoders)
{
    //    dataStream>>robot.distanceRead>>a>>b>>vel1>>vel2; // uwu
    //    robot.distanceRead /= 2.54;
    //    robot.distanceRead += 8; //this will need to be removed
    posTracker.update(encoders.x, encoders.y);
    robot.angle = posTracker.getAngle();
    robot.position = posTracker.getPos();
}

void World::botMotorsUpdated(Vec2d motors)
{

}

int World::encoders(double distance)
{
    return int(distance * RobotParams::countsPerRev / RobotParams::encWheelCirc());
}

// takes encoder units and returns inches
double World::inches(int encs)
{
    return (RobotParams::encWheelCirc() * encs / RobotParams::countsPerRev);
}


// float degrees(double rads)
// {
//   return rads * 57.2958; //conversion factor
// }

float World::rads(int degrees)
{
    return degrees / 57.2958;
}

float World::radToArc(float rads)
{ // returns arc in inches
    return RobotParams::encSeparation/ 2 * rads;
}

Vec2d World::generateTurn(int degrees)
{
    int sign = degrees < 0 ? -1 : 1;
    // positive turn is counterclockwise
    degrees = sign * (abs(degrees) % 360); // constrain to +/- 360
    if (abs(degrees) > 180)
    {
        degrees = (360 - abs(degrees)) * -sign; // find complement, flip sign
    }
    float angle = rads(degrees);
    float arc = radToArc(angle);
    double dist = encoders(arc);
    Vec2d motorVals(-dist, dist);
    return motorVals;
}

double angle(Vec2d p1, Vec2d p2) //angle between these two rays
{
    return atan2(crossProduct(p1,p2),p1*p2);
}

double World::distanceToPoint(Vec2d p, Vec2d src)
{
    return Vec2d{p-src}.magnitude();
}

navPoint World::encsForTurn(double currentAngle, Vec2d inchPos, Vec2d inchDest) //returns encoder offset when turning between two points that are in inches units
{
    //calculate how our encoders will change
    Vec2d dir1 = Vec2d{1,0}.rotated(currentAngle);
    Vec2d dir2 = (inchDest-inchPos).unit();
    double dangle = angle(dir1,dir2);
    Vec2d angularEncTarget = generateTurn(dangle);
    navPoint destPoint{targets.empty() ? posTracker.position : targets.back().pos,currentAngle+dangle,angularEncTarget,true};
    //    cout<<"Encs for turn from "<<inchPos<<" to " << inchDest << " of angle " << dangle << " is " <<destPoint.encoderReadings<<endl;
    return destPoint;
}


void World::updateTargets()
{
    if(targets.size())
    {
        if(posTracker.encoderReadings.equals(targets[0].encoderReadings,acceptableError))
        {
            cout<<"Pos "<<posTracker.position<<", I think I've reached "<<targets[0].pos<<endl;
            cout<<"Targets: ";
            for(auto targ : targets)
            {
                cout<<targ.pos;
            }
            cout<<endl;
            cout<<"Reached" <<targets[0].pos<<endl;
            targets.erase(targets.begin());
            targetsChanged = true;
        }
    }
}
void World::navToPoint(Vec2d start, Vec2d dest, double currentAngle)
{
    navPoint turnPoint = encsForTurn(currentAngle,start,dest);
    cout<<"turn pos: " <<turnPoint.pos<<endl;
    turnPoint.encoderReadings = turnPoint.encoderReadings + (targets.empty() ? posTracker.encoderReadings : targets.back().encoderReadings);
    targets.push_back(turnPoint);
    Vec2d finalEncs = turnPoint.encoderReadings + Vec2d{encoders(distanceToPoint(turnPoint.pos,dest)),encoders(distanceToPoint(turnPoint.pos,dest))};
    navPoint final{dest,turnPoint.angle,finalEncs};
    targets.push_back(final);
}

void World::findEncPath(std::vector<Vec2d> path) //pass a path of inches coordinates to navigate
{
    targets = {};
    anglesAfterWaypoints = {};
    if(path.size())
    {
        navToPoint(posTracker.position,path[0],posTracker.angle);
    }
    else{
        cout<<"findEncPath was passed an empty path!"<<endl;
    }
    for(int i = 0; !path.empty() && i < path.size()-1; i++)
    {
        navToPoint(path[i],path[i+1],targets[i].angle);
    }
}


void World::save(ostream& strm)
{
    tree.save(strm);
}

void World::save(string filename)
{
    ofstream stream {filename};
    save(stream);
}

void World::load(istream &strm)
{
    tree.load(strm);
}

void World::load(string filename)
{
    ifstream stream {filename, std::ifstream::in};
    load(stream);
}

World::World(Vec2d botStart, vector<Vec2d> obstaclePoints, double width, double height)
    :robot(botStart), tree(width,height,robot.width)
{
    strm = ofstream("writeNode.txt");
    // obstacles.emplace_back(obstaclePoints);
}

void World::draw(Graphics &g)
{if(drawBoundaries){
        tree.draw(g,view);
    }
    if(diagnostics){
        g.text({50,70}, 20, "x pos " + to_string(robot.position.x));
        g.text({50,50}, 20, "y pos " + to_string(robot.position.y));
        g.text({50,90}, 20, "angle " + to_string(robot.angle));
    }

    robot.update();
    sensorCoords();
 //   processLidarData(g);
    //    if(robot.moved)
    //    {

    //        robot.moved = false;
    //    }

    vector<Vec2d> tempPoints = robot.pointsToDraw;
    for (Vec2d& temp:tempPoints)
    {
        temp.rotate(robot.angle);
        temp.x += robot.position.x;
        temp.y += robot.position.y;
    }


    for(auto p : polyOp.polys)
    {
        for(auto &point : p)
        {
            point = view.worldToScreen(point);
        }
        g.polygon(p,YELLOW);
    }

    for(navPoint nav : posTracker.navPoints)
    {
        Vec2d head = nav.pos + Vec2d{9,0}.rotated(nav.angle);//second point in arrow
        Vec2d t1 = head+ Vec2d{3,0}.rotated(nav.angle+2.87979); //165 deg
        Vec2d t2 = head+ Vec2d{3,0}.rotated(nav.angle-2.87979);
        vector<Vec2d> arrowPoints = {nav.pos,head,t1,head,t2};
        for(auto& i : arrowPoints)
        {
            i = view.worldToScreen(i);
        }
        g.polyline(arrowPoints,{50,50+posTracker.navPoints.size(),150});
    }
    if(showBeam){
        //for(int i = 0; i<world.robot.measuredPoints.size(); i++)
        //{
        vector<Vec2d> copy = robot.measuredPoints;
        for(Vec2d& n: copy)
        {
            n = view.worldToScreen(n);
        }
        g.polyline(copy,WHITE);
        //}
    }

    if(showObstacle)
    {
        for(int i = 0; i<obstacles.size();i++){
            for(int j =0; j<obstacles[i].pts.size();j++)
            {
                g.point(view.worldToScreen(obstacles[i].pts[j]), RED);
            }
        }
    }
    vector<Vec2d> copyRobot = tempPoints;
    for(Vec2d& n:copyRobot)
    {
        n = view.worldToScreen(n);
    }
    g.polygon(copyRobot,WHITE);
    //g.polyline(obstacle.pts,RED);
//    for(int i = 0; i<newlyDetected.size();i++)
//    {
//        g.point(view.worldToScreen(newlyDetected[i]),BLUE);
//    }
//    for(int i = 0; i<alreadyDetected.size();i++)
//    {
//        g.point(view.worldToScreen(alreadyDetected[i]),GREEN);
//    }
}



void World::sensorCoords()
{
//    //  cout << "ND: " << newlyDetected.size() << endl;
//    for(auto& point : newlyDetected)
//    {
//        point = point*(1/25.4); //convert from mm to in
//        point.rotate(posTracker.getAngle());
//        point = point + posTracker.getPos(); // take the rectangular lidar points, centered around the origin, and center them about the robot
//        //  sensedCoords.push_back(point);
//    }
//    // cout << "Sensed: " << sensedCoords.size() << endl;
//    //clear newlyDetected?
}

void World::createRandomObstacles(Graphics &g)
{
//    int q = g.randomInt(3,10);
//    for(int i = 0; i <q;i++)
//    {
//        newlyDetected.push_back({g.randomDouble(100,500),g.randomDouble(0,500)});

//        //        Obstacle obstacle({{g.randomDouble(100,500),g.randomDouble(0,500)}});
//        //        //,g.randomDouble(100,500)},{g.randomDouble(100,500),g.randomDouble(100,500)},{g.randomDouble(100,500),g.randomDouble(100,500)},{g.randomDouble(100,500),g.randomDouble(100,500)},{g.randomDouble(100,500),g.randomDouble(100,500)/*}});
//        //        obstacle.interpolate();
//        //        obstacles.push_back(obstacle);

//    }

}

void World::placeObstacle(Vec2d point)
{
    Obstacle obstacle({point});
    obstacles.push_back(obstacle);
}

//void World::placeObstaclesFromList(std::vector<Vec2d> points)
//{
//    //newlyDetected = points;
//}

double triangleAngle(vector<Vec2d>triangle) //finds interior angle from the first point in the list to the other two. expects three points.
{
    if(triangle.size() != 3)
    {
        throw new logic_error("Tried to call triangle angle on a shape with num points != 3!");
    }
    //does law of cosines
    double a = segment{triangle[1],triangle[2],30}.magnitude();
    double b = segment{triangle[0],triangle[2],30}.magnitude();
    double c = segment{triangle[0],triangle[1],30}.magnitude();
    return qRadiansToDegrees(acos((pow(b,2)+pow(c,2)-pow(a,2))/(2*b*c)));
}

double angleDiffSpecial(double prev, double curr)
{
    if (curr < prev) {
        // wrapped from 2pi back to zero?
        curr += M_PI*2;
    }
    return curr - prev;
}

void giveStripThickness(Vec2d scanCenter, vector<Vec2d>& strip, double thickness)
{
    for (int i = strip.size()-1; i >= 0; i--) {
        Vec2d& p = strip[i];
        Vec2d u = (p - scanCenter).unit();
        strip.push_back(p + u * thickness);
    }
}

void World::processLidarData(Graphics& g)
{

    if (newLidar.size() > 400) {
        std::vector<LidarPoint> newLidarTmp;
        bool gotStart = false;

        for (int i = 0; i < newLidar.size(); i++) {
            if (i > 0 && newLidar[i-1].worldAngle > 1.5*M_PI && newLidar[i].worldAngle < 0.5*M_PI) {
                // wrapped!
                if (!gotStart) {
                    gotStart = true;
                }
                else {
                    newLidarTmp.push_back(newLidar[i]);
                    break;
                }
            }
            if (gotStart) {
                newLidarTmp.push_back(newLidar[i]);
            }
        }
        cout << "Discarding lidar data. Was: " << newLidar.size() << " Keeping: " << newLidarTmp.size() << endl;
        newLidar = newLidarTmp;
    }

    vector<Vec2d> wedge;
    vector<Vec2d> obstPts;
    vector<Vec2d> strip;
    vector<vector<Vec2d>> strips;

    for (auto& pnt : newLidar)
    {
        bool   lastValid = lastLidarPoint.valid;
        bool   thisValid = pnt.valid;
        double angleDiff = angleDiffSpecial(lastLidarPoint.worldAngle, pnt.worldAngle);
        bool   angleValid = angleDiff < 1;
        bool   breakStrip = false;

        if (thisValid && !strip.empty() && (abs(pnt.distance - lastLidarPoint.distance) > 4)) {
            breakStrip = true;
        }

        if (thisValid && !breakStrip) {
            strip.push_back(pnt.worldPos);
        }
        else {
            if (strip.size() > 0) {
                strips.push_back(move(strip));
                strip.clear();
            }
            if (thisValid) {
                strip.push_back(pnt.worldPos);
            }
        }

        if (lastValid && thisValid && angleValid) {
            // create or append to wedge
            if (wedge.empty()) {
                wedge.push_back(posTracker.position);
                wedge.push_back(lastLidarPoint.worldPos);
            }
            wedge.push_back(pnt.worldPos);
        }
        else if (wedge.size() > 0) {
            // finished this wedge, process it
            polyOp.polys = polyOp.clip(polyOp.polys,wedge,ClipperLib::ClipType::ctDifference);
            g.polygon(view.worldToScreen(wedge),PURPLE,Color(255,0,255,100));
            wedge.clear();
        }

        lastLidarPoint = pnt;

        if (thisValid) {
            obstPts.push_back(pnt.worldPos);
        }
    }

    newLidar.clear();

    bool toggleColor = true;
    for (auto& s : strips) {
        if (s.size() > 1) {
            giveStripThickness(posTracker.position, s, 5);
            polyOp.polys = polyOp.clip(polyOp.polys,s,ClipperLib::ClipType::ctUnion);
           // polyOp.polys = polyOp.simplify(polyOp.polys);
            g.polygon(view.worldToScreen(s), toggleColor ? YELLOW : GREEN, toggleColor ? YELLOW : GREEN);
            toggleColor = !toggleColor;
        }
        else {
            g.point(view.worldToScreen(s[0]), CYAN);
        }
    }


//    for (auto& p : obstPts) {
//        vector<Vec2d> poly = polyOp.makeCircle(obstacleRadius,8, p);
//        polyOp.polys.push_back(poly);
//        polyOp.polys = polyOp.clip(polyOp.polys,poly,ClipperLib::ClipType::ctUnion);
//        polyOp.polys = polyOp.simplify(polyOp.polys);

//    }

    for(const auto& poly : polyOp.polys) {
        for (Vec2d p : poly)
        {
            tree.closeNodeByPoint(p);
        }
    }

}

bool World::followPath(Graphics&g)
{
    if(!path.size())
    {
        return true;
    }
    if(phys.goTo(path[0],g))
    {
        path.erase(path.begin());
    }
    return false;
}

bool World::hasWorldChanged()
{
    return false; //change!!!!!!!!!!!
}

void World::callNav(mssm::Graphics&g, Vec2d dest)
{
    if(hasWorldChanged())
    {
        cout<<"recalculating in callNav"<<endl;
        path = tree.navigation(phys.position,dest,g,view);
    }
}

bool World::masterNav(Vec2d dest,mssm::Graphics&g)
{
    // cout<<"power in master: left: "<<phys.leftPower<<" right: "<<phys.rightPower<<endl;
    if(!(destination == dest))
    {
        navigated = false;
        cout<<"dest changed"<<endl;
        destination = dest;
    }
    if(!navigated)
    {
        cout<<"recalculating in masterNav"<<endl;
        path = tree.navigation(phys.position,dest,g,view);
        navigated = true;
    }
    callNav(g,dest);
    return(followPath(g));
}

double Robot::measureStep2(vector<Obstacle>obstacles) // do it here
{
    if(distanceRead < 200/2.54)
    {
        return distanceRead;
    }
    return(-1.0);
}


