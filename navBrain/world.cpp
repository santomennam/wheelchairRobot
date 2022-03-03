#include "world.h"
#include <iostream>
#include <algorithm>
#include <ostream>
using namespace std;
using namespace mssm;

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
    g.text({50,50}, 20, to_string(robot.position.x));
         g.text({50,70}, 20, to_string(robot.position.y));
         g.text({50,90}, 20, to_string(robot.angle));

    robot.update();
    if(robot.moved)
    {
        sensorCoords();
        makeNewArea();
        robot.moved = false;
    }

    vector<Vec2d> tempPoints = robot.pointsToDraw;
    for (Vec2d& temp:tempPoints)
    {
        temp.rotate(robot.angle);
        temp.x += robot.position.x;
        temp.y += robot.position.y;
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
    g.polygon(copyRobot,WHITE,WHITE);
    //g.polyline(obstacle.pts,RED);
    for(int i = 0; i<newlyDetected.size();i++)
    {
        g.point(view.worldToScreen(newlyDetected[i]),BLUE);
    }
    for(int i = 0; i<alreadyDetected.size();i++)
    {
        g.point(view.worldToScreen(alreadyDetected[i]),GREEN);
    }
}

void World::dataInterp(string data)
{
    std::replace(data.begin(),data.end(),'\r','\n');
    incomingData += data;
    while(true){
        auto i = find(incomingData.begin(),incomingData.end(),'#');
        if(i == incomingData.end())
        {
            return;
        }
        incomingData.erase(incomingData.begin(),i);
        auto j = find(incomingData.begin(),incomingData.end(),'\n');
        if(j == incomingData.end())
        {
            return;
        }
        string useful = incomingData.substr(1,(j-incomingData.begin()));
        incomingData.erase(incomingData.begin(),j);
        stringstream dataStream(useful);
        double a;
        double b;
        dataStream>>robot.distanceRead>>a>>b; // uwu
        robot.distanceRead /= 2.54;
        robot.distanceRead += 8; //this will need to be removed
        posTracker.update(a,b);
        robot.angle = posTracker.getAngle();
        robot.position = posTracker.getPos();
    }
}

void World::sensorCoords()
{
    if(robot.measureStep2(obstacles) > 0) // was >=
    {
        //  cout<<"yes"<<endl;
        Vec2d point(robot.position);
        Vec2d offset{robot.measureStep2(obstacles)*10,0};
        offset.rotate(robot.angle);
        cout<<"offset: "<<offset.magnitude()<<endl;
        point = robot.position + offset;
        //        point.rotate(robot.angle);
        sensedCoords.push_back(point);
        newlyDetected.push_back(point);

        // cout << "ND: " << newlyDetected.size() << endl;
    }
}

void World::createObstacles(Graphics &g)
{
    int q = g.randomInt(3,10);
    for(int i = 0; i <q;i++)
    {
        Obstacle obstacle({{g.randomDouble(100,500),g.randomDouble(0,500)}});
        //,g.randomDouble(100,500)},{g.randomDouble(100,500),g.randomDouble(100,500)},{g.randomDouble(100,500),g.randomDouble(100,500)},{g.randomDouble(100,500),g.randomDouble(100,500)},{g.randomDouble(100,500),g.randomDouble(100,500)/*}});
        obstacle.interpolate();
        obstacles.push_back(obstacle);

    }

}

void World::makeNewArea()
{
    while(newlyDetected.size())
    {
        bool cont = true;
        for(auto p:alreadyDetected)
        {
            if((newlyDetected[0]-p).magnitude()<3)
            {
                // cout<<"too close, deleting"<<endl;
                newlyDetected.erase(newlyDetected.begin());
                cont = false;
                break;
            }

        }
        if(cont){
            //cout<<"new points, making area"<<endl;
            alreadyDetected.push_back(newlyDetected[0]);
            newlyDetected.erase(newlyDetected.begin());
            cout<<"x: "<<alreadyDetected.back().x<< " y: "<<alreadyDetected.back().y<<endl;
            cout<<"position x: "<<robot.position.x<<" position y: "<<robot.position.y<<endl;
            strm.close();
            strm.open("writeNode.txt");
            tree.visitStuff(strm);
            tree.placer(alreadyDetected.back());
        }
    }
    //make area


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


