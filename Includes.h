#ifndef INCLUDES_H
#define INCLUDES_H

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <ctime>
#include <stdlib.h>
#include <set>
#include "Circle.h"

#define PI 3.14159265

//DELTA defines the length of a timestep for Eulerian integration. If you find that your solution is too far away from
//the query, you may want to make DELTA smaller
#define DELTA 0.05

using namespace std;

typedef std::pair<double,double> Point2d;

//////////////////////////////////////
//////////////HEADERS/////////////////
//////////////////////////////////////
double RandDouble();

double Norm(const Point2d& _lhs, const Point2d& _rhs);

//calculates outer and inner tangent lines 
//first two results are the other tangents
//if second results exist, they are the inner tangents
vector<pair<Point2d,Point2d> > TangentLines(Circle _c1, Circle _c2);

//bool left indicates a circle direction
//If lhs must turn through a negative angle and the circle direction is left, then we actually want to
//turn through the larger angle
//similarly if lhs must turn through a positive angle and the circle direction is right, then we actually want
//to turn through the larger angle
double ArcLength(const Point2d& _center, const Point2d& _lhs, const Point2d& _rhs, double _radius, bool left);


//Classes and functions for creating and using paths
class Control{
  public:
    double steeringAngle;
    double timesteps;
    Control(){
      steeringAngle=0.0;
      timesteps=0.0;
    }
};

enum TrajectoryType{ //for Dubin's curves
  LRL=0,
  RLR,
  LSL,
  LSR,
  RSL,
  RSR
};

class DubinsTrajectory{
  public:
    TrajectoryType type;
    vector<Control> controls;
    double length; //path metric
    DubinsTrajectory(){
      type = RSR;
      length = 1e9;
    }
};

class CompareTrajectories{
  public:
    bool operator()(const DubinsTrajectory& _lhs, const DubinsTrajectory& _rhs){
      return _lhs.length < _rhs.length;
    }
};


class AgentState{
  public:
    AgentState(){
      pos = make_pair(0.0,0.0);
      theta = 0.0;
    }//end AgentState constructor

    //print (X, Y, Theta)
    friend ostream& operator << (ostream& fout, const AgentState& _rhs){
      fout << "(" << _rhs.pos.first << ", " << _rhs.pos.second << ", " << _rhs.theta << ")" << endl;
      return fout;
    }

    Point2d pos;
    double theta;
};

//xMax and yMax define the size of the environment. Queries will be between 0 and xMax, 0 and yMax, 0 and 2PI
AgentState RandomQuery(double xMax=50.0, double yMax=50.0);
#endif
