#include "Includes.h"
#ifndef DUBINS_H
#define DUBINS_H

class Dubins{
  public:
    //calculate shortest trajectory to get to _query configuration
    DubinsTrajectory DubinsShortestPath(const double& _minTurnRadius, const double& _wheelbase, const AgentState& _start, const AgentState& _goal);

  private:
    DubinsTrajectory BestCSCTrajectory(const Circle& _agentLeft, const Circle& _agentRight, 
        const Circle& _queryLeft, const Circle& _queryRight);

    DubinsTrajectory BestCCCTrajectory(const Circle& _agentLeft, const Circle& _agentRight, 
        const Circle& _queryLeft, const Circle& _queryRight);

    DubinsTrajectory RSRTrajectory(vector<pair<Point2d, Point2d> >& _RRTangents, const Circle& _agentRight,
        const Circle& _queryRight);

    DubinsTrajectory LSLTrajectory(vector<pair<Point2d, Point2d> >& _LLTangents, const Circle& _agentLeft,
        const Circle& _queryLeft);

    DubinsTrajectory RSLTrajectory(vector<pair<Point2d, Point2d> >& _RLTangents, const Circle& _agentRight,
        const Circle& _queryLeft);

    DubinsTrajectory LSRTrajectory(vector<pair<Point2d, Point2d> >& _LRTangents, const Circle& _agentLeft,
        const Circle& _queryRight);

    //interior Angle is the relative angle C3 (the third circle to turn about) is from _agentRight
    DubinsTrajectory RLRTrajectory(const double& _interiorTheta, const Circle& _agentRight,
        const Circle& _queryRight);

    //interior Angle is the relative angle C3 (the third circle to turn about) is from _agentRight
    DubinsTrajectory LRLTrajectory(const double& _interiorTheta, const Circle& _agentLeft,
        const Circle& _queryLeft);

    AgentState m_start, m_goal;
    double m_maxSteering;
    double m_minTurnRadius;
};
#endif

