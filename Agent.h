#ifndef AGENT_H
#define AGENT_H

#include "Includes.h"
#include "Dubins.h"

using namespace std;

#define WHEELBASE 2.665 
#define MINRADIUS 6.275
#define MAXSTEER asin(WHEELBASE/MINRADIUS)

//width = 2.057 m
//length = 4.1610 m
//height = 1.135 m

class AgentController{
  public:
    AgentController(size_t _velocity=1, double _wheelbase = WHEELBASE, double _minRadius = MINRADIUS, string _name ="DubinAgent");

    bool Update();
    void SetState(AgentState _start){m_myState = _start;}
    void SetGoal(AgentState _goal);
    void SetVelocity(size_t _velocity){m_velocity = _velocity;}

    AgentState GetState(){return m_myState;}
    AgentState GetGoal(){return m_goal;}
    size_t GetVelocity(){return m_velocity;}


  private:
    std::string m_name;
    AgentState m_myState;
    DubinsTrajectory m_nextTrajectory;
    AgentState m_goal;

    //dynamics properties
    double  m_wheelbase;
    double  m_minRadius;
    size_t m_velocity;
};

#endif
