#include "Dubins.h"
DubinsTrajectory
Dubins::DubinsShortestPath(const double& _minTurnRadius, const double& _wheelbase, const AgentState& _start, const AgentState& _goal){
  Circle agentLeft;
  Circle agentRight;
  Circle queryLeft;
  Circle queryRight;
  m_start = _start;
  m_goal = _goal;
  m_maxSteering = asin(_wheelbase/_minTurnRadius),
  m_minTurnRadius = _minTurnRadius;

  double theta = m_start.theta;
  theta += PI/2.0;
  if (theta > PI)
    theta -= 2.0*PI;

  agentLeft.SetPos(m_start.pos.first + m_minTurnRadius*cos(theta), m_start.pos.second + m_minTurnRadius*sin(theta));
  agentLeft.SetRadius(m_minTurnRadius);

  theta = m_start.theta;
  theta -= PI/2.0;
  if (theta < -PI)
    theta += 2.0*PI;
  agentRight.SetPos(m_start.pos.first + m_minTurnRadius*cos(theta), m_start.pos.second + m_minTurnRadius*sin(theta));
  agentRight.SetRadius(m_minTurnRadius);

  theta = m_goal.theta;
  theta += PI/2.0;
  if (theta > PI)
    theta -= 2.0*PI;

  queryLeft.SetPos(m_goal.pos.first + m_minTurnRadius*cos(theta), m_goal.pos.second + m_minTurnRadius*sin(theta));
  queryLeft.SetRadius(m_minTurnRadius);

  theta = m_goal.theta;
  theta -= PI/2.0;
  if (theta < -PI)
    theta += 2.0*PI;

  queryRight.SetPos(m_goal.pos.first + m_minTurnRadius*cos(theta), m_goal.pos.second + m_minTurnRadius*sin(theta));
  queryRight.SetRadius(m_minTurnRadius);


  DubinsTrajectory shortest, next;
  next = BestCSCTrajectory(agentLeft,agentRight,queryLeft,queryRight);
  if (next.length < shortest.length)
    shortest = next;

  next = BestCCCTrajectory(agentLeft,agentRight,queryLeft,queryRight);
  if(next.length < shortest.length)
    shortest = next;


  cout << "To reach query point: " << m_goal << " from: " << m_start << " Agent chose ";
  switch(shortest.type){
    case RSR:
      cout << "RSR";
      break;
    case RSL:
      cout << "RSL";
      break;
    case LSL:
      cout << "LSL";
      break;
    case LSR:
      cout << "LSR";
      break;
    case RLR:
      cout << "RLR";
      break;
    case LRL:
      cout << "LRL";
      break;
    default: 
      cout << "???";
  }//end switch
  cout << " trajectory with path length: " << shortest.length << endl;
  return shortest;
}//end DubinShortestPath function

DubinsTrajectory 
Dubins::BestCCCTrajectory(const Circle& _agentLeft, const Circle& _agentRight, 
    const Circle& _queryLeft, const Circle& _queryRight){
  DubinsTrajectory shortest, next;

  //find the relative angle for L and right
  double theta  = 0.0;
  double D = Norm(_agentRight.GetPos(),_queryRight.GetPos());
  ////////////////////////////////////////////////////
  /////////////////calculate RLR//////////////////////
  ////////////////////////////////////////////////////
  if (D < 4.0*m_minTurnRadius){
    theta = acos(D/(4.0*m_minTurnRadius));

    theta += atan2(_queryRight.GetY() - _agentRight.GetY(),  _queryRight.GetX() -
        _agentRight.GetX());

    next = RLRTrajectory(theta, _agentRight, _queryRight);
    if (next.length < shortest.length)
      shortest = next;
  }

  ////////////////////////////////////////////////////
  /////////////////calculate LRL//////////////////////
  ////////////////////////////////////////////////////
  D = Norm(_agentLeft.GetPos(),_queryLeft.GetPos());
  if (D < 4.0*m_minTurnRadius){
    theta = acos(D/(4.0*m_minTurnRadius));

    theta = atan2(_queryLeft.GetY() - _agentLeft.GetY(),  _queryLeft.GetX() -
        _agentLeft.GetX()) - theta;

    next = LRLTrajectory(theta, _agentLeft, _queryLeft);
    if (next.length < shortest.length)
      shortest = next;
  }
  return shortest;
}

DubinsTrajectory 
Dubins::BestCSCTrajectory(const Circle& _agentLeft, const Circle& _agentRight, 
    const Circle& _queryLeft, const Circle& _queryRight){

  vector<pair<Point2d,Point2d> > RRTangents = TangentLines(_agentRight,_queryRight);
  vector<pair<Point2d,Point2d> > LLTangents = TangentLines(_agentLeft,_queryLeft);
  vector<pair<Point2d,Point2d> > RLTangents = TangentLines(_agentRight,_queryLeft);
  vector<pair<Point2d,Point2d> > LRTangents = TangentLines(_agentLeft,_queryRight);

  DubinsTrajectory shortest, next;

  ////////////////////////////////////////////////////
  /////////////////calculate RSR//////////////////////
  ////////////////////////////////////////////////////
  next = RSRTrajectory(RRTangents, _agentRight, _queryRight);
  if (next.length < shortest.length)
    shortest = next;

  ////////////////////////////////////////////////////
  /////////////////calculate LSL//////////////////////
  ////////////////////////////////////////////////////
  next = LSLTrajectory(LLTangents, _agentLeft, _queryLeft);
  if (next.length < shortest.length)
    shortest = next;

  ////////////////////////////////////////////////////
  /////////////////calculate RSL//////////////////////
  ////////////////////////////////////////////////////
  next = RSLTrajectory(RLTangents, _agentRight, _queryLeft);
  if (next.length < shortest.length)
    shortest = next;
  ////////////////////////////////////////////////////
  /////////////////calculate LSR//////////////////////
  ////////////////////////////////////////////////////
  next = LSRTrajectory(LRTangents, _agentLeft, _queryRight);
  if (next.length < shortest.length)
    shortest = next;

  return shortest;

}//end calculation of best CSC trajectory

DubinsTrajectory
Dubins::LRLTrajectory(const double& _interiorTheta, const Circle& _agentLeft,
    const Circle& _queryLeft){
  DubinsTrajectory next;
  next.type = LRL;
  double arcL1, arcL2, arcL3; //arcLengths
  Control nextControl; //for a control vector
  Point2d agentTan, queryTan;
  Circle rCircle;
  rCircle.SetRadius(m_minTurnRadius);

  //compute tangent circle's pos using law of cosines + atan2 of line between agent and query circles
  rCircle.SetPos(_agentLeft.GetX() + (2.0*m_minTurnRadius*cos(_interiorTheta)), _agentLeft.GetY() +
      (2.0*m_minTurnRadius*sin(_interiorTheta)));

  //compute tangent points given tangent circle
  agentTan.first = (rCircle.GetX() + _agentLeft.GetX())/2.0; 
  agentTan.second = (rCircle.GetY() + _agentLeft.GetY())/2.0;

  queryTan.first = (rCircle.GetX() + _queryLeft.GetX())/2.0;
  queryTan.second= (rCircle.GetY() + _queryLeft.GetY())/2.0;

  nextControl.steeringAngle = m_maxSteering; //left turn at max
  arcL1 = ArcLength(_agentLeft.GetPos(), m_start.pos, agentTan, m_minTurnRadius, true);

  //don't use velocities because Dubins assumes unit forward velocity
  nextControl.timesteps = arcL1 / DELTA;
  next.controls.push_back(nextControl);

  nextControl.steeringAngle = -1.0 * m_maxSteering; //right turn at max
  arcL2 = ArcLength(rCircle.GetPos(), agentTan, queryTan, m_minTurnRadius, false);
  nextControl.timesteps = arcL2 / DELTA;
  next.controls.push_back(nextControl);

  nextControl.steeringAngle = m_maxSteering; //left turn at max
  arcL3 = ArcLength(_queryLeft.GetPos(), queryTan, m_goal.pos, m_minTurnRadius, true);
  nextControl.timesteps = arcL3 / DELTA;
  next.controls.push_back(nextControl);

  //calculate total length
  next.length =  arcL1 + arcL2 + arcL3;
  return next;
}//end LRLTrajectory

DubinsTrajectory
Dubins::RLRTrajectory(const double& _interiorTheta, const Circle& _agentRight,
    const Circle& _queryRight){
  DubinsTrajectory next;
  next.type = RLR;
  double arcL1, arcL2, arcL3; //arcLengths
  Control nextControl; //for a control vector
  Point2d agentTan, queryTan;
  Circle lCircle;
  lCircle.SetRadius(m_minTurnRadius);

  //compute tangent circle's pos using law of cosines + atan2 of line between agent and query circles
  lCircle.SetPos(_agentRight.GetX() + (2.0*m_minTurnRadius*cos(_interiorTheta)), _agentRight.GetY() +
      (2.0*m_minTurnRadius*sin(_interiorTheta)));

  //compute tangent points given tangent circle
  agentTan.first = (lCircle.GetX() + _agentRight.GetX())/2.0; 
  agentTan.second = (lCircle.GetY() + _agentRight.GetY())/2.0;

  queryTan.first = (lCircle.GetX() + _queryRight.GetX())/2.0;
  queryTan.second= (lCircle.GetY() + _queryRight.GetY())/2.0;


  nextControl.steeringAngle = -1.0 * m_maxSteering; //right turn at max
  arcL1 = ArcLength(_agentRight.GetPos(), m_start.pos, agentTan, m_minTurnRadius, false);

  //don't use velocities because Dubins assumes unit forward velocity
  nextControl.timesteps = arcL1 / DELTA;
  next.controls.push_back(nextControl);

  nextControl.steeringAngle = m_maxSteering; //left turn at max
  arcL2 = ArcLength(lCircle.GetPos(), agentTan, queryTan, m_minTurnRadius, true);
  nextControl.timesteps = arcL2 / DELTA;
  next.controls.push_back(nextControl);

  nextControl.steeringAngle =  -1.0 * m_maxSteering; //right turn at max
  arcL3 = ArcLength(_queryRight.GetPos(), queryTan, m_goal.pos, m_minTurnRadius, false);
  nextControl.timesteps = arcL3 / DELTA;
  next.controls.push_back(nextControl);

  //calculate total length
  next.length =  arcL1 + arcL2 + arcL3;
  return next;
}//end RLRTrajectory

DubinsTrajectory
Dubins::RSRTrajectory(vector<pair<Point2d, Point2d> >& _RRTangents, const Circle& _agentRight,
    const Circle& _queryRight){
  DubinsTrajectory next;
  next.type = RSR;
  double arcL1, arcL2, arcL3; //arcLengths
  Control nextControl; //for a control vector
  if (_RRTangents.size() > 0){
    //tangent pts function returns outer tangents for RR connection first
    nextControl.steeringAngle = -1.0 * m_maxSteering; //right turn at max
    arcL1 = ArcLength(_agentRight.GetPos(), m_start.pos, _RRTangents.at(0).first, m_minTurnRadius, false);
    //don't use velocities because Dubins assumes unit forward velocity
    nextControl.timesteps = arcL1 / DELTA;
    next.controls.push_back(nextControl);

    nextControl.steeringAngle = 0.0; //straight
    arcL2 = Norm(_RRTangents.at(0).first, _RRTangents.at(0).second);
    nextControl.timesteps = arcL2 / DELTA;
    next.controls.push_back(nextControl);

    nextControl.steeringAngle = -1.0 * m_maxSteering; //right turn at max
    arcL3 = ArcLength(_queryRight.GetPos(),_RRTangents.at(0).second, m_goal.pos, m_minTurnRadius, false);
    nextControl.timesteps = arcL3 / DELTA;
    next.controls.push_back(nextControl);

    //calculate total length
    next.length =  arcL1 + arcL2 + arcL3;
  }
  return next;
}//end RSRTrajectory

DubinsTrajectory
Dubins::LSLTrajectory(vector<pair<Point2d, Point2d> >& _LLTangents, const Circle& _agentLeft,
    const Circle& _queryLeft){
  DubinsTrajectory next;
  next.type = LSL;
  double arcL1, arcL2, arcL3; //arcLengths
  Control nextControl; //for a control vector
  if (_LLTangents.size() > 1){
    //tangent pts function returns outer tangents for LL connection second
    nextControl.steeringAngle = m_maxSteering; //left turn at max
    arcL1 = ArcLength(_agentLeft.GetPos(), m_start.pos, _LLTangents.at(1).first, m_minTurnRadius, true);
    //don't use velocities because Dubins assumes unit forward velocity
    nextControl.timesteps = arcL1 / DELTA;
    next.controls.push_back(nextControl);

    nextControl.steeringAngle = 0.0; //straight
    arcL2 = Norm(_LLTangents.at(1).first, _LLTangents.at(1).second);
    nextControl.timesteps = arcL2 / DELTA;
    next.controls.push_back(nextControl);

    nextControl.steeringAngle = m_maxSteering; //left turn at max
    arcL3 = ArcLength(_queryLeft.GetPos(),_LLTangents.at(1).second, m_goal.pos, m_minTurnRadius, true);
    nextControl.timesteps = arcL3 / DELTA;
    next.controls.push_back(nextControl);

    //calculate total length
    next.length =  arcL1 + arcL2 + arcL3;
  }
  return next;
}//end LSLTrajectory

DubinsTrajectory
Dubins::RSLTrajectory(vector<pair<Point2d, Point2d> >& _RLTangents, const Circle& _agentRight,
    const Circle& _queryLeft){
  DubinsTrajectory next;
  next.type = RSL;
  double arcL1, arcL2, arcL3; //arcLengths
  Control nextControl; //for a control vector
  if (_RLTangents.size() > 2){
    //tangent pts function returns inner tangents for RL connection third 
    nextControl.steeringAngle = -1.0 * m_maxSteering; //right turn at max
    arcL1 = ArcLength(_agentRight.GetPos(), m_start.pos, _RLTangents.at(2).first, m_minTurnRadius, false);
    //don't use velocities because Dubins assumes unit forward velocity
    nextControl.timesteps = arcL1 / DELTA;
    next.controls.push_back(nextControl);

    nextControl.steeringAngle = 0.0; //straight
    arcL2 = Norm(_RLTangents.at(2).first, _RLTangents.at(2).second);
    nextControl.timesteps = arcL2 / DELTA;
    next.controls.push_back(nextControl);

    nextControl.steeringAngle = m_maxSteering; //left turn at max
    arcL3 = ArcLength(_queryLeft.GetPos(), _RLTangents.at(2).second, m_goal.pos, m_minTurnRadius, true);
    nextControl.timesteps = arcL3 / DELTA;
    next.controls.push_back(nextControl);

    //calculate total length
    next.length =  arcL1 + arcL2 + arcL3;
  }
  return next;
}//end RSLTrajectory

DubinsTrajectory
Dubins::LSRTrajectory(vector<pair<Point2d, Point2d> >& _LRTangents, const Circle& _agentLeft,
    const Circle& _queryRight){
  DubinsTrajectory next;
  next.type = LSR;
  double arcL1, arcL2, arcL3; //arcLengths
  Control nextControl; //for a control vector
  if (_LRTangents.size() > 3){
    //tangent pts function returns inner tangents for LR connection fourth 
    nextControl.steeringAngle = m_maxSteering; //left turn at max
    arcL1 = ArcLength(_agentLeft.GetPos(), m_start.pos, _LRTangents.at(3).first, m_minTurnRadius, true);
    //don't use velocities because Dubins assumes unit forward velocity
    nextControl.timesteps = arcL1 / DELTA;
    next.controls.push_back(nextControl);

    nextControl.steeringAngle = 0.0; //straight
    arcL2 = Norm(_LRTangents.at(3).first, _LRTangents.at(3).second);
    nextControl.timesteps = arcL2 / DELTA;
    next.controls.push_back(nextControl);

    nextControl.steeringAngle = -1.0 * m_maxSteering; //right turn at max
    arcL3 = ArcLength(_queryRight.GetPos(), _LRTangents.at(3).second, m_goal.pos, m_minTurnRadius, false);
    nextControl.timesteps = arcL3 / DELTA;
    next.controls.push_back(nextControl);

    //calculate total length
    next.length =  arcL1 + arcL2 + arcL3;
  }
  return next;
}//end LSRTrajectory
