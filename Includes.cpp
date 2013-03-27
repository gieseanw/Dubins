#include "Includes.h"

//////////////////////////////////////
//////////Implementations/////////////
//////////////////////////////////////
double RandDouble(){
  return (double)rand()/(double)RAND_MAX;
}

double Norm(const Point2d& _lhs, const Point2d& _rhs){
  return sqrt((_rhs.first - _lhs.first)*(_rhs.first - _lhs.first) +
      (_rhs.second - _lhs.second)*(_rhs.second- _lhs.second));
}//end Euclidean distance function


vector<pair<Point2d,Point2d> > TangentLines(Circle _c1, Circle _c2){
  double x1 = _c1.GetX();
  double y1 = _c1.GetY();
  double x2 = _c2.GetX();
  double y2 = _c2.GetY();
  double r1 = _c1.GetRadius();
  double r2 = _c2.GetRadius();
  double d_sq = pow(x2-x1,2) + pow(y2-y1,2);
  vector<pair<Point2d,Point2d> > returnVec;
  if (d_sq < (r1-r2)*(r1-r2)){
    //we may have a problem, the circles are either intersecting, one is within the other, but still tangent
    //at one point, or one is completely in the other. We only have a problem if one is within the other, but
    //not tangent to it anywhere
    if (d_sq != max(r1,r2) && d_sq < max(r1,r2)){
      cerr << "Circles are contained with each other and not tangent. No tangent lines exist" << endl;
      return returnVec;
    }//else they are intersecting or one is within the other, but still tangent to it
    //in the other two cases, either 1 or 2 external tangent lines remain, but there are no internal tangent
    //lines
  }

  double d = sqrt(d_sq); 
  double vx = (x2 - x1) / d;
  double vy = (y2 - y1) / d;
  for (int sign1 = +1; sign1 >= -1; sign1 -= 2) {
    double c = (r1 - sign1 * r2) / d;
    if (c*c > 1.0) continue; //want to be subtracting small from large, not adding
    double h = sqrt(max(0.0, 1.0 - c*c));

    for (int sign2 = +1; sign2 >= -1; sign2 -= 2) {
      double nx = vx * c - sign2 * h * vy;
      double ny = vy * c + sign2 * h * vx;
      returnVec.push_back(make_pair(make_pair(x1 + r1 * nx, y1 + r1 * ny),
            make_pair(x2 + sign1 * r2 * nx, y2 + sign1 * r2 * ny)));
    }
  }
  return returnVec;
}//end TangentLines function

double ArcLength(const Point2d& _center, const Point2d& _lhs, const Point2d& _rhs, double _radius, bool left){
  //ArcLength is defined as the radius of the circle * theta, the angle between the two points along the
  //circumference

  //generally, you can find the short angle between the points given the circle's center point if you turn the
  //points on the circumference into vectors from the circle center. Using the dot product of the vectors, we
  //can determine the angle between them.
  //However, for Dubin's cars we need to know directional information, and acos() only gives us a range of
  //[0,PI] radians. Because circles for the Dubins cars are either right or left-turn only circles, we need to
  //know the angle between the two points, if we were only traveling the circle's direction (left or right).
  //Using atan2, which gives us [-PI, PI] range, we can get a positive or negative angle between our start
  //(_lhs) and goal (_rhs) points. atan2(goal) - atan2(start) will give us a positive angle if, going from
  //start to goal we must rotate through a positive angle (regardless of circle direction). Atan2 still only
  //gives us the short angle, but the directional information is useful. We can say that if the returned angle
  //to rotate through is negative (right turn) but the circle's direction is positive (left turn), we'd rather
  //have the larger angle (2PI - abs(angle_returned). Vice versa for positive angles in a right-turn circle.

  Point2d vec1,vec2;
  vec1.first = _lhs.first - _center.first;
  vec1.second = _lhs.second - _center.second;

  vec2.first = _rhs.first - _center.first;
  vec2.second = _rhs.second - _center.second;

  double theta = atan2(vec2.second, vec2.first) - atan2(vec1.second,vec1.first);
  if (theta < -1e6 && left)
    theta += 2.0*PI;
  else if (theta > 1e6 && !left)
    theta -= 2.0 * PI;

  return fabs(theta*_radius);
}//end ArcLength

AgentState RandomQuery(double xMax, double yMax){
  AgentState query; //query config; where we want to get to
  query.pos.first = RandDouble() * xMax;
  query.pos.second = RandDouble() * yMax;
  query.theta = RandDouble() * 2.0 * PI;
  if (query.theta > PI){
    query.theta /= -2.0;
  }
  cout << "Next query configuration: " << query << endl;
  return query;
}
