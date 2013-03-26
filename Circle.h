#ifndef CIRClE_H
#define CIRCLE_H
#include <iostream>
#include <fstream>

class Circle{
	public: Circle(){values[0]=values[1]=values[2]=0.0;}
  
	Circle(double _posX, double _posY, double _radius){
		values[0]=_posX;
		values[1]=_posY;
		values[2]=_radius;
	}

	void SetPos(double _x, double _y){
		values[0] = _x;
		values[1] = _y;
	}

	void SetPos(std::pair<double,double> _pos){
		values[0] = _pos.first;
		values[1] = _pos.second;
	}

	void SetRadius(double _radius){
		values[2] = _radius;
	}

	std::pair<double,double> GetPos() const{
		return std::make_pair(values[0],values[1]);
	}
	double GetX() const {
		return values[0];
	}

	double GetY() const{
		return values[1];
	}

	double GetRadius() const{
		return values[2];
	}

	friend std::ostream& operator<< (std::ostream& fout, const Circle& lhs){
		fout << "Circle at point: (" << lhs.values[0] << ", " <<  lhs.values[1] << ") with radius = " <<
		lhs.values[2] << std::endl; 
		return fout;
	}//end overloaded output operator

	private:
	double values[3];
};

#endif
