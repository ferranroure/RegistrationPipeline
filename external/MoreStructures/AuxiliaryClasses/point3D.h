#ifndef _POINT3D_
#define _POINT3D_

#include <math.h>
#include <iostream>
using namespace std;

#include "../../vector3D.h"

class point3D
{
	private:
		double x;
		double y;
		double z;
		
	public:
		point3D(); //Default constructor
		point3D(double i, double j, double k); //constructor that receives the coordinates of the point
		point3D(const point3D &p); //Copy constructor
		
		// accessor methods
		double getX()const; 
		double getY()const;
		double getZ()const; 
		void setX(double i); 
		void setY(double j); 
		void setZ(double k); 
		
		//Operators
		void operator=(point3D p); //assignment
		point3D operator+(vector3D v); 
		point3D operator-(vector3D v);
		vector3D operator-(point3D p); 

		double dist(point3D a); //distance between two points

		//Comparison operators
		bool operator ==(point3D p)const;
		bool operator <(point3D p)const;
		bool operator !=(point3D p)const;
		bool operator >(point3D p)const;
		bool operator <=(point3D p)const;
		bool operator >=(point3D p)const;

		double outsideCoordinate(); //technical method to use points and octrees
		
		void write(); //print point coordinates
		void write(ostream& os)
		{
			os << "(" << x << "," << y << "," << z << ")";
		}
		friend ostream& operator<<(ostream& os,point3D p) 
		{
			p.write(os);
			return os;
		}
};


#endif
