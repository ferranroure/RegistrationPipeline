#ifndef _VECTOR3D_
#define _VECTOR3D_

#include <math.h>
#include <iostream>
#include <cmath>
using namespace std;

#define ERR 0.01

class vector3D
{
    public:
		double x;
		double y;
		double z;
		
		vector3D(); //Default constructor
		vector3D(double i, double j, double k); // Constructor that receives the three coordinates of the vector
		vector3D(const vector3D &v); //Copy constructor
		
		// Accessor methods
		double getX(); 
		double getY(); 
		double getZ(); 
		void setX(double i); 
		void setY(double j); 
		void setZ(double k);
		
		double modulus() const; // modulus of a vector

		// methods to normalize a vector
		vector3D& normalize(); // affects "this"
		vector3D returnNormalized()const;

		// Class and arithmetic Operators
		bool operator==(vector3D v);
		vector3D operator+(vector3D v); 
		vector3D operator-(vector3D v);
		vector3D operator*(double r); // multiply each component times a real number
        vector3D operator/(double r); // divide each component times a real number

		// cross and dot products
		vector3D crossProduct(const vector3D& v)const; 
		double dotProduct(const vector3D& v)const; 

		vector3D normalTo(); // return a vector that is ortogonal to this		
		double signedAngle(const vector3D& v2, const vector3D& reference2) const; // returns signed angle between two vectors considering a reference direction, everyone must have previously been normalized!

		void write(); 
		void write(ostream& os)
		{
			os << "(" << x << "," << y << "," << z << ")";
		}
		friend ostream& operator<<(ostream& os,vector3D v) 
		{
			v.write(os);
			return os;
		}
};

#endif
