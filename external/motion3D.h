/***************************************************************************
 * Joan Rosset
 * 2007
 * Yago Diez
 * 2010
 ***************************************************************************/
#ifndef motion3D_H
#define motion3D_H

//#include <iostream.h>
//#include <math.h>

#include "../include/point.h"
#include "vector3D.h"

#include <vector>

#define WANT_STREAM                  // include.h will get stream fns
#define WANT_MATH                    // include.h will get math fns
                                     // newmatap.h will get include.h
#include "newmat10/newmatap.h"                // need matrix applications

#include "newmat10/newmatio.h"                // need matrix output routines

#include <vector>
#include "XForm.h"

#include <Eigen/Dense>

#ifdef use_namespace
using namespace NEWMAT;              // access NEWMAT namespace
#endif


class motion3D
{
	protected:
		double mat[4][4]; //motion matrix

	public:
		motion3D(); //default constuctor
		motion3D(double[4][4]); // contructor receiving a motion matrix in homogeneous coordinates
		motion3D(const motion3D &m); //copy constructor
		motion3D transpose(); // transpose motion matrix 	
		motion3D(double v1, double v2,double v3); // translation constructor (from vector)
        motion3D(double alfa, int type); // rotation constructor, from angle and type
        motion3D(Point p1, Point p2, Point p3, Point q1, Point q2, Point q3, double toler); //constructor to create a motion3D to move one point oriented triplet to another
        motion3D(vector<Point> &puntsA , vector<Point> &puntsB);
        motion3D(Eigen::Matrix3f &m);
        motion3D(Eigen::Matrix4f &m);
		
		~motion3D(); //Destructor

        void update(motion3D *m);

		// arythmetic operator
		void operator=(const motion3D m); 
		motion3D operator+(const motion3D m); 
		motion3D operator*(const motion3D m);  // motion composition
		motion3D operator*(double d); 
        Point operator*(const Point p); // apply motion
		vector3D operator*(const vector3D v);
				
		motion3D inverseMotion(); //inverse motion (in terms of matrices and of motion composition)
		void svdcmp(double* a[], int m, int n, double w[], double* v[]);
		void error(char error_text[]);

		void write(ostream& os); //write matrix through the given ostream

		// compatibiility function between motion3D and xform class
		xform motion3DtoXform() {return xform((const double *)(this->transpose()).mat);} // xforms seem to be stored in colum major order for some reason
		motion3D(xform xf);	

		friend ostream& operator<<(ostream& os,motion3D m)
		{	
			os<<"motion3D::writing motion3D "<<endl;
			m.write(os);			
			os<<"motion3D::writing motion3D "<<endl;
			return os;
		}
};

#endif
