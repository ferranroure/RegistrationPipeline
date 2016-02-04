/** 
Interface file for the 4PCS method to match two pointsets in 3D under rigid transformation

The code is an implementation of the 4PCS algorithm presented in:

4-points Congruent Sets for Robust Surface Registration
Dror Aiger, Niloy J. Mitra, Daniel Cohen-Or
ACM SIGGRAPH 2008 (to appear), ACM Transaction of Graphics .

AUTORIGHTS
Copyright (C) 2008 Ben Gurion University of the Negev, Beer Sheva, Israel.
All rights reserved

Written by Dror Aiger (aiger@cs.bgu.ac.il) and Niloy J. Mitra (niloym@gmail.com)


Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met

* The use for research only (no for any commercial application).
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS AND CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <set>

#include <boost/numeric/ublas/detail/config.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include <vector>
#include <ANN/ANN.h>

// --------------- FERRAN ---------------
#include "../ISearchingStrategy.h"
#include "plyio.h"
#include "point.h"
// --------------------------------------

// ublass definitions
namespace ublas  = boost::numeric::ublas;
typedef ublas::matrix<double, ublas::column_major, ublas::unbounded_array<double> > LA_Fmat;
typedef ublas::vector<double, ublas::unbounded_array<double> >                      LA_Fvec;


// holds the base from P
struct QuadIndex
{
	int a;
	int b;
	int c;
	int d;
	QuadIndex(int _a,int _b,int _c,int _d):a(_a),b(_b),c(_c),d(_d){}
};


/** The basic 3D point structure. A point contains also directional information that may or may not be used.
The directional information can be computed from a mesh or other structures.
*/
struct Point3D
{
	// coordinates
	double x;
	double y;
	double z;

	// normal
	double n1;
	double n2;
	double n3;
	int a;

	// -------------- FERRAN ----------
	int r;
	int g;
	int b;

	Point3D(double _x,double _y,double _z, int _r, int _g, int _b):x(_x),y(_y),z(_z),r(_r),g(_g),b(_b),a(0){n1=n2=n3=0;}

	// --------------------------------
	Point3D(double _x,double _y,double _z,int _a):x(_x),y(_y),z(_z),a(_a){n1=n2=n3=0;}
	Point3D(double _x,double _y,double _z):x(_x),y(_y),z(_z),a(0){n1=n2=n3=0;}
	Point3D(){n1=n2=n3=0;}

	bool operator == (const Point3D &t)
	{
		return x==t.x && y==t.y;
	}
	bool operator < (const Point3D &t) const
	{
		if (x<t.x) return true;
		else if (x==t.x && y<t.y) return true;
		else if (x==t.x && y==t.y && z<t.z) return true;
		else return false;
	}
};


//----------------------------------------------------------------------------------------------
/** The class for the computation of the 4PCS algorithm (see the paper for details) */
class fpcsRegistrationObject
{
private:
	// internal data members
	IDataStructure *dataStruct; // This data structure is used for Data Structure Tests, to be able to check the runtime. However, can also be used as a regular datastructure.
	ANNkd_tree *the_tree;		
	ANNpointArray data_pts0;
	float app;
	int useNormals;
	float normDiff;
	// és un diàmetre tunejat. Deu servir per seleccionar les bases wide.
	float qd;
	float diam;
	int numTry;
	// threshold per considarar punts aparellats (diria)
	float cdelta;
	float meanDist0;
	// estimacio de l'oberlapping (diria)
	float estFrac;
	float bcx,bcy,bcz;
	float btx,bty,btz;
	float thr;
	int sample;
	bool debug;
	int base[4];
	int currentCongruent[4];
	std::vector<Point3D> list1;
	std::vector<Point3D> list2;
	std::vector<Point3D> quad;
	std::vector<int> sampMapP;
	std::vector<int> sampMapQ;
	std::vector<Point3D> cv2;
	float xc1,yc1,zc1;
	float xc2,yc2,zc2;
	float bestf;
	int currentTry;

	// internal private methods
	bool tryOne(
				   std::vector<Point3D> &m1,
				   std::vector<Point3D> &m2,
				   float &bestf,
				   float eps,
				   LA_Fmat &rMat
				   );
	void bruteForcePairs(
							const std::vector<Point3D> &v,
							double d,
							double dd,
							double e,
							std::vector<std::pair<int,int> > &r);
	double verify(const std::vector<Point3D> &v1,
					 double eps,LA_Fmat &R,
					 double bestf,
					 double cx,double cy,
					 double cz,double tx,double ty,double tz
					 );

	double meanDist(std::vector<Point3D> &v);
	bool selectQuad(const std::vector<Point3D> &v,std::vector<Point3D> &quad,
		double &f1,double &f2,int &a,int &b,int &c,int &d);
	bool selectRandomDiam(const std::vector<Point3D> &v,int &a,int &b,int &c);
	bool tryQuad(const std::vector<Point3D> &v,std::vector<Point3D> &quad,double &f1,double &f2,float e);
	bool findQuads(const std::vector<Point3D> &v,
							   std::vector<Point3D> &quad,
							   std::vector<std::pair<int,int> > &r1,
							   std::vector<std::pair<int,int> > &r2,
							   double f1,
							   double f2,
							   double e,
							   double e1,
							   std::vector<QuadIndex> &ret
							   );

// public section
public:
	fpcsRegistrationObject():
			dataStruct(0),
			myPoints(0),
			the_tree(0),
			data_pts0(0),
			app(0.0),
			useNormals(1),
			normDiff(0.01),
			qd(-1),
			numTry(-1),
			meanDist0(1.0),
			estFrac(0.9),
			thr(-1),
			debug(false),
			sample((float)500)
	  {
		  quad.resize(4);
	  }
	~fpcsRegistrationObject() {}
	/**
	Compute an approximation of the LCP (directional) from set2 to set1.
	The input sets may or may not contain a normal information for any point.
	@param [in] set1 the first input set
	@param [in] set2 the second input set 
	@param [in] the delta for the LCP (see the paper)
	@param [in] overlapEstimation estimation of the overlap between the set as a fraction of the
	size of second set ([0..1])
	@param [out] mat transformation matrix
	@return the computed LCP measure
	*/
	float compute(vector<Point3D> &v1, vector<Point3D> &v2, float delta, float overlapEstimation,
				  double mat[4][4]);

	// set number of samples from P (a fixed number should be OK but the application might want to set it otherwise)
	void setSample(int a) { sample = a; }

	// set a threshold. the algorithm quits when this threshold was reached. -1 means that the overlap value is taken
	void setThreshold(float a) { thr = a; }

	// set the number of sample points. in general the algorithm works well with constant number of points
	void setNumberOfPoints(int a) { sample = (float)a; }

	// set the difference between normals that is allowed (degrees)
	void setNormDiff(float a) 
	{ 
		static double PI=acos(-1.0);
		if (a>180) normDiff = 2.0;
		else normDiff = 2.0*sin((a/2.0)*PI/180.0); 
	}

	void setUseNormal(bool a) { useNormals = a; }

	// return indices (to the set P, the first set in "compute") to the 4-point of the base
	void getBase(int b[4]) 
	{
		b[0] = sampMapP[base[0]];
		b[1] = sampMapP[base[1]];
		b[2] = sampMapP[base[2]];
		b[3] = sampMapP[base[3]];
	}

	// return indices (to the set Q, the second set in "compute") to the 4-point of the current selected congruent set
	// in Q. The set changed while the algorithm is running.
	void getCurrentCongruentSet(int c[4])
	{
		c[0] = sampMapQ[currentCongruent[0]];
		c[1] = sampMapQ[currentCongruent[1]];
		c[2] = sampMapQ[currentCongruent[2]];
		c[3] = sampMapQ[currentCongruent[3]];
	}

	void	initialize(std::vector<Point3D> &v1,std::vector<Point3D> &v2,float delta,float overlapEstimation);
	bool	perform_N_steps(int n,double mat[4][4],std::vector<Point3D> &v2);
	float	getCurrentLCP() { return bestf; }
	int		getMaxNumTry() const{return numTry;}


	// ---------------- FERRAN --------------

	void writePly(string outpath, vector<Point3D> points);

	void setDataStruct(IDataStructure *_ids){dataStruct = _ids;}

	double customVerify(const std::vector<Point3D> &v1,
						double eps, LA_Fmat &R,
						double bestf,
						double cx, double cy,
						double cz, double tx, double ty, double tz
	);

	vector<Point*> *myPoints;
	void setMyPoints(vector<Point*> *vp){myPoints = vp;}

};