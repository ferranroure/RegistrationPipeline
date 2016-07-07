/**
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


#include "4pcsRegistrationObject.h"
#include "../external/vec.h"
#include "timer.h"

#include <fstream>

#define FIX_RAND 1000
#define TOO_CLOSE (qd*0.1)

#define randu() ((float) rand() / (float) RAND_MAX)
#define randf(a,b) (randu()*((b)-(a))+(a))
#define _sqr(x) ((x)*(x))
using namespace std;


fpcsRegistrationObject::fpcsRegistrationObject() {

	dataStruct = NULL;
	dataStructType = "gridtree";
	app = 0.0;
	useNormals = 1;
	normDiff = 0.01;
	qd = -1;
	numTry = -1;
	meanDist0 = 1.0;
	estFrac = 0.9;
	thr = -1;
	debug = false;
	sample = ((float)500);
	quad.resize(4);
}

fpcsRegistrationObject::~fpcsRegistrationObject(){

	delete dataStruct;
}



static fstream debugf;

//----------------------------------------------------------------------------------------------
static double PointsDistance(const Point3D &p, const Point3D &q)
{
	return sqrt(_sqr(p.x-q.x) + _sqr(p.y-q.y) + _sqr(p.z-q.z));
}


//----------------------------------------------------------------------------------------------
static double PointsDistance2(const Point3D &p, const Point3D &q)
{
	return sqrt(_sqr(p.x-q.x) + _sqr(p.y-q.y));
}





//----------------------------------------------------------------------------------------------
static double computeBestRigid(vector<pair<Point3D,Point3D> > &pairs,
							   LA_Fmat &R,
							   double &tx,double &ty,double &tz,
							   double &cx,double &cy,double &cz
							   )
{
	int i,j;
	double	x1 = 0.0;
	double	y1 = 0.0;
	double	z1 = 0.0;
	double	x2 = 0.0;
	double	y2 = 0.0;
	double	z2 = 0.0;

	for(i = 0; i < pairs.size(); i++)
	{
		const Point3D &p = pairs[i].first;
		const Point3D &q = pairs[i].second;
		x1+=p.x;
		y1+=p.y;
		z1+=p.z;
		x2+=q.x;
		y2+=q.y;
		z2+=q.z;
	}
	x1/=pairs.size();
	y1/=pairs.size();
	z1/=pairs.size();
	x2/=pairs.size();
	y2/=pairs.size();
	z2/=pairs.size();

	point centroid1(x1,y1,z1);
	point centroid2(x2,y2,z2);

	double u1=0,u2=0;
	vector<pair<Point3D,Point3D> > tp=pairs;
	double l1,l2,t;

	for (i=0;i<pairs.size();i++)
	{
		double d=FLT_MAX;
		int best;
		for (j=0;j<pairs.size();j++)
		{
			point s1(tp[i].first.x,tp[i].first.y,tp[i].first.z);
			point s2(tp[j].second.x,tp[j].second.y,tp[j].second.z);
			l1=dist2(s1,centroid1);
			l2=dist2(s2,centroid2);
			t=fabs(l1-l2);
			if (t<d)
			{
				d=t;
				best=j;
			}
		}
		pairs[i].second = tp[best].second;
	}

	point rl1(pairs[0].first.x,pairs[0].first.y,pairs[0].first.z);
	point rl2(pairs[1].first.x,pairs[1].first.y,pairs[1].first.z);
	point rl3(pairs[2].first.x,pairs[2].first.y,pairs[2].first.z);
	point rr1(pairs[0].second.x,pairs[0].second.y,pairs[0].second.z);
	point rr2(pairs[1].second.x,pairs[1].second.y,pairs[1].second.z);
	point rr3(pairs[2].second.x,pairs[2].second.y,pairs[2].second.z);

	centroid1 = rl1 + rl2 + rl3;
	centroid1 /= 3.0;
	centroid2 = rr1 + rr2 + rr3;
	centroid2 /= 3.0;

	point xl = rl2 - rl1;
	normalize(xl);
	point yl = (rl3 - rl1) - ((rl3-rl1) DOT xl) * xl;
	normalize(yl);
	point zl = xl CROSS yl;

	point xr = rr2 - rr1;
	normalize(xr);
	point yr = (rr3 - rr1) - ((rr3-rr1) DOT xr) * xr;
	normalize(yr);
	point zr = xr CROSS yr;

	LA_Fmat Tl(3,3);
	Tl(0,0) = xl[0];
	Tl(0,1) = xl[1];
	Tl(0,2) = xl[2];
	Tl(1,0) = yl[0];
	Tl(1,1) = yl[1];
	Tl(1,2) = yl[2];
	Tl(2,0) = zl[0];
	Tl(2,1) = zl[1];
	Tl(2,2) = zl[2];

	LA_Fmat Tr(3,3);
	Tr(0,0) = xr[0];
	Tr(0,1) = xr[1];
	Tr(0,2) = xr[2];
	Tr(1,0) = yr[0];
	Tr(1,1) = yr[1];
	Tr(1,2) = yr[2];
	Tr(2,0) = zr[0];
	Tr(2,1) = zr[1];
	Tr(2,2) = zr[2];

	LA_Fmat Tlt = ublas::trans(Tl);
	R = ublas::prod(Tlt,Tr);



	cx = centroid2[0];
	cy = centroid2[1];
	cz = centroid2[2];
	tx = centroid1[0]-centroid2[0];
	ty = centroid1[1]-centroid2[1];
	tz = centroid1[2]-centroid2[2];

	LA_Fmat p(3,1),pp;
	double u=0.0;
	for (i=0;i<3;i++)
	{
		p(0,0)=pairs[i].second.x-centroid2[0];
		p(1,0)=pairs[i].second.y-centroid2[1];
		p(2,0)=pairs[i].second.z-centroid2[2];
		pp=ublas::prod(R,p);
		u +=
			sqrt(_sqr(pp(0,0)-(pairs[i].first.x-centroid1[0]))+
			_sqr(pp(1,0)-(pairs[i].first.y-centroid1[1]))+
			_sqr(pp(2,0)-(pairs[i].first.z-centroid1[2])));
	}
	u/=pairs.size();
	return u;
}

//----------------------------------------------------------------------------------------------
static void transform(Point3D &u,LA_Fmat &R,double cx,double cy,double cz,double tx,double ty,double tz)
{
	int i;

	LA_Fmat p(3,1),pp;
	p(0,0)=(u.x-cx);
	p(1,0)=(u.y-cy);
	p(2,0)=(u.z-cz);
	pp=ublas::prod(R,p);
	u.x = pp(0,0)+cx+tx;
	u.y = pp(1,0)+cy+ty;
	u.z = pp(2,0)+cz+tz;

}

//----------------------------------------------------------------------------------------------
static void transform(vector<Point3D> &v,LA_Fmat &R,double cx,double cy,double cz,double tx,double ty,double tz)
{
	int i;

	LA_Fmat p(3,1),pp;
	for (i=0;i<v.size();i++)
	{
		p(0,0)=(v[i].x-cx);
		p(1,0)=(v[i].y-cy);
		p(2,0)=(v[i].z-cz);
		pp=ublas::prod(R,p);
		v[i].x = pp(0,0)+cx+tx;
		v[i].y = pp(1,0)+cy+ty;
		v[i].z = pp(2,0)+cz+tz;

	}
}


//----------------------------------------------------------------------------------------------
template <class T>
static bool sameSet(T a,T b,T c,T d,T a1,T b1,T c1,T d1)
{
	if (a==b || a==c || a==d || b==c || b==d || c==d) return false;
	set<T> s;
	s.insert(a);
	s.insert(b);
	s.insert(c);
	s.insert(d);
	if (s.find(a1)!=s.end() && s.find(b1)!=s.end() && s.find(c1)!=s.end() && s.find(d1)!=s.end()) return true;
	return false;
}


//----------------------------------------------------------------------------------------------
static inline bool checkQuad(const Point3D &p1,const Point3D &p2,const Point3D &p3,const Point3D &p4,
							 float d1,float d2,float d3,float d4,float e)
{
	float dd1=PointsDistance(p1,p3);
	float dd2=PointsDistance(p1,p4);
	float dd3=PointsDistance(p2,p3);
	float dd4=PointsDistance(p2,p4);
	if (fabs(d1-dd1)<2*e && fabs(d2-dd3)<2*e && fabs(d3-dd2)<2*e && fabs(d4-dd4)<2*e) return true; else return false;
}


//----------------------------------------------------------------------------------------------

// --------------------- FERRAN ----------------------------------------------------------------
// This function checks if the pointColor of points are different in order to discard points in walls or roofs

static inline bool checkColor(const Point3D &p1,const Point3D &p2,const Point3D &p3,const Point3D &p4, float thrs){

	int diff = 255 * thrs;
//cout << "caca" << endl;
	if( (abs(p1.r-p2.r)<diff && abs(p2.r-p3.r)<diff && abs(p3.r-p4.r)<diff) &&
		(abs(p1.g-p2.g)<diff && abs(p2.g-p3.g)<diff && abs(p3.g-p4.g)<diff) &&
		(abs(p1.b-p2.b)<diff && abs(p2.b-p3.b)<diff && abs(p3.b-p4.b)<diff) ){

		cout << "same pointColor" << endl;
		return false;
	}
	else return true;
}


// ---------------------------------------------------------------------------------------------


bool fpcsRegistrationObject::findQuads(const vector<Point3D> &v,
									   vector<Point3D> &quad,
									   vector<pair<int,int> > &r1,
									   vector<pair<int,int> > &r2,
									   double f1,
									   double f2,
									   double e,
									   double e1,
									   vector<QuadIndex> &ret
									   )
{
	// -------------- FERRAN --------------
	float colorThrs = 0.3;
	// ------------------------------------

	int i,j;
	int n_pts = 2*r1.size();
	ANNkd_tree		*tree;
//	IDataStructure *dt_aux = NULL;
	int n = n_pts;

	ANNpointArray	data_pts = annAllocPts(n_pts, 3);
	ANNpoint		query_pt;
	ANNidxArray		nn_idx;
	ANNdistArray	dists;
	query_pt = annAllocPt(3);
	nn_idx = new ANNidx[n];
	dists = new ANNdist[n];

	ret.clear();

	float d1=PointsDistance(quad[0],quad[2]);
	float d2=PointsDistance(quad[0],quad[3]);
	float d3=PointsDistance(quad[1],quad[2]);
	float d4=PointsDistance(quad[1],quad[3]);

	for (i = 0; i < r1.size(); i++)
	{
		const Point3D &p1 = v[r1[i].first];
		const Point3D &p2 = v[r1[i].second];
		data_pts[i*2][0] = p1.x+f1*(p2.x-p1.x);
		data_pts[i*2][1] = p1.y+f1*(p2.y-p1.y);
		data_pts[i*2][2] = p1.z+f1*(p2.z-p1.z);
		data_pts[i*2+1][0] = p1.x+f2*(p2.x-p1.x);
		data_pts[i*2+1][1] = p1.y+f2*(p2.y-p1.y);
		data_pts[i*2+1][2] = p1.z+f2*(p2.z-p1.z);
	}

	tree = new ANNkd_tree(
		data_pts,
		n_pts,
		3);

	for (i=0;i<r2.size();i++)
	{
		const Point3D &p1 = v[r2[i].first];
		const Point3D &p2 = v[r2[i].second];

		// Aquest deu ser el primer punt intermig e.
		query_pt[0] = p1.x+f1*(p2.x-p1.x);
		query_pt[1] = p1.y+f1*(p2.y-p1.y);
		query_pt[2] = p1.z+f1*(p2.z-p1.z);
		tree->annkFRSearch(
			query_pt,
			e1,
			n,
			nn_idx,
			dists,
			app);


		for (j=0;j<n;j++)
		{
			if (dists[j]!=ANN_DIST_INF)
			{
				int id = nn_idx[j]/2;
				if (checkQuad(v[r2[i].first],v[r2[i].second],v[r1[id].first],v[r1[id].second],d1,d2,d3,d4,e)) {
					//--------------- FERRAN ------------
//					if(checkColor(v[r2[i].first],v[r2[i].second],v[r1[id].first],v[r1[id].second], colorThrs)) {
						ret.push_back(QuadIndex(r1[id].first, r1[id].second, r2[i].first, r2[i].second));
//					}
					// ----------------------------------
				}
			} else break;
		}

		// I aquest deu ser el segon punt intermig e.
		query_pt[0] = p1.x+f2*(p2.x-p1.x);
		query_pt[1] = p1.y+f2*(p2.y-p1.y);
		query_pt[2] = p1.z+f2*(p2.z-p1.z);
		tree->annkFRSearch(
			query_pt,
			e1,
			n,
			nn_idx,
			dists,
			app);
		for (j=0;j<n;j++)
		{
			if (dists[j]!=ANN_DIST_INF)
			{
				int id = nn_idx[j]/2;
				if (checkQuad(v[r2[i].first],v[r2[i].second],v[r1[id].first],v[r1[id].second],d1,d2,d3,d4,e)) {
					//--------------- FERRAN ------------
//					if(checkColor(v[r2[i].first],v[r2[i].second],v[r1[id].first],v[r1[id].second], colorThrs)) {
						ret.push_back(QuadIndex(r1[id].first,r1[id].second,r2[i].first,r2[i].second));
//					}
					// ----------------------------------
				}
			} else break;
		}
	}

	annDeallocPt(query_pt);
	annDeallocPts(data_pts);
	delete [] nn_idx;
	delete [] dists;
	delete tree;
	return ret.size()!=0;
}


//----------------------------------------------------------------------------------------------
static float dist3D_Segment_to_Segment( point &p1,point &p2,point &q1, point &q2,float &f1,float &f2)
{
	point   u = p2 - p1;
	point   v = q2 - q1;
	point   w = p1 - q1;
	float    a = u DOT u;
	float    b = u DOT v;
	float    c = v DOT v;
	float    d = u DOT w;
	float    e = v DOT w;
	float    D = a*c - b*b;
	float    sc, sN, sD = D;
	float    tc, tN, tD = D;

	if (D < 0.0001) {
		sN = 0.0;
		sD = 1.0;
		tN = e;
		tD = c;
	}
	else {
		sN = (b*e - c*d);
		tN = (a*e - b*d);
		if (sN < 0.0) {
			sN = 0.0;
			tN = e;
			tD = c;
		}
		else if (sN > sD) {
			sN = sD;
			tN = e + b;
			tD = c;
		}
	}

	if (tN < 0.0) {
		tN = 0.0;
		if (-d < 0.0)
			sN = 0.0;
		else if (-d > a)
			sN = sD;
		else {
			sN = -d;
			sD = a;
		}
	}
	else if (tN > tD) {
		tN = tD;
		if ((-d + b) < 0.0)
			sN = 0;
		else if ((-d + b) > a)
			sN = sD;
		else {
			sN = (-d + b);
			sD = a;
		}
	}
	sc = (abs(sN) < 0.0001 ? 0.0 : sN / sD);
	tc = (abs(tN) < 0.0001 ? 0.0 : tN / tD);

	point   dP = w + (sc * u) - (tc * v);  // = S1(sc) - S2(tc)

	f1 = sc;
	f2 = tc;

	return len(dP);
}






















bool fpcsRegistrationObject::tryQuad(const vector<Point3D> &v,vector<Point3D> &quad,double &f1,double &f2,float e)
{
	// ORDERNA ELS PUNTS PQ ES CREUIN ELS SEGMENTS
	int i,j;
	vector<point> nq;
	nq.push_back(point(quad[0].x,quad[0].y,quad[0].z));
	nq.push_back(point(quad[1].x,quad[1].y,quad[1].z));
	nq.push_back(point(quad[2].x,quad[2].y,quad[2].z));
	nq.push_back(point(quad[3].x,quad[3].y,quad[3].z));

	float minv = FLT_MAX,u1,u2;
	int b1,b2,b3,b4;
	int k,l;
	for (i=0;i<4;i++)
	{
		for (j=0;j<4;j++)
		{
			if (i==j) continue;
			k=0; while (k==i || k==j) k++;
			l=0; while (l==i || l==j || l==k) l++;
			float f = dist3D_Segment_to_Segment(nq[i],nq[j],nq[k],nq[l],u1,u2);
			if (f<minv)
			{
				minv = f;
				b1 = i;
				b2 = j;
				b3 = k;
				b4 = l;
				f1 = u1;
				f2 = u2;
			}
		}
	}
	vector<Point3D> tmp = quad;
	quad[0] = tmp[b1];
	quad[1] = tmp[b2];
	quad[2] = tmp[b3];
	quad[3] = tmp[b4];
	return true;
}





bool fpcsRegistrationObject::selectRandomDiam(const vector<Point3D> &v,int &a,int &b,int &c)
{
	float d=0.0,dt;
	int n=v.size();
	int i,at,bt,ct;
	a=b=c=-1;
	float x;

	// punt aleatori que serà fixat
	at=rand()%n;

	// Ho prova 1000 vegades.
	for (i=0;i<FIX_RAND;i++)
	{
		// dos punts aleatoris dins el rang.
		bt=rand()%n;
		ct=rand()%n;
		// dos vectors entre els dos aleatoris i l'escollit abans.
		point u(v[bt].x-v[at].x,v[bt].y-v[at].y,v[bt].z-v[at].z);
		point w(v[ct].x-v[at].x,v[ct].y-v[at].y,v[ct].z-v[at].z);
		// La meitat de la distància del prod vectorial dels dos vectors.
		dt = 0.5 * len(u CROSS w);

		// comprova que els punts estiguis prou wide. qd és un diametre tunejat que ve per paràmetre (depen de l'estimació de l'overlap crec).
		if (dt>d && len(u)<qd && len(w)<qd)
		{
			d=dt;
			a=at;
			b=bt;
			c=ct;
		}
	}
	if (a==-1||b==-1||c==-1) return false; else return true;
}





bool fpcsRegistrationObject:: selectQuad(const vector<Point3D> &v,vector<Point3D> &quad,double &f1,double &f2,int &a,int &b,int &c,int &d)
{
	int i,j;
	double x,y;
	int tr=0;
	while (tr<FIX_RAND)
	{
		// Selecciona els tres punts de la Base que són wide entre ells.
		if (!selectRandomDiam(v,a,b,c)) return false;

//		cout << "Area: " << ISearchingStrategy::calcArea(new Point(v[a].x, v[a].y, v[a].z),
//														 new Point(v[b].x, v[b].y, v[b].z),
//														 new Point(v[c].x, v[c].y, v[c].z)) << endl;

		quad[0]=v[a];
		quad[1]=v[b];
		quad[2]=v[c];

		double x1=quad[0].x;
		double y1=quad[0].y;
		double z1=quad[0].z;
		double x2=quad[1].x;
		double y2=quad[1].y;
		double z2=quad[1].z;
		double x3=quad[2].x;
		double y3=quad[2].y;
		double z3=quad[2].z;

		// Fa el determinant
		double X = (-x3*y2*z1 + x2*y3*z1 + x3*y1*z2 - x1*y3*z2 - x2*y1*z3 + x1*y2*z3);

		// centre de masses dels tres punts.
		Point3D cg((x1+x2+x3)/3,(y1+y2+y3)/3,(z1+z2+z3)); ///// COMPTE QUE DEU FALTAR UN /3!!!!!!!!!!!!!!!!!!!
		double bestv=FLT_MAX;
		double di;
		int u;
		double d1,d2,d3,d0;

		// Si no són linealment independents (no pertanyen a la mateixa recta)
		if (X!=0)
		{
			// Troben els coeficiens del pla.
			double A = (-y2*z1 + y3*z1 + y1*z2 - y3*z2 - y1*z3 + y2*z3)/X;
			double B = (x2*z1 - x3*z1 - x1*z2 + x3*z2 + x1*z3 - x2*z3)/X;
			double C = (-x2*y1 + x3*y1 + x1*y2 - x3*y2 - x1*y3 + x2*y3)/X;
			d=-1;
			bestv = FLT_MAX;
			for (i=0;i<v.size();i++)
			{
				d0=PointsDistance(v[i],cg);
				d1=PointsDistance(v[i],v[a]);
				d2=PointsDistance(v[i],v[b]);
				d3=PointsDistance(v[i],v[c]);
				// Si està massa aprop d'un dels punts, ho descartem pq no seria wide.
				if (d1<TOO_CLOSE || d2<TOO_CLOSE || d3<TOO_CLOSE) continue;
				// No sé què és aixo. És una distancia però no sé de què. Sembla que sigui la funció del pla.
				// Deu mirar si és coplanar. Agafa el que s'ajusta més.
				di=fabs(A*v[i].x+B*v[i].y+C*v[i].z-1.0);
				if (di<bestv)
				{
					bestv=di;
					d = i;
				}
			}

			// Si hem trobat el 4t punt
			if (d!=-1)
			{
				quad[3]=v[d];
				// Provem que sigui bo. Diria que reordena la base perquè les parelles siguin bones.
				// SI NO es creuen en busca de nous.
				if (tryQuad(v,quad,f1,f2,2*bestv))
				{
					// ------------- FERRAN ----------
//					if(checkColor(quad[0], quad[1], quad[2], quad[3], 0.3)) {
						return true;
//					}
					// -------------------------------
				}
			}
		}
		tr++;
	}
	return false;

}










double fpcsRegistrationObject::meanDist(vector<Point3D> &v)
{
	srand(0);
	int i;
	float d=0.0;

	int n=0;
	for (i=0;i<FIX_RAND*10;i++)
//	for (int k=0; k<v.size(); k++)
	{
		int k=rand()%v.size();
		Point *queryP = new Point(v[k].x, v[k].y, v[k].z);
		queryP->setIndex(k);

		returnData rd = dataStruct->calcOwnNN(queryP);

		if (sqrt(rd.sqrDist)<diam*0.05)
		{
			d += sqrt(rd.sqrDist);
			n++;
		}

//		cout << k << ";" << rd.index << ";" << v[rd.index].x << ";"<< v[rd.index].y << ";" << v[rd.index].z << ";" << rd.sqrDist << endl;

		delete queryP;
	}


	return d/(float)n;
}



// THIS METHOD CHECKS THE LCP BETWEEN BOTH MODELS
double fpcsRegistrationObject::	verify(const vector<Point3D> &v1,
									  double eps,LA_Fmat &R,
									  double bestf,
									  double cx,double cy,
									  double cz,double tx,double ty,double tz
									  )
{
	int i;
	float d;
	int s=0;
	double e=eps;
	int n1=v1.size();
	float rnd = n1;
	int a=bestf*rnd;
	Point3D	p;
	float jmp = (float)n1/(float)rnd;
	float root_e = sqrt(e);

//	vector<int> indx;
//	indx.push_back(400);
//	indx.push_back(411);
//	indx.push_back(421);
//	indx.push_back(438);
//	indx.push_back(448);
//	indx.push_back(452);
//	indx.push_back(502);
//	indx.push_back(505);
//	indx.push_back(575);
//	indx.push_back(576);



	for (i=0;i<rnd;i++)
//	for (i=0;i<indx.size();i++)
	{
//		p=v1[indx.at(i)];
		p=v1[i];

		transform(p,R,cx,cy,cz,tx,ty,tz);

		Point *queryP = new Point(p.x, p.y, p.z);
		queryP->setIndex(i);

		returnData rd = dataStruct->calcOneNN(queryP, root_e);



		if (rd.sqrDist<e)
		{
			s++;
//			cout << i << " " << rd.index << " dist: " << sqrt(rd.sqrDist) << " thrs: " << e << endl;

		}

		delete queryP;

		if (rnd-i+s<a) return (float)s / (float)rnd;

	}

	cout << "-------------------------> LCP: " << s << " s/size: " << (float)s/(float)rnd << endl;
//	exit(0);
	return (float)s / (float)rnd;
}



void fpcsRegistrationObject::bruteForcePairs(
	const vector<Point3D> &v,
	double d,
	double dd,
	double e,
	vector<pair<int,int> > &r)
{
	int i,j;

	double t,t1;
	r.clear();
	r.reserve(2*v.size());
	int k=0;

	double dd1,dd2,norm2;
	for (j=0;j<v.size();j++)
	{
		const Point3D &p = v[j];
		for (i=j+1;i<v.size();i++)
		{
			// dist. entre els dos punts.
			t=sqrt(_sqr(v[i].x-p.x)+_sqr(v[i].y-p.y)+_sqr(v[i].z-p.z));
			if (useNormals)
			{
				double dd1 = PointsDistance(Point3D(v[i].n1,v[i].n2,v[i].n3),
											Point3D(p.n1,p.n2,p.n3));
				double dd2 = PointsDistance(Point3D(v[i].n1,v[i].n2,v[i].n3),
											Point3D(-p.n1,-p.n2,-p.n3));
				t1 = min(fabs(dd1-dd),fabs(dd2-dd));
			}
			else t1=0;
			if (fabs(t-d)<e && (!useNormals || (useNormals && t1<normDiff)))
			{
				r.push_back(pair<int,int>(j,i));
				r.push_back(pair<int,int>(i,j));
			}
		}

	}

}







bool fpcsRegistrationObject::tryOne(
									vector<Point3D> &m1,
									vector<Point3D> &m2,
									float &bestf,
									float eps,
									LA_Fmat &rMat
									)
{

	int i,j,k;
	double a,b,c,tx,ty,tz;
	vector<pair<Point3D,Point3D> > pr(4);

	double f1,f2;

	// Deuen ser els id's del quad
	int id1,id2,id3,id4;

	// Selecciona la primera base. Ho fa random però que compleixi que sigui wide.
	if (!selectQuad(m1,quad,f1,f2,id1,id2,id3,id4))
	{
		return false;
	}

	//printing the base A
//	writePly("../models/bases/baseA.ply", quad);

//	cout << "Selected base: " << endl;
//	cout << quad[0].x << ", " << quad[0].y << ", " << quad[0].z << endl;
//	cout << quad[1].x << ", " << quad[1].y << ", " << quad[1].z << endl;
//	cout << quad[2].x << ", " << quad[2].y << ", " << quad[2].z << endl;
//	cout << quad[3].x << ", " << quad[3].y << ", " << quad[3].z << endl << endl;

	// Distància entre punts.
	double d1=PointsDistance(quad[0],quad[1]);
	double d2=PointsDistance(quad[2],quad[3]);

	vector<pair<int,int> > r1,r2;

	vector<QuadIndex> ret;

	double dd1=0,dd2=0;
	if (useNormals)
	{
		dd1 = PointsDistance(Point3D(quad[0].n1,quad[0].n2,quad[0].n3),Point3D(quad[1].n1,quad[1].n2,quad[1].n3));
		dd2 = PointsDistance(Point3D(quad[2].n1,quad[2].n2,quad[2].n3),Point3D(quad[3].n1,quad[3].n2,quad[3].n3));

	}

	// Pot ser que aquí agafi els punts de veritat. Al paper posa que, pels punts approx copanars,
	// fa servir els punts més propers entre les linies que uneixen les parelles per calcular els ratios.
	// Aquí deu estar agafant els punts reals del conjunt... suposo...
	bruteForcePairs(m2,d1,dd1,eps,r1);
	bruteForcePairs(m2,d2,dd2,eps,r2);

//	cout <<  r1.size() << " " << r2.size() << " eps: " << eps << endl;

	if (r1.size()==0 || r2.size()==0) return false;


	// Aqui deu buscar els quads candidats de Q.
	Timer timer;
	timer.start();

	if (!findQuads(m2,quad,r1,r2,f1,f2,eps*2,eps,ret))
	{
		return false;
	}
//	cout << ret.size() << " of " << r1.size() << " candidate found in : " << timer.elapsed() << "s. " << endl;

	bool first = false;

	LA_Fmat R(3,3),R1(3,3),R0(3,3);




	unsigned int A = ret.size();

	Timer globalTimer;
	globalTimer.start();
	for (i=0;i<A;i++)
	{
		timer.reset();

		pr.resize(4);
		int a1=ret[i].a,b1=ret[i].b,c1=ret[i].c,d1=ret[i].d;
		pr[0].first=m1[id1];
		pr[0].second=m2[a1];
		pr[1].first=m1[id2];
		pr[1].second=m2[b1];
		pr[2].first=m1[id3];
		pr[2].second=m2[c1];
		pr[3].first=m1[id4];
		pr[3].second=m2[d1];



		double cx,cy,cz;

		// f deu ser la distància mitjana entre les correspondències.
		double f;
		f=computeBestRigid(pr,R,tx,ty,tz,cx,cy,cz);

//		writeMatrix("matrix.xls", R, cx, cy, cz, tx, ty, tz);

		if (f<5*eps) {
			// Aqui no sé perquè torna a multiplicar meanDist*eps*2. En principi esta fent: meanDist*(meanDist*2*delta)*2
			timer.reset();
			f = verify(list2, meanDist0 * eps * 2.0, R, bestf, cx, cy, cz, tx, ty, tz);

			// For residue computation tests.

			if (f > bestf) {


				base[0] = id1;
				base[1] = id2;
				base[2] = id3;
				base[3] = id4;

				currentCongruent[0] = a1;
				currentCongruent[1] = b1;
				currentCongruent[2] = c1;
				currentCongruent[3] = d1;

				// ------------ FERRAN -------------
//				cout << "Saving Bases..." << endl;
//				cout << "BaseA: " << id1 << " "<< id2 << " "<< id3 << " "<< id4 << endl;
//				cout << "BaseB: " << a1 << " "<< b1 << " "<< c1 << " "<< d1 << endl;
//				vector<Point3D> baseA;
//				baseA.push_back(Point3D(m1[id1].x+xc1, m1[id1].y+yc1, m1[id1].z+zc1));
//				baseA.push_back(Point3D(m1[id2].x+xc1, m1[id2].y+yc1, m1[id2].z+zc1));
//				baseA.push_back(Point3D(m1[id3].x+xc1, m1[id3].y+yc1, m1[id3].z+zc1));
//				baseA.push_back(Point3D(m1[id4].x+xc1, m1[id4].y+yc1, m1[id4].z+zc1));
//
//				vector<Point3D> query;
//				query.push_back(Point3D(m2[a1].x+xc2, m2[a1].y+yc2, m2[a1].z+zc2));
//				query.push_back(Point3D(m2[b1].x+xc2, m2[b1].y+yc2, m2[b1].z+zc2));
//				query.push_back(Point3D(m2[c1].x+xc2, m2[c1].y+yc2, m2[c1].z+zc2));
//				query.push_back(Point3D(m2[d1].x+xc2, m2[d1].y+yc2, m2[d1].z+zc2));

//				vector<Point3D> baseA;
//				baseA.push_back(m1[id1]);
//				baseA.push_back(m1[id2]);
//				baseA.push_back(m1[id3]);
//				baseA.push_back(m1[id4]);
//
//				vector<Point3D> query;
//				query.push_back(m2[a1]);
//				query.push_back(m2[b1]);
//				query.push_back(m2[c1]);
//				query.push_back(m2[d1]);

//				writePly("../models/bases/baseA.ply", baseA);
//				writePly("../models/bases/baseB.ply", query);


//		exit(0);

				// ---------------------------------


				bestf = f;
				first = true;
				rMat = R;
				bcx = cx;
				bcy = cy;
				bcz = cz;
				btx = tx;
				bty = ty;
				btz = tz;


			}
			if (bestf > thr) goto done;
		}
//		cout << "             Candidate checked in: " << timer.elapsed() << endl; timer.reset();
	}
done:
//	cout << "     " <<  A << " candidates checked in: " <<  globalTimer.elapsed() << " bestf: " << bestf << endl;
	if (bestf>thr) return true; else return false;
}






static void printMat(LA_Fmat &m,char x)
{
	printf("%c:\n%25.3f %25.3f %25.3f\n%25.3f %25.3f %25.3f\n%25.3f %25.3f %25.3f\n\n",
		x,
		m(0,0),m(0,1),m(0,2),
		m(1,0),m(1,1),m(1,2),
		m(2,0),m(2,1),m(2,2)
	);
}

static void printMat4(LA_Fmat &mat)
{
	printf("%25.3f %25.3f %25.3f %25.3f\n%25.3f %25.3f %25.3f %25.3f\n%25.3f %25.3f %25.3f %25.3f\n%25.3f %25.3f %25.3f %25.3f\n\n",
		mat(0,0),mat(0,1),mat(0,2),mat(0,3),
		mat(1,0),mat(1,1),mat(1,2),mat(1,3),
		mat(2,0),mat(2,1),mat(2,2),mat(2,3),
		mat(3,0),mat(3,1),mat(3,2),mat(3,3));
}

void fpcsRegistrationObject::initialize(std::vector<Point3D> &v1,std::vector<Point3D> &v2,float delta,float overlapEstimation)
{

	//debugf.open("debug.dat",ios::out);

	srand(1962);
	int i,j;

	estFrac = overlapEstimation;

	// NO SÉ QUÈ ÉS AQUESTS XC'S
	xc1=0;yc1=0;zc1=0;
	xc2=0;yc2=0;zc2=0;


	// Això són vectors de punts.
	list1.clear();
	list2.clear();
	// i això vectors d'enters (identificadors)
	sampMapP.clear();
	sampMapQ.clear();

	if(sample<v1.size())
	{
		cout<<"************************************************************************************************************************************sampling needed "<<v1.size()<<endl;

		// SEL·LECCIÓ RANDOM UNA MICA ESTRANYA.
		// sample és el número de punts que li diem a la crida del compute.
		// Agafa els randoms divisibles per s1 i s2. Acaba aconseguint més o menys #"sample" punts.
		int s1 = v1.size() / sample;
		int s2 = v2.size() / sample;

		for (i = 0; i < v1.size(); i++) {
			if (rand() % s1 == 0) {
				//			cout << "id: " << i << endl;
				list1.push_back(v1[i]);
				sampMapP.push_back(i);
			}
		}
		for (i = 0; i < v2.size(); i++) {
			if (rand() % s2 == 0) {
				list2.push_back(v2[i]);
				sampMapQ.push_back(i);
			}
		}
	}
	else// no sampling needed
	{
		//cout<<"no sampling "<<v1.size()<<endl;

		for (i = 0; i < v1.size(); i++) {
				list1.push_back(v1[i]);
				sampMapP.push_back(i);
		}
		for (i = 0; i < v2.size(); i++) {
				list2.push_back(v2[i]);
				sampMapQ.push_back(i);
		}
	}

	cout<<"dins del 4points object, he samplejat "<<list1.size()<<" "<<list2.size()<<" venitn de smaple "<<sample<<endl;

	// TOTA AQUESTA HISTÒRIA ÉS PER TRASLLADAR ELS PUNTS SELECIONATS AL CENTRE DE MASSES
	// I TREBALLAR DES D'ALLÀ.
	for (i=0;i<list1.size();i++)
	{
		xc1+=list1[i].x;
		yc1+=list1[i].y;
		zc1+=list1[i].z;
	}

	xc1/=list1.size();
	yc1/=list1.size();
	zc1/=list1.size();


	for (i=0;i<list2.size();i++)
	{
		xc2+=list2[i].x;
		yc2+=list2[i].y;
		zc2+=list2[i].z;
	}


	xc2/=list2.size();
	yc2/=list2.size();
	zc2/=list2.size();

	for (i=0;i<list1.size();i++)
	{
		list1[i].x -= xc1;
		list1[i].y -= yc1;
		list1[i].z -= zc1;
	}
	for (i=0;i<list2.size();i++)
	{
		list2[i].x -= xc2;
		list2[i].y -= yc2;
		list2[i].z -= zc2;
	}


	// passa de la llista a un format per tractar amb el kdtree.
	int n_pts = list1.size();
//cout << "# of samples: " << n_pts << endl;

	if (dataStruct) delete dataStruct;

	vector<Point*> points;

	for (i = 0; i < list1.size(); i++)
	{
		Point *p = new Point(list1[i].x, list1[i].y, list1[i].z);
		p->setIndex(i);
		points.push_back(p);
	}



	diam = 0.0;
	int at,bt;
	// FIX_RAND = 1000.
	if(FIX_RAND<list2.size()) {
		for (i = 0; i < FIX_RAND; i++) {
			// suposo que deu ser per agafar un punt random dins de list2.size();
			at = rand() % list2.size();
			bt = rand() % list2.size();
			// crea un vector entre els dos punts sel·leccionats
			point u(list2[bt].x - list2[at].x, list2[bt].y - list2[at].y, list2[bt].z - list2[at].z);
			double l = len(u);
			// n'extreu la distància i ho fa servir de diametre (suposo que servirà per sel·leccionar el quad inicial)
			if (l > diam) {
				diam = l;
			}
		}
		cout<<"4pcs regobject approximate diameter "<<diam<<endl;

	}
else // we have few points so we compute the real diameter
 {
		for (i = 0; i < list2.size(); i++) {
			for (j = 0; j < list2.size(); j++) {
				// crea un vector entre els dos punts sel·leccionats
				point u(list2[i].x - list2[j].x, list2[i].y - list2[j].y, list2[i].z - list2[j].z);
				double l = len(u);
				// n'extreu la distància i ho fa servir de diametre (suposo que servirà per sel·leccionar el quad inicial)
				if (l > diam) {
					diam = l;
				}
			}
		}
	cout<<"4pcs regobject Computed diameter "<<diam<<endl;
	}


	if(dataStructType == "kdtree"){
		dataStruct = new myKdtree(&points);
	}
	else if(dataStructType == "gridtree"){
		dataStruct = new myGridTree(&points, diam);
	}
	else if(dataStructType == "trihash"){
		dataStruct = new myTriHash(&points, diam);
	}
	else{
		dataStruct = new myGridTree(&points, diam);
	}


	meanDist0 = meanDist(list1)*2.0;

//	cout << meanDist0/2 <<  endl;

	// incialitza variables globals.
	delta=meanDist0*delta;
	qd = diam*estFrac*2.0;
	double u = log(0.00001)/log(1.0-pow((double)estFrac,4.0));
	numTry=(int)u*(diam/0.3)/qd;
	if (thr<0) thr = estFrac;
	if (numTry<4) numTry=4;
	cdelta = delta;
	currentTry = 0;
	bestf = 0.0;
	cv2 = v2;
	for (i=0;i<4;i++)
	{
		base[i]=0;
		currentCongruent[i]=0;
	}
}



bool fpcsRegistrationObject::perform_N_steps(int n,double mat[4][4],vector<Point3D> &v2)
{
	memset(mat,0,sizeof(double)*16);
	for (int i=0;i<4;i++) mat[i][i]=1.0;

	LA_Fmat rMat(3,3);
	float f;
	bool ok;
	int i,j;
	float lb=bestf;


//	cout << "4PCS_reg::Numtry = " << numTry <<" "<<n<<endl;
	for (i=currentTry;i<currentTry+n;i++)
	{
//		cout << "4PCS_reg :: tries: " << i << " "<<endl;
		ok=tryOne(list1,list2,bestf,cdelta,rMat);
		if (ok || i>numTry) break;
	}

	currentTry += n;

	if (bestf>lb)
	{

		// HERE RETURNS ADD THE RETURN VECTOR TO THE ORIGINAL POSITION
		Point3D p(bcx+xc2,bcy+yc2,bcz+zc2);
		transform(p,rMat,0,0,0,0,0,0);
		v2 = cv2;
		vector<Point3D> ccv = v2;
		LA_Fmat omat(4,4);
		omat(0,0) = rMat(0,0);
		omat(0,1) = rMat(0,1);
		omat(0,2) = rMat(0,2);
		omat(0,3) = bcx-p.x+btx+xc1;
		omat(1,0) = rMat(1,0);
		omat(1,1) = rMat(1,1);
		omat(1,2) = rMat(1,2);
		omat(1,3) = bcy-p.y+bty+yc1;
		omat(2,0) = rMat(2,0);
		omat(2,1) = rMat(2,1);
		omat(2,2) = rMat(2,2);
		omat(2,3) = bcz-p.z+btz+zc1;
		omat(3,0) = 0;
		omat(3,1) = 0;
		omat(3,2) = 0;
		omat(3,3) = 1;

		for (i=0;i<ccv.size();i++)
		{
			LA_Fmat p(4,1),pp;
			p(0,0)=ccv[i].x;
			p(1,0)=ccv[i].y;
			p(2,0)=ccv[i].z;
			p(3,0)=1;
			pp=ublas::prod(omat,p);

			ccv[i].x=pp(0,0);
			ccv[i].y=pp(1,0);
			ccv[i].z=pp(2,0);
		}

		for (i=0;i<4;i++)
		{
			for (j=0;j<4;j++)
			{
				mat[i][j]=omat(i,j);
			}
		}
		v2 = ccv;
	}

	if(!ok) {cout<<"+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++4points Failed to produce a result "<<endl; }
	else{cout<<"++++++++++++++++++++++++++++++++++++++++++++++++++++Finished ok!"<<endl;}

	return ok || currentTry>numTry;
}


float fpcsRegistrationObject::compute(vector<Point3D> &v1, vector<Point3D> &v2, float delta, float overlapEstimation,
									  double mat[4][4])
{
	initialize(v1,v2,delta,overlapEstimation);

//	writePly("../models/bases/model4pcsA.ply", list1);
//	writePly("../models/bases/model4pcsB.ply", list2);

	bool ok=false;
	while (!ok)
	{
		ok=perform_N_steps(numTry,mat,v2);
	}

	return bestf;
}


// -------------- FERRAN --------------

void fpcsRegistrationObject::writePly(string outpath, vector<Point3D> points) {

	PlyIO plyio;

	vector<Point> lin;

	for (int i = 0; i < points.size(); ++i) {

		Point p(points.at(i).x, points.at(i).y, points.at(i).z);
		lin.push_back(p);
	}

	if(lin.size() <= 4) {
		plyio.writeBase(outpath, &lin);
	}
	else{
		plyio.writeFile(outpath, &lin);
	}
}

void fpcsRegistrationObject::writeMatrix(const char *outpath, LA_Fmat &rMat, double cx, double cy, double cz, double tx,
										 double ty, double tz) {

	ofstream file;
	file.open (outpath, std::ios_base::app);

	Point3D p(cx+xc2,cy+yc2,cz+zc2);
	transform(p,rMat,0,0,0,0,0,0);
	file << rMat(0,0) << ";";
	file << rMat(0,1) << ";";
	file << rMat(0,2) << ";";
	file << cx-p.x+tx+xc1 << ";";
	file << rMat(1,0) << ";";
	file << rMat(1,1) << ";";
	file << rMat(1,2) << ";";
	file << cy-p.y+ty+yc1 << ";";
	file << rMat(2,0) << ";";
	file << rMat(2,1) << ";";
	file << rMat(2,2) << ";";
	file << cz-p.z+tz+zc1 << ";";
	file << 0 << ";";
	file << 0 << ";";
	file << 0 << ";";
	file << 1 << ";";
	file << "\n";

	file.close();

}

void fpcsRegistrationObject::setDataStructType(string type){

	dataStructType = type;
}

void fpcsRegistrationObject::printStats(){

	cout << "VALUES OF 4PCS" << endl;
	string dataStructType;

	cout << "app: " << app << endl;
	cout << "usenormals: " << useNormals << endl;
	cout << "normaldiff: " << normDiff << endl;
	// és un diàmetre tunejat. Deu servir per seleccionar les bases wide.
	cout << "qd: " << qd << endl;
	cout << "diam: " << diam << endl;
	cout << "numtry: " <<  numTry << endl;
	// threshold per considarar punts aparellats (diria)
	cout << "cdelta: " << cdelta << endl;
	cout << "meandist: " << meanDist0 << endl;
	// estimacio de l'oberlapping (diria)
	cout << "estfrac: " << estFrac << endl;
	cout << "thr: " << thr << endl;
	cout << "bestf: " << bestf << endl;
	cout << "currenttry: " << currentTry << endl;
}
