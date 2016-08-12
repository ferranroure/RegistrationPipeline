/***************************************************************************
 * Joan Rosset
 * 2007
 * Yago Diez
 * 2010
 ***************************************************************************/
#include "motion3D.h"

#define DIMENSIONS 4
#define TOL 0.0000001

#include <iostream>
#include <cstdlib>

#ifndef PI
#define PI M_PI
#endif


// INVERSE METHODS AND VARIABLES FROM NUMERICAL RECIPES
/*
PYTHAG computes sqrt(a^{2} + b^{2}) without destructive overflow or underflow.
*/
static double at, bt, ct;
#define PYTHAG(a, b) ((at = fabs(a)) > (bt = fabs(b)) ? \
(ct = bt/at, at*sqrt(1.0+ct*ct)): (bt ? (ct = at/bt, bt*sqrt(1.0+ct*ct)): 0.0))

static double maxarg1, maxarg2;
#define MAX(a, b) (maxarg1 = (a), maxarg2 = (b), (maxarg1) > (maxarg2) ? \
(maxarg1) : (maxarg2))

#define SIGN(a, b) ((b) < 0.0 ? -fabs(a): fabs(a))

void motion3D::error(char error_text[])
    /* Numerical Recipes standard error handler.			*/
  {

    //cerr	<< "Numerical Recipes run-time error...\n"
		//<< form("%s\n", error_text)
	//	<< "...now exiting to system...\n";
   // exit(1);
  }

/*
Given a matrix a[m][n], this routine computes its singular value
decomposition, A = U*W*V^{T}.  The matrix U replaces a on output.
The diagonal matrix of singular values W is output as a vector w[n].
The matrix V (not the transpose V^{T}) is output as v[n][n].
m must be greater or equal to n;  if it is smaller, then a should be
filled up to square with zero rows.
*/
void motion3D::svdcmp(double* a[], int m, int n, double w[], double* v[])
  {
    int flag, i, its, j, jj, k, l, nm;
    double c, f, h, s, x, y, z;
    double anorm = 0.0, g = 0.0, scale = 0.0;

    if (m < n)
      error("SVDCMP: Matrix A must be augmented with extra rows of zeros.");
    double* rv1 = new double [n];

    /* Householder reduction to bidiagonal form.			*/
    for (i = 0; i < n; i++)
      {
	l = i + 1;
        rv1[i] = scale*g;
        g = s = scale = 0.0;
        if (i < m)
	  {
	    for (k = i; k < m; k++)
	      scale += fabs(a[k][i]);
	    if (scale)
	      {
	        for (k = i; k < m; k++)
	          {
		    a[k][i] /= scale;
		    s += a[k][i]*a[k][i];
	          };
	        f = a[i][i];
	        g = -SIGN(sqrt(s), f);
	        h = f*g - s;
	        a[i][i] = f - g;
	        if (i != n - 1)
	          {
		    for (j = l; j < n; j++)
		      {
		        for (s  = 0.0, k = i; k < m; k++)
		          s += a[k][i]*a[k][j];
		        f = s/h;
		        for ( k = i; k < m; k++)
			  a[k][j] += f*a[k][i];
		      };
	          };
	        for (k = i; k < m; k++)
	          a[k][i] *= scale;
	      };
          };
        w[i] = scale*g;
        g = s= scale = 0.0;
        if (i < m && i != n - 1)
          {
	    for (k = l; k < n; k++)
	      scale += fabs(a[i][k]);
	    if (scale)
	      {
	        for (k = l; k < n; k++)
	          {
		    a[i][k] /= scale;
		    s += a[i][k]*a[i][k];
	          };
	        f = a[i][l];
	        g = -SIGN(sqrt(s), f);
	        h = f*g - s;
	        a[i][l] = f - g;
	        for (k = l; k < n; k++)
	          rv1[k] = a[i][k]/h;
	        if (i != m - 1)
	          {
		    for (j = l; j < m; j++)
		      {
		        for (s = 0.0, k = l; k < n; k++)
		          s += a[j][k]*a[i][k];
		        for (k = l; k < n; k++)
		          a[j][k] += s*rv1[k];
		      };
	          };
	        for (k = l; k < n; k++)
	          a[i][k] *= scale;
	      };
	  };
        anorm = MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
      };
    /* Accumulation of right-hand transformations.			*/
    for (i = n - 1; 0 <= i; i--)
      {
	if (i < n - 1)
	  {
	    if (g)
	      {
		for (j = l; j < n; j++)
		  v[j][i] = (a[i][j]/a[i][l])/g;
		  /* Double division to avoid possible underflow:	*/
		for (j = l; j < n; j++)
		  {
		    for (s = 0.0, k = l; k < n; k++)
		      s += a[i][k]*v[k][j];
		    for (k = l; k < n; k++)
		      v[k][j] += s*v[k][i];
		  };
	      };
	    for (j = l; j < n; j++)
	      v[i][j] = v[j][i] = 0.0;
	  };
	v[i][i] = 1.0;
	g = rv1[i];
	l = i;
      };
    /* Accumulation of left-hand transformations.			*/
    for (i = n - 1; 0 <= i; i--)
      {
	l = i + 1;
	g = w[i];
	if (i < n - 1)
	  for (j = l; j < n; j++)
	    a[i][j] = 0.0;
	if (g)
	  {
	    g = 1.0/g;
	    if (i != n - 1)
	      {
		for (j = l; j < n; j++)
		  {
		    for (s = 0.0, k = l; k < m; k++)
		      s += a[k][i]*a[k][j];
		    f = (s/a[i][i])*g;
		    for (k = i; k < m; k++)
		      a[k][j] += f*a[k][i];
		  };
	       };
	    for (j = i; j < m; j++)
	      a[j][i] *= g;
	  }
	else
	  for (j = i; j < m; j++)
	    a[j][i] = 0.0;
	++a[i][i];
      };
    /* Diagonalization of the bidiagonal form.				*/
    for (k = n - 1; 0 <= k; k--)	/* Loop over singular values.	*/
      {
	for (its = 0; its < 30; its++)	/* Loop over allowed iterations.*/
	  {
	    flag = 1;
	    for (l = k; 0 <= l; l--)	/* Test for splitting:		*/
	      {
		nm = l - 1;		/* Note that rv1[0] is always zero.*/
		if (fabs(rv1[l]) + anorm == anorm)
		  {
		    flag = 0;
		    break;
		  };
		if (fabs(w[nm]) + anorm == anorm)
		  break;
	      };
	    if (flag)
	      {
		c = 0.0;		/* Cancellation of rv1[l], if l>0:*/
		s = 1.0;
		for (i = l; i <= k; i++) {
		    f = s*rv1[i];
		    if (fabs(f) + anorm != anorm)
		      {
			g = w[i];
			h = PYTHAG(f, g);
			w[i] = h;
			h = 1.0/h;
			c = g*h;
			s = (-f*h);
			for (j = 0; j < m; j++)
			  {
			    y = a[j][nm];
			    z = a[j][i];
			    a[j][nm] = y*c + z*s;
			    a[j][i]  = z*c - y*s;
			  };
		      };
		  };
	      };
	    z = w[k];
	    if (l == k)		/* Convergence.				*/
	      {
		if (z < 0.0)	/* Singular value is made non-negative.	*/
		  {
		    w[k] = -z;
		    for (j = 0; j < n; j++)
		      v[j][k] = (-v[j][k]);
		  };
		break;
	      };
	    if (its == 29)
	      error("No convergence in 30 SVDCMP iterations.");
	    x = w[l];		/* Shift from bottom 2-by-2 minor.	*/
	    nm = k - 1;
	    y = w[nm];
	    g = rv1[nm];
	    h = rv1[k];
	    f = ((y - z)*(y + z) + (g - h)*(g + h))/(2.0*h*y);
	    g = PYTHAG(f, 1.0);
	    f = ((x - z)*(x + z) + h*((y/(f + SIGN(g, f))) - h))/x;
	    /* Next QR transformation:					*/
	    c = s = 1.0;
	    for (j = l; j <= nm; j++)
	      {
		i = j + 1;
		g = rv1[i];
		y = w[i];
		h = s*g;
		g = c*g;
		z = PYTHAG(f, h);
		rv1[j] = z;
		c = f/z;
		s = h/z;
		f = x*c + g*s;
		g = g*c - x*s;
		h = y*s;
		y = y*c;
		for (jj = 0; jj < n;  jj++)
		  {
		    x = v[jj][j];
		    z = v[jj][i];
		    v[jj][j] = x*c + z*s;
		    v[jj][i] = z*c - x*s;
		  };
		z = PYTHAG(f, h);
		w[j] = z;	/* Rotation can be arbitrary if z = 0.	*/
		if (z)
		  {
		    z = 1.0/z;
		    c = f*z;
		    s = h*z;
		  };
		f = (c*g) + (s*y);
		x = (c*y) - (s*g);
		for (jj = 0; jj < m; jj++)
		  {
		    y = a[jj][j];
		    z = a[jj][i];
		    a[jj][j] = y*c + z*s;
		    a[jj][i] = z*c - y*s;
		  };
	      };
	    rv1[l] = 0.0;
	    rv1[k] = f;
	    w[k] = x;
	  };
      };
    delete [] rv1;
  }
//********************************************


motion3D::motion3D() // modified to create identity matrix
{
	for(int i=0; i< DIMENSIONS; i++){
		for(int j=0; j< DIMENSIONS; j++){
            if (i==j){
                mat[i][j] = 1.0;
            }
            else{
                mat[i][j] = 0.0;
            }
		}
	}
}

motion3D::motion3D(double m[4][4])
{
	for(int i=0; i< DIMENSIONS; i++){
		for(int j=0; j< DIMENSIONS; j++){
			mat[i][j] = m[i][j];
		}
	}
}

motion3D motion3D::transpose()
{
	double mat2[4][4];
	for(int i=0; i< DIMENSIONS; i++){
		for(int j=0; j< DIMENSIONS; j++){
			mat2[i][j] = mat[j][i];
		}
	}
	return motion3D(mat2);
}


motion3D::motion3D(double v1, double v2,double v3) // tranlation constructor, from vector
{
	double m[4][4] = { {1,0,0,v1},
			   {0,1,0,v2},
			   {0,0,1,v3},
			   {0,0,0,1} };

	for(int i=0; i< DIMENSIONS; i++){
		for(int j=0; j< DIMENSIONS; j++){
			mat[i][j] = m[i][j];
		}
    }
}

motion3D::motion3D(double alfa,int type ) // rotation constructor, from angle and type 1=x,2=y,3=z 
{
	if( type==1)
	{
		double m[4][4] = { {1,0,0,0},
                {0,cos(alfa),-sin(alfa),0},
                {0,sin(alfa),cos(alfa),0},
			    {0,0,0,1} };

		for(int i=0; i< DIMENSIONS; i++){
			for(int j=0; j< DIMENSIONS; j++){
				mat[i][j] = m[i][j];
			}
		}
	}	
	else if(type==2)
	{
		double m[4][4] = { {cos(alfa),0,sin(alfa),0},
			    {0,1,0,0},	
			    {-sin(alfa),0,cos(alfa),0},	
			    {0,0,0,1} };

		for(int i=0; i< DIMENSIONS; i++){
			for(int j=0; j< DIMENSIONS; j++){
			mat[i][j] = m[i][j];
			}
		}
	}
	else if(type==3)
	{
		double m[4][4] = { {cos(alfa),-sin(alfa),0,0},
			    {sin(alfa),cos(alfa),0,0},	
			    {0,0,1,0}, 
			    {0,0,0,1} };

		for(int i=0; i< DIMENSIONS; i++){
			for(int j=0; j< DIMENSIONS; j++){
			mat[i][j] = m[i][j];
			}
		}
	}
	else
	{
		cout<<"A motion3D.CPP, ROTATION CONSTRUTOR, WRONG TYPE"<<endl;
		exit(-1);	
	}

	
}

motion3D::motion3D(const motion3D &m)
{
	if(&m!=this)
	{
		for(int i=0; i< DIMENSIONS; i++){
			for(int j=0; j< DIMENSIONS; j++){
				mat[i][j] = m.mat[i][j];
			}
		}
	}
}

//constructor to create a motion3D to move one point oriented triplet to another, watch out for distance restrictions (triplets must have same distance between points except tolerance)
motion3D::motion3D(Point p1, Point p2, Point p3, Point q1, Point q2, Point q3, double toler)
{
	// check distance restriction
	vector3D v1,v2,v3,w1,w2,w3;

    v1=vector3D(p2.getX()-p1.getX(),p2.getY()-p1.getY(),p2.getZ()-p1.getZ());
    v2=vector3D(p3.getX()-p1.getX(),p3.getY()-p1.getY(),p3.getZ()-p1.getZ());
    v3=vector3D(p3.getX()-p2.getX(),p3.getY()-p2.getY(),p3.getZ()-p2.getZ());

    w1=vector3D(q2.getX()-q1.getX(),q2.getY()-q1.getY(),q2.getZ()-q1.getZ());
    w2=vector3D(q3.getX()-q1.getX(),q3.getY()-q1.getY(),q3.getZ()-q1.getZ());
    w3=vector3D(q3.getX()-q2.getX(),q3.getY()-q2.getY(),q3.getZ()-q2.getZ());

    /*cout<<"vectors: "<<endl;
	cout<<"v,w1"<<v1<<" "<<w1<<endl;
	cout<<"v,w2"<<v2<<" "<<w2<<endl;
    cout<<"v,w3"<<v3<<" "<<w3<<endl;*/

    if( fabs(v1.modulus()-w1.modulus())>toler ) throw ("motion3D::motion3D(Point p1, Point p2, Point p3, Point p4,Point q1, Point q2, Point q3, Point q4, double tol) DISTANCE DO NOT CHECK!");
    if( fabs(v2.modulus()-w2.modulus())>toler ) throw ("motion3D::motion3D(Point p1, Point p2, Point p3, Point p4,Point q1, Point q2, Point q3, Point q4, double tol) DISTANCE DO NOT CHECK!");
    if( fabs(v3.modulus()-w3.modulus())>toler ) throw ("motion3D::motion3D(Point p1, Point p2, Point p3, Point p4,Point q1, Point q2, Point q3, Point q4, double tol) DISTANCE DO NOT CHECK!");

	// compute motion3D!

	// motion3D from p1,p2,p3 "to origin"
	vector3D vBase1, vBase2, vBase3, vBase4,vAux;
	vBase1 = (p2-p1).normalize();
	vAux = (p3-p1).normalize();
	vBase2 = vBase1.crossProduct(vAux);
    vBase2.normalize();
	vBase3 = vBase1.crossProduct(vBase2);
    vBase3.normalize();
    vBase4 = p1-Point(0,0,0);

    double mMov[4][4] = { {vBase1.x,vBase2.x,vBase3.x,vBase4.x},
                {vBase1.y,vBase2.y,vBase3.y,vBase4.y},
                {vBase1.z,vBase2.z,vBase3.z,vBase4.z},
				{0,0,0,1} };

	motion3D mov = motion3D(mMov);
    motion3D movInvers = mov.inverseMotion();

	// motion3D from q1,q2,q3 "to origin"
	vBase1 = (q2-q1).normalize();
	vAux = (q3-q1).normalize();
	vBase2 = vBase1.crossProduct(vAux);
    vBase2.normalize();
	vBase3 = vBase1.crossProduct(vBase2);
    vBase3.normalize();
    vBase4 = q1-Point(0,0,0);

    double mMov2[4][4] = { {vBase1.x,vBase2.x,vBase3.x,vBase4.x},
                {vBase1.y,vBase2.y,vBase3.y,vBase4.y},
                {vBase1.z,vBase2.z,vBase3.z,vBase4.z},
				{0,0,0,1} };
		
	motion3D mov2 = motion3D(mMov2);
	motion3D movInvers2 = mov2.inverseMotion();

	motion3D compost=mov2*movInvers;

	// put the computed motion3D into "this"
	for(int i=0; i< DIMENSIONS; i++){
		for(int j=0; j< DIMENSIONS; j++){
			mat[i][j] = compost.mat[i][j];
		}
	}
}

motion3D::motion3D(vector<Point> &puntsA , vector<Point> &puntsB)
{


// check that we have the same number of points
if(puntsA.size()!=puntsB.size())
{
    cout<<"motion3D(vector<Point> &puntsA , vector<Point> &puntsB) you are trying to calculate the LST movement of two sets of point with different cardinality"<<endl;
    throw "motion3D(vector<Point> &puntsA , vector<Point> &puntsB) you are trying to calculate the LST movement of two sets of point with different cardinality";
}

int n=puntsA.size(); //number of points
int d= DIMENSIONS-1;// space dimension as DIM is set in homogeneous coordinates

// define the matrices that contain the sets of points:
vector<double > pA = vector<double>(3*n);
vector<double > pB = vector<double>(3*n);

// traverse the points in both sets and put them as "columns" in pA,pB 
for(int i=0;i<puntsA.size();i++)
{
    pA[i]=puntsA[i].getX();
    pA[n+i]=puntsA[i].getY();
    pA[2*n+i]=puntsA[i].getZ();

    pB[i]=puntsB[i].getX();
    pB[n+i]=puntsB[i].getY();
    pB[2*n+i]=puntsB[i].getZ();

}

double *pointer;
pointer=&pA[0];

Matrix X(d,n);
X<<pointer;

//cout<<"Matrix for points in A: \n"<<X<<endl;

pointer=&pB[0];
Matrix Y(d,n);
Y<<pointer;

//cout<<"Matrix for points in B: \n"<<Y<<endl;

// compute centroids
double xCentroidA,yCentroidA,zCentroidA;
double xCentroidB,yCentroidB,zCentroidB;

xCentroidA=0;
yCentroidA=0;
zCentroidA=0;

xCentroidB=0;
yCentroidB=0;
zCentroidB=0;

// point in columns!
for(int i=0;i<n;i++)
{
	xCentroidA+=pA[i];
	yCentroidA+=pA[n+i];
	zCentroidA+=pA[2*n+i];

	xCentroidB+=pB[i];
	yCentroidB+=pB[n+i];
	zCentroidB+=pB[2*n+i];
}

// divide by the total number of points:

double inverseNpoints=1./n;

xCentroidA*=inverseNpoints;
yCentroidA*=inverseNpoints;
zCentroidA*=inverseNpoints;

xCentroidB*=inverseNpoints;
yCentroidB*=inverseNpoints;
zCentroidB*=inverseNpoints;

Matrix centreA(d,1);
Matrix centreB(d,1);

centreA(1,1)=xCentroidA;
centreA(2,1)=yCentroidA;
centreA(3,1)=zCentroidA;

centreB(1,1)=xCentroidB;
centreB(2,1)=yCentroidB;
centreB(3,1)=zCentroidB;

//cout<<"Centroids computed!  \n A: ( "<<xCentroidA<<" , "<<yCentroidA<<" , "<<zCentroidA<<" ) \n and B:  ( "<<xCentroidB<<" , "<<yCentroidB<<" , "<<zCentroidB<<" ) "<<endl;

//cout<<"Centroids computed2!  \n "<<centreA<<"\n "<<centreB<<endl;
X.Row(1)=X.Row(1)-xCentroidA;
X.Row(2)=X.Row(2)-yCentroidA;
X.Row(3)=X.Row(3)-zCentroidA;

//cout<<"Matrix for points in A: \n"<<X<<endl;

Y.Row(1)=Y.Row(1)-xCentroidB;
Y.Row(2)=Y.Row(2)-yCentroidB;
Y.Row(3)=Y.Row(3)-zCentroidB;

//cout<<"Matrix for points in B: \n"<<Y<<endl;

// define also the diagonal matrix that contains the weights
DiagonalMatrix W (n);
// set all weights to 1
for(int i=1;i<=n;i++) W(i)=1;

//cout<<"weight matrix:\n "<<W<<endl;

Matrix U, V; DiagonalMatrix D;

Matrix S = X*W*Y.t();

//cout<<"computing SVD of S: \n"<<S<<endl;
   SVD(S,D,U,V);                              // S = U * D * V.t()

//cout<<"result: U: \n"<<U<<endl;
//cout<<"result: D: \n"<<D<<endl;
//cout<<"result: V: \n"<<V<<endl;

double det = Determinant(V*U.t());

//cout<<"determinant V*U.t(): "<<det_FPFHSignature33<<endl;

DiagonalMatrix Aux(n);
for(int i=1;i<n;i++) Aux(i)=1;
Aux(n)=det;

Matrix R = V*Aux*U.t();

//cout<<"Rotation Matrix: \n"<<R<<endl;

Matrix Trans = centreB - R*centreA;

//cout<<"Translation Matrix: \n"<<Trans<<endl;

Matrix puntA1(d,1);
puntA1(1,1)=pA[0];
puntA1(2,1)=pA[3];
puntA1(3,1)=pA[6];

Matrix puntA2(d,1);
puntA2(1,1)=pA[1];
puntA2(2,1)=pA[4];
puntA2(3,1)=pA[7];

Matrix puntA3(d,1);
puntA3(1,1)=pA[2];
puntA3(2,1)=pA[5];
puntA3(3,1)=pA[8];

/*cout<<"punt A1!\n "<<puntA1<<endl;
cout<<"punt A2!\n "<<puntA2<<endl;
cout<<"punt A3!\n "<<puntA3<<endl;

cout<<"Punt A1 rotat! \n"<<(R*puntA1)<<endl;
cout<<"Punt A2 rotat! \n"<<(R*puntA2)<<endl;
cout<<"Punt A3 rotat! \n"<<(R*puntA3)<<endl;

cout<<"Punt A1 desplaçat! \n"<<(R*puntA1 + Trans)<<endl;
cout<<"Punt A2 desplaçat! \n"<<(R*puntA2 + Trans)<<endl;
cout<<"Punt A3 desplaçat! \n"<<(R*puntA3 + Trans)<<endl;
*/
	for(int i=0; i< DIMENSIONS; i++)
	{
		for(int j=0; j< DIMENSIONS; j++)
		{
			if(i!= DIMENSIONS-1)
			{		
				if(j!= DIMENSIONS-1)
				{
					mat[i][j] = R.element(i,j);
				}
				else 
				{
					mat[i][j] = Trans.element(i,0);
				}
			}
			else
			{
				if(j!= DIMENSIONS-1)
				{
					mat[i][j]=0;
				}
				else 
				{
					mat[i][j]=1;
				}
			}
		}
	}

//cout<<"motion3D::motion3D(vector<Point> &puntsA , vector<Point> &puntsB) out!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;


}



motion3D::~motion3D()
{
}

void motion3D::operator=(const motion3D m)
{
	for(int i=0; i< DIMENSIONS; i++){
		for(int j=0; j< DIMENSIONS; j++){
			mat[i][j] = m.mat[i][j];
		}
	}
}

motion3D motion3D::operator+(const motion3D m)
{
	double res[DIMENSIONS][DIMENSIONS];
	for(int i=0; i< DIMENSIONS; i++){
		for(int j=0; j< DIMENSIONS; j++){
			res[i][j] = 0.0;
		}
	}

	for(int i=0; i< DIMENSIONS; i++){
		for(int j=0; j< DIMENSIONS; j++){
			res[i][j]=mat[i][j]+m.mat[i][j];
		}
	}

	return motion3D(res);
}

motion3D motion3D::operator*(const motion3D m)
{
	double res[DIMENSIONS][DIMENSIONS];
	for(int i=0; i< DIMENSIONS; i++){
		for(int j=0; j< DIMENSIONS; j++){
			res[i][j] = 0.0;
		}
	}

	for(int i=0; i< DIMENSIONS; i++){
		for(int j=0; j< DIMENSIONS; j++){
			for(int k=0; k< DIMENSIONS; k++){
				res[i][j]+=mat[i][k]*m.mat[k][j];
			}
		}
	}
	return motion3D(res);
}

motion3D motion3D::operator*(double d)
{
	double res[DIMENSIONS][DIMENSIONS];

	for(int i=0; i< DIMENSIONS; i++){
		for(int j=0; j< DIMENSIONS; j++){
			res[i][j]=mat[i][j]*d;
		}
	}
	return motion3D(res);
}

Point motion3D::operator*(Point p)
{
	double res[DIMENSIONS] = {0.0,0.0,0.0,0.0};
    double pComplet[DIMENSIONS] = {p.getX(),p.getY(),p.getZ(),1.0};

	for(int i=0; i< DIMENSIONS; i++){
		for(int k=0; k< DIMENSIONS; k++){
			res[i]+=mat[i][k]*pComplet[k];
			//cout<<"M["<<i<<"]["<<k<<"]="<<mat[i][k]<<endl;	
		}

		//Rectify small errors
		if ( (res[i]>0) && (res[i]<TOL) ) res[i]=0.0;
		if ( (res[i]<0) && (res[i]>-TOL) ) res[i]=0.0;
	}

    return Point(res[0],res[1],res[2]);
}
/*
Element motion3D::operator*(const Element e) //Operator mat * Element
{
    Point p=e.getPoint();
	double res[DIM] = {0.0,0.0,0.0,0.0};
    double pComplet[DIM] = {p.x,p.y,p.z,1.0};

	for(int i=0; i<DIM; i++){
		for(int k=0; k<DIM; k++){
			res[i]+=mat[i][k]*pComplet[k];
		}

		//Rectify small errors
		if ( (res[i]>0) && (res[i]<TOL) ) res[i]=0.0;
		if ( (res[i]<0) && (res[i]>-TOL) ) res[i]=0.0;
	}

    return Element(Point(res[0],res[1],res[2]),e.getRadi());
}
*/

vector3D motion3D::operator*(vector3D v)
{
	double res[DIMENSIONS] = {0.0,0.0,0.0,0.0};
    double vComplet[DIMENSIONS] = {v.x,v.y,v.z,1.0};

	for(int i=0; i< DIMENSIONS; i++){
		for(int k=0; k< DIMENSIONS; k++){
			res[i]+=mat[i][k]*vComplet[k];
		}
		
		//Rectify small errors
		if ( (res[i]>0) && (res[i]<TOL) ) res[i]=0.0;
		if ( (res[i]<0) && (res[i]>-TOL) ) res[i]=0.0;
	}

        return vector3D(res[0],res[1],res[2]);
}

//motion3D inverse
motion3D motion3D::inverseMotion()
{
	double epsilon = 1e-12; 

	double a[DIMENSIONS][DIMENSIONS];
	double* uu = new double [DIMENSIONS* DIMENSIONS];
	double* vv = new double [DIMENSIONS* DIMENSIONS];
	double*  w = new double [DIMENSIONS];
	double** u = new double* [DIMENSIONS];
	double** v = new double* [DIMENSIONS];

	//void svdcmp(double**, int, int, double*, double**);

	for (int i = 0; i < DIMENSIONS; i++)
		u[i] = &(uu[DIMENSIONS*i]);
	for (int j = 0; j < DIMENSIONS; j++)
		v[j] = &(vv[DIMENSIONS*j]);
	{
	for (int i = 0; i < DIMENSIONS; i++)
		for (int j = 0; j < DIMENSIONS; j++)
			u[i][j] = mat[i][j];
	}
	svdcmp(u, DIMENSIONS, DIMENSIONS, w, v);	// Singular value decomposition.
	double wmax = 0.0;			// Maximum singular value.
	{
		for (int j = 0; j < DIMENSIONS; j++)
			if (w[j] > wmax)
				wmax = w[j];
	}
	double wmin = wmax*epsilon;
	for (int k = 0; k < DIMENSIONS; k++)
		if (w[k] < wmin)
			w[k] = 0.0;
		else
			w[k] = 1.0/w[k];
	{
	for (int i = 0; i < DIMENSIONS; i++)
		for (int j = 0; j < DIMENSIONS; j++) {
			a[i][j] = 0.0;
			for (int k = 0; k < DIMENSIONS; k++)
				//a[DIM*i+j] += v[i][k]*w[k]*u[j][k];
				a[i][j] += v[i][k]*w[k]*u[j][k];

			//Rectifiquem si hi ha hagut algun petit error
			if ( (a[i][j]>-TOL) && (a[i][j]<TOL) ) a[i][j]=0.0;
			};
	}
	delete [] w;
	delete [] u;
	delete [] v;
	delete [] uu;
	delete [] vv;
	
	return motion3D(a);
}


void motion3D::write(ostream& os)
{
	double aux;
	for(int i=0; i< DIMENSIONS; i++){
		os << "\n( ";
		for(int j=0; j< DIMENSIONS; j++){
			aux = mat[i][j]*1000000;

			if ((int)aux%10 == 0) {
                os << mat[i][j] << "         ";
			} else {
                os << mat[i][j] << "  ";
			}
		}
		os << ")";
	}
}

void motion3D::update(motion3D *m){

    for(int i=0; i< DIMENSIONS; i++){
        for(int j=0; j< DIMENSIONS; j++){
            mat[i][j] = m->mat[i][j];
        }
    }

}

// compatibility between motion 3D and xform
motion3D::motion3D(xform xf)
{
	// retrieve xform matrix, it comes as a single vector and column major, so we have to distribute correctly while we transpose
	double* dAux=xf;	

	// put the values in the matrix in the correct order
	mat[0][0]=dAux[0]; mat[1][0]=dAux[1]; mat[2][0]=dAux[2]; mat[3][0]=dAux[3]; 
	mat[0][1]=dAux[4]; mat[1][1]=dAux[5]; mat[2][1]=dAux[6]; mat[3][1]=dAux[7]; 
	mat[0][2]=dAux[8]; mat[1][2]=dAux[9]; mat[2][2]=dAux[10]; mat[3][2]=dAux[11]; 
	mat[0][3]=dAux[12]; mat[1][3]=dAux[13]; mat[2][3]=dAux[14]; mat[3][3]=dAux[15];

}	

motion3D::motion3D(Eigen::Matrix3f &m){

    // Tranform to motion3D
    for(int i=0; i<=2; i++){
        for(int j=0; j<=2; j++){
            mat[i][j] = m(i,j);
        }
    }

    mat[3][0] = 0;
    mat[3][1] = 0;
    mat[3][2] = 0;

    mat[0][3] = 0;
    mat[1][3] = 0;
    mat[2][3] = 0;
    mat[3][3] = 1;
}


motion3D::motion3D(Eigen::Matrix4f &m){

    // Tranform to motion3D
    for(int i=0; i<=2; i++){
        for(int j=0; j<=2; j++){
            mat[i][j] = m(i,j);
        }
    }
}

