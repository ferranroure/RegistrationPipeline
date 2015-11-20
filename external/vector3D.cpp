#include "vector3D.h"


vector3D::vector3D()
{
	x = 0;
	y = 0;
	z = 0;
}

vector3D::vector3D(double i, double j, double k)
{
	x = i;
	y = j;
	z = k;
}

vector3D::vector3D(const vector3D &v)
{
	x = v.x;
	y = v.y;
	z = v.z;
}

double vector3D::getX()
{
	return x;
}

double vector3D::getY()
{
	return y;
}

double vector3D::getZ()
{
	return z;	
}

void vector3D::setX(double i)
{
	x = i;
}

void vector3D::setY(double j)
{
	y = j;
}

void vector3D::setZ(double k)
{
	z = k;
}

double  vector3D::modulus() const
{
	return sqrt(x*x + y*y + z*z);
}

vector3D& vector3D::normalize() 
{
	double r = modulus();
	x/=r; y/=r; z/=r;
	return *this;
}

vector3D vector3D::returnNormalized() const 
{
	double r = modulus();
	vector3D retorn = vector3D(x/r,y/r,z/r);
	return retorn;
}


bool vector3D::operator==(vector3D v)
{
	if(abs(x-v.x) < ERR && abs(y-v.y) < ERR && abs(z-v.z) < ERR){
		return true;
	}
	else{
		return false;
	}
}

vector3D vector3D::operator+(vector3D v)
{
	vector3D c; 
   
	c.x = x+v.x; 
	c.y = y+v.y; 
	c.z = z+v.z; 
   
	return c; 
}

vector3D vector3D::operator-(vector3D v)
{
	vector3D c; 
   
	c.x = x-v.x; 
	c.y = y-v.y; 
	c.z = z-v.z; 
   
	return c; 
}

vector3D vector3D::operator*(double r)
{
	vector3D c; 
   
	c.x = x*r; 
	c.y = y*r; 
	c.z = z*r; 
   
	return c; 
}

vector3D vector3D::operator/(double r)
{
	vector3D c; 
   
	c.x = x/r; 
	c.y = y/r; 
	c.z = z/r; 
   
	return c; 
}

vector3D vector3D::crossProduct(const vector3D& v) const
{
	vector3D c; 
   
	c.x = y * v.z - z * v.y;
	c.y = z * v.x - x * v.z; 
	c.z = x * v.y - y * v.x; 
   
	return c; 
}

double  vector3D::dotProduct(const vector3D& v) const
{
	double res;

	res = x*v.x + y*v.y + z*v.z;
	return res;
}

// return a vector that is ortogonal to this
vector3D vector3D::normalTo() 		
{
	if(x!=0) return vector3D(y,-x,0);
	else if(y!=0) return vector3D(-y,x,0);
	else if(z!=0) return vector3D(0,-z,y);
	else 
	{
		cout<<"vector3D::normalTo exception, trying to compute a vector normal to (0,0,0)"<<endl;
		throw "vector3D::normalTo exception, trying to compute a vector normal to (0,0,0)";
	}

}


double vector3D::signedAngle(const vector3D& v2, const vector3D& reference2) const
{
   vector3D u = returnNormalized(); 	
   vector3D v = v2.returnNormalized(); 	
   vector3D reference = reference2.returnNormalized(); 	

    vector3D c = u.crossProduct(v);
    double angle = atan2(c.modulus(), u.dotProduct(v));
    return c.dotProduct(reference) < 0.f ? -angle : angle;
}

void vector3D::write()
{
    cout << "(" << x << "," << y << "," << z << ")" << endl;
}
