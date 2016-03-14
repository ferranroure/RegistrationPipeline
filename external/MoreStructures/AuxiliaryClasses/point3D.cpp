#include "point3D.h"

#define tol 0.00000001

point3D::point3D()
{
	x = 0;
	y = 0;
	z = 0;
}

point3D::point3D(double i, double j, double k)
{
	x = i;
	y = j;
	z = k;
}

point3D::point3D(const point3D &p)
{
	x = p.x;
	y = p.y;
	z = p.z;
}

double point3D::getX() const
{
	return x;
}

double point3D::getY() const
{
	return y;
}

double point3D::getZ() const
{
	return z;	
}

void point3D::setX(double i)
{
	x = i;
}

void point3D::setY(double j)
{
	y = j;
}

void point3D::setZ(double k)
{
	z = k;
}

void point3D::operator=(point3D p)
{
	x = p.x;
	y = p.y;
	z = p.z;
}

point3D point3D::operator+(vector3D v)
{
	point3D c; 
   
	c.x = x+v.getX(); 
	c.y = y+v.getY(); 
	c.z = z+v.getZ(); 
   
	return c; 
}

point3D point3D::operator-(vector3D v)
{
	point3D c; 
   
	c.x = x-v.getX(); 
	c.y = y-v.getY(); 
	c.z = z-v.getZ(); 
   
	return c; 
}

vector3D point3D::operator-(point3D p)
{
	vector3D c; 
   
	c.setX( x-p.x ); 
	c.setY( y-p.y ); 
	c.setZ( z-p.z ); 
   
	return c; 
}

bool point3D::operator==(point3D p) const
{
	// account for small variations attributed to noise or rounding errors	
	return ( (fabs(x-p.x )<tol) && (fabs(y-p.y )<tol) && (fabs(z-p.z )<tol) );
}

bool point3D::operator<(point3D p) const //lexicographyc order
{
if ( x < p.x ) {
		return 1;
	} else {
		if ( fabs(x - p.x)<tol ) {
			if (y < p.y) {
				return 1;
			} else {
				if ( fabs(y - p.y)<tol ) {
					if (z < p.z) {
						return 1;
					} else {
						return 0;
					}
				} else {
					return 0;
				}
			}
		} else { 
			return 0;
		}
	}
}

bool point3D::operator !=(point3D p) const
{	
	return( !(*this == p) );
}

bool point3D::operator <=(point3D p)  const
{
	return ( (*this<p)||(*this==p) );
}

bool point3D::operator >(point3D p) const
{
	return ( !(*this<=p) );
}

bool point3D::operator >=(point3D p) const
{
	return ( !(*this<p) );
}

double point3D::dist(point3D a)
{
	double res;
	res=sqrt( pow(a.x-x,2) + pow(a.y-y,2) + pow(a.z-z,2) );
	return res;
}

double point3D::outsideCoordinate()
{
	double res = 0;
	
	if (x > res || x < -res) {
		res = fabs(x);
	}
	if (y > res || y < -res) {
		res = fabs(y);
	}
	if (z > res || z < -res) {
		res = fabs(z);
	}
	
	return res;
}

void point3D::write()
{
	cout << "(" << x << "," << y << "," << z << ")";
}
