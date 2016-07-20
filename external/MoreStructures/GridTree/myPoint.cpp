//
// Created by ferran on 4/03/16.
//

#include "myPoint.h"

myPoint::myPoint() {

    x = 0;
    y = 0;
    z = 0;
}

myPoint::myPoint(double _x, double _y, double _z, int _ind) {

    x = _x;
    y = _y;
    z = _z;
    index = _ind;
}

myPoint::myPoint(const myPoint &p) {

    x = p.x;
    y = p.y;
    z = p.z;
}


myPoint::~myPoint() {

}

double myPoint::dist(myPoint &p) {

    double res = 0;
    res = sqrt( pow(p.getX()-x,2) + pow(p.getY()-y,2) + pow(p.getZ()-z,2) );

    return res;
}

double myPoint::sqrdist(myPoint &p) {

//    double res = 0;
//    res = ( pow(p.getX()-x,2) + pow(p.getY()-y,2) + pow(p.getZ()-z,2) );
//
//    return res;

    double _x = p.getX()-x;
    double _y = p.getY()-y;
    double _z = p.getZ()-z;

    return ( (_x*_x) + (_y*_y) + (_z*_z) );
}


int myPoint::getIndex() {

    if(index == -1) cerr << "myPoint::Be careful! Point index not set!" << endl;
    return index;
}

void myPoint::setIndex(int _ind) {

    index = _ind;
}

void myPoint::print() {

    cout << "(" << x << ",\t" << y << ",\t" << z << ") Index: " << index << endl;

}

bool myPoint::operator==(myPoint p) const
{
    // account for small variations attributed to noise or rounding errors
    return ( (fabs(x-p.x )<tole) && (fabs(y-p.y )<tole) && (fabs(z-p.z )<tole) );
}


bool myPoint::operator<(myPoint p) const //lexicographyc order
{
    if ( x < p.x ) {
        return 1;
    } else {
        if ( fabs(x - p.x)<tole ) {
            if (y < p.y) {
                return 1;
            } else {
                if ( fabs(y - p.y)<tole ) {
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

bool myPoint::operator !=(myPoint p) const
{
    return( !(*this == p) );
}

bool myPoint::operator <=(myPoint p)  const
{
    return ( (*this<p)||(*this==p) );
}

bool myPoint::operator >(myPoint p) const
{
    return ( !(*this<=p) );
}

bool myPoint::operator >=(myPoint p) const
{
    return ( !(*this<p) );
}