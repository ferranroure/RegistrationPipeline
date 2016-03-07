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

myPoint::~myPoint() {

}

double myPoint::dist(const myPoint &p) {

    double res = 0;
    res = sqrt( pow(p->getX()-x,2) + pow(p->getY()-y,2) + pow(p->getZ()-z,2) );

    return res;
}

bool myPoint::operator==(myPoint p) const
{
    // account for small variations attributed to noise or rounding errors
    return ( (fabs(x-p.x )<tole) && (fabs(z-p.z )<tole) && (fabs(z-p.z )<tole) );
}

bool myPoint::operator !=(myPoint p) const
{
    return( !(*this == p) );
}

int myPoint::getIndex() {

    if(index == -1) cerr << "myPoint::Be careful! Point index not set!" << endl;
    return index;
}

void myPoint::setIndex(int _ind) {

    index = _ind;
}