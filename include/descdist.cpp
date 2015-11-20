#include "descdist.h"

DescDist::DescDist()
{
}


DescDist::DescDist(Point *p, float d){
    pnt = p;
    dist = d;
}



bool DescDist::operator < (const DescDist& d) const {

    if(dist < d.dist) return true;
    else return false;
}

bool DescDist::operator == (const DescDist& d) const{

    if(dist == d.dist) return true;
    else return false;
}

bool DescDist::operator > (const DescDist& d) const{

    if(dist > d.dist) return true;
    else return false;
}

