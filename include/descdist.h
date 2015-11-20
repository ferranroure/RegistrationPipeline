#ifndef DESCDIST_H
#define DESCDIST_H

#include "point.h"

class DescDist
{
public:

    Point *pnt;
    float dist;


    DescDist();
    DescDist(Point *p, float d);
    bool operator <(const DescDist& d) const;
    bool operator ==(const DescDist& d) const;
    bool operator >(const DescDist& d) const;
};

#endif // DESCDIST_H
