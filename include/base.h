/*******************************************************************************
 *  CLASS BASE
 *
 *  This class contains a group of points which forms a base.
 *
 *  Ferran Roure (froure@eia.udg.edu)
 *  ViCOROB Research Group
 *  University of Girona
 ******************************************************************************/

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include "point.h"

#ifndef BASE_H
#define BASE_H

class Base
{
public:

    // Elements ----------------------------------------------------------
    vector<Point*> base;
    Point *a, *b, *c;
    double dist_ab, dist_bc, dist_ca;
    double alpha, beta, gamma;      // alpha->from a, beta->from b, gamma->from c
    double area;

    // Methods -----------------------------------------------------------
    Base();
    Base(Point *pi, Point *pj, Point *pk);
    ~Base();
    void addPoint(Point *p);
    void addBase(Point *pi, Point *pj, Point *pk);
    void removeLastPoint();
    void clear();
    Point * getPoint(int pos);
    int size();
    void print();
    bool existRepeatedPoints();
    void computeProperties();

    Point *getA() const {
        return a;
    }

    Point *getB() const {
        return b;
    }

    Point *getC() const {
        return c;
    }

    double getDistAB()  {

        if(dist_ab == -1){
            dist_ab = a->dist(b);
        }
        return dist_ab;
    }

    double getDistBC()  {

        if(dist_bc == -1){
            dist_bc = b->dist(c);
        }
        return dist_bc;
    }

    double getDistCA()  {

        if(dist_ca == -1){
            dist_ca = c->dist(a);
        }
        return dist_ca;
    }

    double getAlpha()  {

        return alpha;
    }

    double getBeta()  {
        return beta;
    }

    double getGamma()  {
        return gamma;
    }

    double getArea() const {
        return area;
    }

};


#endif // BASE_H
