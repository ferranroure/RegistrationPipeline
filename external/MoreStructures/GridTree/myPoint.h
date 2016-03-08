//
// Created by ferran on 4/03/16.
//


#ifndef PIPELINE_MYPOINT_H
#define PIPELINE_MYPOINT_H

#include <iostream>
#include "math.h"

#define tole 0.000001

using namespace std;

class myPoint {

private:
    double x;
    double y;
    double z;

    int index;

public:
    myPoint();
    myPoint(double _x, double _y, double _z, int _ind=-1);
    myPoint(const myPoint &p);
    ~myPoint();

    double getX(){return x;};
    double getY(){return y;};
    double getZ(){return z;};

    int getIndex();
    void setIndex(int _ind);

    double dist(myPoint &p);

    void print();

    //Comparison operators
    bool operator ==(myPoint p)const;
    bool operator <(myPoint p)const;
    bool operator !=(myPoint p)const;
    bool operator >(myPoint p)const;
    bool operator <=(myPoint p)const;
    bool operator >=(myPoint p)const;
};


#endif //PIPELINE_MYPOINT_H
