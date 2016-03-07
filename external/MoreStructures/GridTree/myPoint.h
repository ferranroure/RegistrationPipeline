//
// Created by ferran on 4/03/16.
//

#ifndef PIPELINE_MYPOINT_H
#define PIPELINE_MYPOINT_H

#define tole 0.00001

class myPoint {

    double x;
    double y;
    double z;

    int index;

public:
    myPoint();
    myPoint(double _x, double _y, double _z, int _ind=-1);
    ~myPoint();

    double getX(){return x;};
    double getY(){return y;};
    double getZ(){return z;};

    int getIndex();
    void setIndex(int _ind);

    double dist(const myPoint &p);
};


#endif //PIPELINE_MYPOINT_H
