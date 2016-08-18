//
// Created by ferran on 4/03/16.
//

#ifndef PIPELINE_CELL_H
#define PIPELINE_CELL_H

#include "Eigen/Core"


#include <iostream>
#include <vector>
#include <accelerators/kdtree.h>
#include <accelerators/bbox.h>
#include "myPoint.h"
#include "../../../include/point.h"


#define DIMENSIONS 3

using namespace std;


class Cell {

    vector<myPoint *> points;
    Super4PCS::KdTree<double> * kdTree;

public:
    double getXMin() const {
        return xMin;
    }

    void setXMin(double xMin) {
        Cell::xMin = xMin;
    }

    double getXMax() const {
        return xMax;
    }

    void setXMax(double xMax) {
        Cell::xMax = xMax;
    }

    double getYMin() const {
        return yMin;
    }

    void setYMin(double yMin) {
        Cell::yMin = yMin;
    }

    double getYMax() const {
        return yMax;
    }

    void setYMax(double yMax) {
        Cell::yMax = yMax;
    }

    double getZMin() const {
        return zMin;
    }

    void setZMin(double zMin) {
        Cell::zMin = zMin;
    }

    double getZMax() const {
        return zMax;
    }

    void setZMax(double zMax) {
        Cell::zMax = zMax;
    }

private:
    double xMin;
    double xMax;
    double yMin;
    double yMax;
    double zMin;
    double zMax;

public:
    Cell(double ixMin,double ixMax,double iyMin,double iyMax,double izMin,double izMax);
    ~Cell();

    void addPoint(myPoint *p);
    myPoint * getPoint(int pos);
    int get_nPoints();
    Super4PCS::KdTree<double> * getKdtree();
    void kdtreezation(int thsPoints);
    bool isKdtreezed();
    bool empty();
    bool xPossiblytouched(double x, double sqEps) {
        if (x > xMax) return ((x - xMax)*(x - xMax)) < sqEps;
        else if (x < xMin) return ((xMin-x)*(xMin-x))< sqEps;
        else return true;
    }

    bool yPossiblytouched(double x, double sqEps) {
        if (x > yMax) return ((x - yMax)*(x - yMax)) < sqEps;
        else if (x < yMin) return ((yMin-x)*(yMin-x))< sqEps;
        else return true;
    }

    bool zPossiblytouched(double x, double sqEps) {
        if (x > zMax) return ((x - zMax)*(x - zMax)) < sqEps;
        else if (x < zMin) return ((zMin-x)*(zMin-x))< sqEps;
        else return true;
    }

    bool touchOut(double x,double y,double z,double sqEps)
    {
        if((x-xMin)*(x-xMin)< sqEps)return true;
        else if((x-xMax)*(x-xMax)< sqEps)return true;
        else if((y-yMin)*(y-yMin)< sqEps)return true;
        else if((y-yMax)*(y-yMax)< sqEps)return true;
        else if((z-zMin)*(z-zMin)< sqEps)return true;
        else if((z-zMax)*(z-zMax)< sqEps)return true;
        return false;

    }


};


#endif //PIPELINE_CELL_H