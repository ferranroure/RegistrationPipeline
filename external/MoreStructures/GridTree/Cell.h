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
    Cell();
    ~Cell();

    void addPoint(myPoint *p);
    myPoint * getPoint(int pos);
    int get_nPoints();
    Super4PCS::KdTree<double> * getKdtree();
    void kdtreezation(int thsPoints);
    bool isKdtreezed();

};


#endif //PIPELINE_CELL_H
