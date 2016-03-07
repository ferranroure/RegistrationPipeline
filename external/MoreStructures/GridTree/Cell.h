//
// Created by ferran on 4/03/16.
//

#ifndef PIPELINE_CELL_H
#define PIPELINE_CELL_H


#include <iostream>
#include <vector>
#include <ANN/ANN.h>
#include "myPoint.h"

using namespace std;


class Cell {

    vector<myPoint *> points;
    ANNkd_tree * kdTree;

public:
    Cell();
    ~Cell();

    void addPoint(myPoint *p);
    myPoint * getPoint(int pos);
    int get_nPoints();
};


#endif //PIPELINE_CELL_H
