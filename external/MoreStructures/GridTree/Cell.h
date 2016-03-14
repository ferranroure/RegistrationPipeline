//
// Created by ferran on 4/03/16.
//

#ifndef PIPELINE_CELL_H
#define PIPELINE_CELL_H


#include <iostream>
#include <vector>
#include <ANN/ANN.h>
#include "myPoint.h"

#define DIMENSIONS 3

using namespace std;


class Cell {

    vector<myPoint *> points;
    ANNkd_tree * kdTree;
    ANNpointArray dataPts;

public:
    Cell();
    ~Cell();

    void addPoint(myPoint *p);
    myPoint * getPoint(int pos);
    int get_nPoints();
    ANNkd_tree * getKdtree();
    void kdtreezation(int thsPoints);
    bool isKdtreezed();
    ANNpointArray convertArray(vector<myPoint *> &P);

};


#endif //PIPELINE_CELL_H
