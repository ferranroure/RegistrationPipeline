//
// Created by ferran on 4/03/16.
//

#ifndef PIPELINE_CELL_H
#define PIPELINE_CELL_H

#include "Eigen/Core"


#include <iostream>
#include <vector>
//#include "myPoint.h"
#include "../../../include/point.h"
#include "../../../include/DataStructures/IDataStructure.h"
#include "../../../include/DataStructures/kdtree/mykdtree.h"
#include "../../../include/DataStructures/compressedOctree/myCompressedOctree.h"
#include "../../../include/DataStructures/S4PCSkdtree/myS4PCSkdtree.h"
#include "../../../include/DataStructures/trihash/myTriHash.h"


#define DIMENSIONS 3

using namespace std;


class Cell {

    vector<Point *> *points;
    IDataStructure * dataStructure;

public:
    Cell();
    ~Cell();

    void addPoint(Point *p);
    Point * getPoint(int pos);
    int get_nPoints();
    IDataStructure * getDataStructure();
    void kdtreezation(string dsType, int thsPoints, float diagonal);
    bool isKdtreezed();
    bool empty();

};


#endif //PIPELINE_CELL_H
