//
// Created by ferran on 8/07/16.
//

#ifndef PIPELINE_MY4PCSKDTREE_H
#define PIPELINE_MY4PCSKDTREE_H

#include <iostream>
#include <vector>

#include "Eigen/Core"

#include <accelerators/kdtree.h>
#include <accelerators/bbox.h>
#include "../IDataStructure.h"
#include "../../point.h"


class myS4PCSkdtree : public IDataStructure{


public:

    // Elements -----------------------------------------------
    Super4PCS::KdTree<double> *kdtree;
    vector<Point*> *dataPts;
    float diagonal;


    // Methods ------------------------------------------------
    myS4PCSkdtree();
    myS4PCSkdtree(vector<Point*> *P, float _diag);
    ~myS4PCSkdtree();

    virtual returnData calcOneNN(Point *queryPoint, float errEps);        // Finds Nearest Neighbor distance to a given QueryPoint.
    virtual returnData calcOwnNN(Point *queryPoint);        // Finds a real NN (not itself) of a given QueryPoint from the same point cloud. (used for MMD)
    virtual vector<returnData>  calcNneigh(Point *queryPoint, int nNeigh);

    void printStats();

};


#endif //PIPELINE_MY4PCSKDTREE_H
