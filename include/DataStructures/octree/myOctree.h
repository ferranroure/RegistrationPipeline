//
// Created by ferran on 15/01/16.
//

#ifndef MYOCTREE_H
#define MYOCTREE_H


#include "../IDataStructure.h"
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <float.h>
#include <math.h>
#include "../../point.h"
#include "../../AdapterPCL.h"

class myOctree : public IDataStructure {

public:

    // Elements -----------------------------------------------------------------------
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> * octree;
    AdapterPCL apcl;

    // Methods ------------------------------------------------------------------------
    myOctree();                                             // Constructor.
    myOctree(vector<Point *> *P, float voxelRes);
    ~myOctree();

    returnData calcOneNN(Point *queryPoint, float errEps);
    returnData calcOwnNN(Point *queryPoint);
    vector<returnData> calcNneigh(Point *queryPoint, int nNeigh);
};


#endif //MYOCTREE_H
