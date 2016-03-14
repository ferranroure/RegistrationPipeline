//
// Created by ferran on 18/01/16.
//

#ifndef PIPELINE_MYGRIDTREE_H
#define PIPELINE_MYGRIDTREE_H


#include "../IDataStructure.h"
#include "../../point.h"
#include "../../Converters/converterGridTree.h"
#include "../../../external/MoreStructures/GridTree/GridTree.h"
#include "../../../external/MoreStructures/GridTree/myPoint.h"


class myGridTree : public IDataStructure {


public:

    // Elements -----------------------------------------------------------
    GridTree *gridtree;
    converterGridTree *ads;
    float diagonal;

    // Methods ------------------------------------------------------------
    myGridTree();
    myGridTree(vector<Point *> *P, float _diag);
    ~myGridTree();

    returnData calcOneNN(Point *queryPoint, float errEps);
    returnData calcOwnNN(Point *queryPoint);
    vector<returnData> calcNneigh(Point *queryPoint, int nNeigh);

    void printStats();

};


#endif //PIPELINE_MYGRIDTREE_H
