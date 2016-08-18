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
#include <unordered_map>


class myGridTree : public IDataStructure {


public:

    // Elements -----------------------------------------------------------
    GridTree *gridtree;
    converterGridTree *ads;
    float diagonal;
    int thrsKdtree;
    int slotSizeFactor;
    float MMD;
    int slotsPerDimension;
    string DStype;

    // Methods ------------------------------------------------------------
    myGridTree();
    myGridTree(vector<Point *> *P, std::unordered_map<string, string> &params);
    ~myGridTree();

    returnData calcOneNN(Point *queryPoint, float eps);
    returnData calcOwnNN(Point *queryPoint);
    vector<returnData> calcNneigh(Point *queryPoint, int nNeigh);

    void printStats();

};


#endif //PIPELINE_MYGRIDTREE_H
