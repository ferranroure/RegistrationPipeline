//
// Created by yago on 16/02/26.
//

#ifndef PIPELINE_MYSKIPOCTREE_H
#define PIPELINE_MYSKIPOCTREE_H


#include "../IDataStructure.h"
#include "../../../external/MoreStructures/SkipOctree/SkipOctree.h"
#include "../../Converters/converterYago.h"
#include "../../point.h"

class mySkipOctree : public IDataStructure{

public:

    // Elements ----------------------------------------------------------------
    SkipOctree *cOctree;
    converterYago *cyg;
    float diagonal;


    // Methods -----------------------------------------------------------------

    mySkipOctree();
    mySkipOctree(vector<Point *> *P, float _diag);
    ~mySkipOctree();

    returnData calcOneNN(Point *queryPoint, float errEps);
    returnData calcOwnNN(Point *queryPoint);
    vector<returnData> calcNneigh(Point *queryPoint, int nNeigh);
};


#endif //PIPELINE_MYSkipOCTREE_H
