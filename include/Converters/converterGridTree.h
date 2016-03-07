//
// Created by ferran on 4/03/16.
//

#ifndef PIPELINE_CONVERTERGRIDTREE_H
#define PIPELINE_CONVERTERGRIDTREE_H


#include "../../external/MoreStructures/GridTree/myPoint.h"
#include "../point.h"

class converterGridTree {

public:

    converterGridTree();
    ~converterGridTree();

    vector<myPoint *> convertArray(vector<Point*> *P);
    myPoint * convertPoint(Point *p);
};


#endif //PIPELINE_CONVERTERGRIDTREE_H
