//
// Created by ferran on 18/01/16.
//

#ifndef PIPELINE_noDataStructure_H
#define PIPELINE_noDataStructure_H


#include "../IDataStructure.h"
#include "../../point.h"

class noDataStructure : public IDataStructure {

public:

    // Elements -----------------------------------------------------------
    // NO Elements
    vector<Point*> *points;

    // Methods ------------------------------------------------------------
    noDataStructure();
    noDataStructure(vector<Point *> *P);
    ~noDataStructure();

    returnData calcOneNN(Point *queryPoint, float errEps);
    returnData calcOwnNN(Point *queryPoint);
    vector<returnData> calcNneigh(Point *queryPoint, int nNeigh);

    void printStats();

};


#endif //PIPELINE_noDataStructure_H
