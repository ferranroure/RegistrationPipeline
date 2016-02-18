//
// Created by ferran on 18/01/16.
//

#include "noDataStructure.h"


noDataStructure::noDataStructure() {

    points = NULL;
}

noDataStructure::noDataStructure(vector<Point *> *P) {

    points = P;
}

noDataStructure::~noDataStructure() {

}

returnData noDataStructure::calcOneNN(Point *queryPoint, float errEps) {

    double bestDist = DBL_MAX;
    int p_id = -1;

    for (int i = 0; i < points->size(); ++i) {

        double dist = queryPoint->dist(points->at(i));

        if(dist < bestDist){

            bestDist = dist;
            p_id = i;
        }
    }

    returnData rd;
    rd.sqrDist = bestDist;
    rd.index = p_id;

    return rd;
}

returnData noDataStructure::calcOwnNN(Point *queryPoint) {

    double bestDist = DBL_MAX;
    int p_id = -1;

    for (int i = 0; i < points->size(); ++i) {

        double dist = queryPoint->dist(points->at(i));

        if(dist < bestDist && queryPoint->getIndex() != i){

            bestDist = dist;
            p_id = i;
        }
    }

    returnData rd;
    rd.sqrDist = bestDist;
    rd.index = p_id;

    return rd;
}

vector<returnData> noDataStructure::calcNneigh(Point *queryPoint, int nNeigh) {




    return std::vector<returnData>();
}
