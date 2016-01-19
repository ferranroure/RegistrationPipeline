//
// Created by ferran on 18/01/16.
//

#include "myTriHash.h"


myTriHash::myTriHash() {

    trihash = NULL;
    ads = NULL;

}

myTriHash::myTriHash(vector<Point *> *P, float _diag) {

    vector<Element *> points = ads->convertArray(P);
    diagonal = _diag;
    trihash = new TriHash(points);
    ads = new AdapterDataStruct();
}

myTriHash::~myTriHash() {

    delete trihash;
    delete ads;
}

returnData myTriHash::calcOneNN(Point *queryPoint) {

    point3D p(queryPoint->getX(), queryPoint->getY(), queryPoint->getZ());


    Element * nn = trihash->nearestNeighbor(p);

    returnData rd;
    rd.index = nn->getIndex();
    Point * pnn = new Point(nn->getPoint().getX(), nn->getPoint().getY(), nn->getPoint().getZ());
    float dist = queryPoint->dist(pnn);
    rd.sqrDist = dist*dist;
    delete pnn;

    return rd;
}

returnData myTriHash::calcOwnNN(Point *queryPoint) {

    return calcOneNN(queryPoint);
}

vector<returnData> myTriHash::calcNneigh(Point *queryPoint, int nNeigh) {




    return std::vector<returnData>();
}
