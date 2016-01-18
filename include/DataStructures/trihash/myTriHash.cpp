//
// Created by ferran on 18/01/16.
//

#include "myTriHash.h"


myTriHash::myTriHash() {

    trihash = NULL;
    ads = NULL;

}

myTriHash::myTriHash(vector<Point *> *P, float _MMD) {

    vector<Element *> points = ads->convertArray(P);
    MMD = _MMD;
    trihash = new TriHash(points);
    ads = new AdapterDataStruct();
}

myTriHash::~myTriHash() {

    delete trihash;
    delete ads;
}

returnData myTriHash::calcOneNN(Point *queryPoint) {

    point3D p(queryPoint->getX(), queryPoint->getY(), queryPoint->getZ());

    vector<Element *> res = trihash->neigbors(p, MMD*5);

    Element *nn = res.at(0);

    returnData rd;
    rd.index = nn->getIndex();
    Point * pnn = new Point(nn->getPoint().getX(), nn->getPoint().getY(), nn->getPoint().getZ());
    float dist = queryPoint->dist(pnn);
    rd.sqrDist = dist*dist;
    delete pnn;

    return rd;
}

returnData myTriHash::calcOwnNN(Point *queryPoint) {
    return returnData();
}

vector<returnData> myTriHash::calcNneigh(Point *queryPoint, int nNeigh) {




    return std::vector<returnData>();
}
