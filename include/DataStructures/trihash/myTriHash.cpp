//
// Created by ferran on 18/01/16.
//

#include "myTriHash.h"


myTriHash::myTriHash() {

    trihash = NULL;
    ads = NULL;

}

myTriHash::myTriHash(vector<Point *> *P, float _diag) {

    diagonal = _diag;
    ads = new converterYago();
    vector<Element *> points = ads->convertArray(P);
    trihash = new TriHash(points);
}

myTriHash::~myTriHash() {

    delete trihash;
    delete ads;
}

returnData myTriHash::calcOneNN(Point *queryPoint, float errEps) {

    point3D p(queryPoint->getX(), queryPoint->getY(), queryPoint->getZ());

    vector<Element*> vnn;
    vnn = trihash->neigbors(p, errEps);

    Element *nn = NULL;


    float bestDist = FLT_MAX;
    for (int i = 0; i < vnn.size(); ++i) {
        float dist = p.dist(vnn.at(i)->getPoint());
        if(dist<bestDist && dist <= errEps){
            bestDist = dist;
            nn = vnn.at(i);
        }
    }

//    Element * nn = trihash->nearestNeighbor(p);

    returnData rd;
    if(vnn.empty()){
        rd.index = -1;
        rd.sqrDist = FLT_MAX;
    }
    else {
        rd.index = nn->getIndex();
        Point *pnn = new Point(nn->getPoint().getX(), nn->getPoint().getY(), nn->getPoint().getZ());
        float dist = queryPoint->dist(pnn);
        rd.sqrDist = dist * dist;
        delete pnn;
    }

    return rd;
}

returnData myTriHash::calcOwnNN(Point *queryPoint) {

    point3D p(queryPoint->getX(), queryPoint->getY(), queryPoint->getZ());

//    cout << diagonal << endl;

    vector<Element*> vnn;
    int factor = 1;
    while(vnn.empty()) {
        vnn = trihash->neigbors(p, diagonal * 0.01 * factor);
        ++factor;
    }
    Element *nn = NULL;

    float bestDist = FLT_MAX;
    for (int i = 0; i < vnn.size(); ++i) {
        float dist = p.dist(vnn.at(i)->getPoint());
        if(dist<bestDist){
            bestDist = dist;
            nn = vnn.at(i);
        }
    }

//    Element * nn = trihash->nearestNeighbor(p);

    returnData rd;
    if(vnn.empty()){
        rd.index = -1;
        rd.sqrDist = FLT_MAX;
    }
    else {
        rd.index = nn->getIndex();
        Point *pnn = new Point(nn->getPoint().getX(), nn->getPoint().getY(), nn->getPoint().getZ());
        float dist = queryPoint->dist(pnn);
        rd.sqrDist = dist * dist;
        delete pnn;
    }

    return rd;
}

vector<returnData> myTriHash::calcNneigh(Point *queryPoint, int nNeigh) {

     return std::vector<returnData>();
}

void myTriHash::printStats() {

    cout << "Load factor: " << trihash->getNumElems() / pow(trihash->getSlotsPerDimension(), 3);
}
