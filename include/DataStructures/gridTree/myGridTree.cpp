//
// Created by ferran on 18/01/16.
//

#include "myGridTree.h"


myGridTree::myGridTree() {

    gridtree = NULL;
    ads = NULL;

}

myGridTree::myGridTree(vector<Point *> *P, float _diag) {

    diagonal = _diag;
    ads = new converterGridTree();
    vector<myPoint *> points = ads->convertArray(P);
    gridtree = new GridTree(points);
}

myGridTree::~myGridTree() {

    delete gridtree;
    delete ads;
}

returnData myGridTree::calcOneNN(Point *queryPoint, float eps) {

    myPoint *p = new myPoint(queryPoint->getX(), queryPoint->getY(), queryPoint->getZ());

//    cout << "ONE NN ----------------------------" << endl;
    myPoint *nn = gridtree->oneNeighbor(p, eps);

    returnData rd;
    if(nn == NULL){
        rd.index = -1;
        rd.sqrDist = FLT_MAX;
    }
    else {
        rd.index = nn->getIndex();
        Point *pnn = new Point(nn->getX(), nn->getY(), nn->getZ());
        rd.sqrDist = queryPoint->sqrDist(pnn);
        delete pnn;
    }

    delete p;

    return rd;
}

returnData myGridTree::calcOwnNN(Point *queryPoint) {


    myPoint *p = new myPoint(queryPoint->getX(), queryPoint->getY(), queryPoint->getZ());
    p->setIndex(queryPoint->getIndex());

    vector<myPoint*> vnn;
    int factor = 1;
    while(vnn.empty()) {
        vnn = gridtree->neighbors(p, diagonal * 0.01 * factor);
        ++factor;
    }

    myPoint *nn = NULL;

    float bestSqrDist = FLT_MAX;
    for (int i = 0; i < vnn.size(); ++i) {
        float sqrdist = p->sqrdist(*(vnn.at(i)));
        if(sqrdist < bestSqrDist){
            bestSqrDist = sqrdist;
            nn = vnn.at(i);
        }
    }

    returnData rd;
    if(vnn.empty()){
        rd.index = -1;
        rd.sqrDist = FLT_MAX;
    }
    else {
        rd.index = nn->getIndex();
        Point *pnn = new Point(nn->getX(), nn->getY(), nn->getZ());
        rd.sqrDist = queryPoint->sqrDist(pnn);

        delete pnn;
    }

    delete p;

    return rd;
}

vector<returnData> myGridTree::calcNneigh(Point *queryPoint, int nNeigh) {

    return std::vector<returnData>();
}

void myGridTree::printStats() {

    // for each cell, get kdtree's height.
    float meanHeight = gridtree->getMeanHeight();

//    cout << "Load factor: " << gridtree->getNumElems() / pow(gridtree->getSlotsPerDimension(), 3);
    cout <<  gridtree->getNumElems() / pow(gridtree->getSlotsPerDimension(), 3) << ";" << meanHeight;
}
