//
// Created by ferran on 18/01/16.
//

#include "myGridTree.h"


myGridTree::myGridTree() {

    gridtree = NULL;
    ads = NULL;

    thrsKdtree = 0;
    slotSizeFactor = 0;
    MMD = 0;
    slotsPerDimension = 0;

}

myGridTree::myGridTree(vector<Point *> *P, std::unordered_map<string, string> &params) {

    ads = new converterGridTree();
    vector<myPoint *> points = ads->convertArray(P);


    try {
        diagonal = stof(params["diagonal"]);
        thrsKdtree = stoi(params["thrsKdtree"]);
        slotSizeFactor = stoi(params["slotSizeFactor"]);
        MMD = stof(params["MMD"]);
    }
    catch(...){
        cerr << "myGridTree :: Some parameter can't be readed! Check parameter names in your params file." << endl;
        exit(EXIT_FAILURE);
    }

    slotsPerDimension = diagonal/(MMD*slotSizeFactor);
    gridtree = new GridTree(points, slotsPerDimension, thrsKdtree);
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

    int factor = 1;
    myPoint *nn = NULL;

    while(nn == NULL) {
        nn = gridtree->oneNeighbor(p, diagonal * 0.01 * factor);
        ++factor;
    }

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

//    myPoint *p = new myPoint(queryPoint->getX(), queryPoint->getY(), queryPoint->getZ());
//    p->setIndex(queryPoint->getIndex());
//
//    vector<myPoint*> vnn;
//    int factor = 1;
//    while(vnn.empty()) {
//        vnn = gridtree->neighbors(p, diagonal * 0.01 * factor);
//        ++factor;
//    }
//
//    myPoint *nn = NULL;
//
//    float bestSqrDist = FLT_MAX;
//    for (int i = 0; i < vnn.size(); ++i) {
//        float sqrdist = p->sqrdist(*(vnn.at(i)));
//        if(sqrdist < bestSqrDist){
//            bestSqrDist = sqrdist;
//            nn = vnn.at(i);
//        }
//    }
//
//    returnData rd;
//    if(vnn.empty()){
//        rd.index = -1;
//        rd.sqrDist = FLT_MAX;
//    }
//    else {
//        rd.index = nn->getIndex();
//        Point *pnn = new Point(nn->getX(), nn->getY(), nn->getZ());
//        rd.sqrDist = queryPoint->sqrDist(pnn);
//
//        delete pnn;
//    }
//
//    delete p;
//
//    return rd;
}

vector<returnData> myGridTree::calcNneigh(Point *queryPoint, int nNeigh) {

    return std::vector<returnData>();
}

void myGridTree::printStats() {

//    // for each cell, get kdtree's height.
//    float meanHeight = gridtree->getMeanHeight();
//
////    cout << "Load factor: " << gridtree->getNumElems() / pow(gridtree->getSlotsPerDimension(), 3);
//    cout <<  gridtree->getNumElems() / pow(gridtree->getSlotsPerDimension(), 3) << ";" << meanHeight;
}
