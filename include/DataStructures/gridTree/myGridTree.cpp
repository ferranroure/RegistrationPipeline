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
//    vector<myPoint *> points = ads->convertArray(P);


    try {
        diagonal = stof(params["diagonal"]);
//        thrsKdtree = stoi(params["thrsKdtree"]);
//        slotSizeFactor = stoi(params["slotSizeFactor"]);
        DStype = params["internDS"];
        MMD = stof(params["MMD"]);
    }
    catch(...){
        cerr << "myGridTree :: Some parameter can't be readed! Check parameter names in your params file." << endl;
        exit(EXIT_FAILURE);
    }

//    slotsPerDimension = diagonal/(MMD*slotSizeFactor);
    slotsPerDimension = slotSizeFactor;
    gridtree = new GridTree(P, DStype, diagonal);
}

myGridTree::~myGridTree() {

    delete gridtree;
    delete ads;
}

returnData myGridTree::calcOneNN(Point *queryPoint, float eps) {


//    cout << "ONE NN ----------------------------" << endl;
    Point *nn = gridtree->oneNeighbor(queryPoint, eps);

    returnData rd;
    if(nn == NULL){
        rd.index = -1;
        rd.sqrDist = FLT_MAX;
    }
    else {
        rd.index = nn->getIndex();
        rd.sqrDist = queryPoint->sqrDist(nn);
    }

    return rd;
}

returnData myGridTree::calcOwnNN(Point *queryPoint) {

    int factor = 1;
    Point *nn = NULL;

    while(nn == NULL) {
        nn = gridtree->oneNeighbor(queryPoint, diagonal * 0.01 * factor);
        ++factor;
    }

    returnData rd;
    if(nn == NULL){
        rd.index = -1;
        rd.sqrDist = FLT_MAX;
    }
    else {
        rd.index = nn->getIndex();
        rd.sqrDist = queryPoint->sqrDist(nn);
    }

    return rd;
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
