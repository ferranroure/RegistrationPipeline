//
// Created by ferran on 1/06/16.
//

#include "myFlann.h"


myFlann::myFlann() {

}

myFlann::myFlann(vector<Point *> *P) {

    dataPts = new flann::Matrix<float>(cfln->convertArray(P), P->size(), 3);

    kdtree = new flann::Index<flann::L2<int> >(*dataPts, flann::KDTreeIndexParams(8));
    kdtree->buildIndex();
}

myFlann::~myFlann() {

}

returnData myFlann::calcOneNN(Point *queryPoint, float errEps) {
    return returnData();
}

returnData myFlann::calcOwnNN(Point *queryPoint) {
    return returnData();
}

vector<returnData> myFlann::calcNneigh(Point *queryPoint, int nNeigh) {
    return std::vector<returnData>();
}

void myFlann::printStats() {

}

