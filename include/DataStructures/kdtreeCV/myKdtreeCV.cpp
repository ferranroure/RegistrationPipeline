//
// Created by ferran on 1/06/16.
//

#include "myKdtreeCV.h"


myKdtreeCV::myKdtreeCV() {

    dataPts = NULL;
    kdtree = NULL;
    ckdcv = NULL;

}

myKdtreeCV::myKdtreeCV(vector<Point *> *P) {



    ckdcv = new converterKdtreeCV();


    dataPts = ckdcv->convertArray(P);

    kdtree = new cv::flann::Index(*dataPts, cv::flann::KDTreeIndexParams(16));

}

myKdtreeCV::~myKdtreeCV() {

    delete dataPts;
    delete kdtree;
}

returnData myKdtreeCV::calcOneNN(Point *queryPoint, float errEps) {

    // distance
    double sqrDist = 0;

    vector<float> q = ckdcv->convertPoint(queryPoint);

    vector<int> indices(1);
    vector<float> dists(1);

    //do a knn search, using 128 checks
    kdtree->knnSearch(q, indices, dists, 1, cv::flann::SearchParams(16));

    sqrDist = dists.at(0);

    returnData rd;
    rd.sqrDist = sqrDist;
    rd.index = indices.at(0);


    return rd;
}

returnData myKdtreeCV::calcOwnNN(Point *queryPoint) {

    // distance
    double sqrDist = 0;

    vector<float> q = ckdcv->convertPoint(queryPoint);

    vector<int> indices(2);
    vector<float> dists(2);

    //do a knn search, using 128 checks
    kdtree->knnSearch(q, indices, dists, 2, cv::flann::SearchParams(16));

    sqrDist = dists.at(1);

    returnData rd;
    rd.sqrDist = sqrDist;
    rd.index = indices.at(1);


    return rd;
}

vector<returnData> myKdtreeCV::calcNneigh(Point *queryPoint, int nNeigh) {
    return std::vector<returnData>();
}

void myKdtreeCV::printStats() {

}

