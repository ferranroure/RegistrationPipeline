//
// Created by ferran on 1/06/16.
//

#include "myFlann.h"


myFlann::myFlann() {

    dataPts = NULL;
    kdtree = NULL;
    cfln = NULL;

}

myFlann::myFlann(vector<Point *> *P) {



    cfln = new converterFlann();


    dataPts = cfln->convertArray(P);

    kdtree = new flann::KDTreeSingleIndex<flann::L2<float> >(*dataPts, flann::KDTreeSingleIndexParams());
    kdtree->buildIndex();

}

myFlann::~myFlann() {

    delete dataPts;
    delete kdtree;
}

returnData myFlann::calcOneNN(Point *queryPoint, float errEps) {

    // distance
    double sqrDist = 0;

    flann::Matrix<float> q = cfln->convertPoint(queryPoint);

    flann::Matrix<int> indices(new int[q.rows], q.rows, 1);
    flann::Matrix<float> dists(new float[q.rows], q.rows, 1);

    if(kdtree->size() > 0){

        //do a knn search, using 128 checks
        kdtree->knnSearch(q, indices, dists, 1, flann::SearchParams());

        sqrDist = dists[0][0];
    }
    else{

        cerr << "ERROR: The kdtree is empty!" << endl;
        exit(EXIT_FAILURE);
    }

    returnData rd;
    rd.sqrDist = sqrDist;
    rd.index = indices[0][0];


    return rd;
}

returnData myFlann::calcOwnNN(Point *queryPoint) {

    // distance
    double sqrDist = 0;
    int nQ = 2;

    flann::Matrix<float> q = cfln->convertPoint(queryPoint);


    flann::Matrix<int> indices(new int[q.rows*nQ], q.rows, nQ);
    flann::Matrix<float> dists(new float[q.rows*nQ], q.rows, nQ);


    if(kdtree->size() > 0){


        //do a knn search, using 128 checks
        kdtree->knnSearch(q, indices, dists, nQ, flann::SearchParams());

//        cout << *(dists[0]) << " " << *(dists.ptr()) << endl;
//        cout << dists[0][0] << " " << dists[1][0] << endl;
        sqrDist = dists[0][1];
    }
    else{

        cerr << "ERROR: The kdtree is empty!" << endl;
        exit(EXIT_FAILURE);
    }

    returnData rd;
    rd.sqrDist = sqrDist;
    rd.index = indices[0][1];

//    cout << queryPoint->getIndex() << endl;
//    cout << rd.index << endl;


    return rd;
}

vector<returnData> myFlann::calcNneigh(Point *queryPoint, int nNeigh) {
    return std::vector<returnData>();
}

void myFlann::printStats() {

}

