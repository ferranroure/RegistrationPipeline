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


//    int nn = 3;
//
//    Matrix<float> dataset;
//    Matrix<float> query;
//
//    Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
//    Matrix<float> dists(new float[query.rows*nn], query.rows, nn);
//
//    // construct an randomized kd-tree index using 4 kd-trees
//    Index<L2<float> > index(dataset, flann::KDTreeIndexParams(4));
//    index.buildIndex();
//
//    // do a knn search, using 128 checks
//    index.knnSearch(query, indices, dists, nn, flann::SearchParams(128));
//
//    delete[] dataset.ptr();
//    delete[] query.ptr();
//    delete[] indices.ptr();
//    delete[] dists.ptr();

    cfln = new converterFlann();

    dataPts = new flann::Matrix<float>(cfln->convertArray(P), P->size(), DIMENSIONS);

    kdtree = new flann::Index<flann::L2<float> >(*dataPts, flann::KDTreeIndexParams(128));
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
        kdtree->knnSearch(q, indices, dists, 1, flann::SearchParams(4));

        sqrDist = *(dists[0]);
    }
    else{

        cerr << "ERROR: The kdtree is empty!" << endl;
        exit(EXIT_FAILURE);
    }

    returnData rd;
    rd.sqrDist = sqrDist;
    rd.index = *(indices[0]);


    return rd;
}

returnData myFlann::calcOwnNN(Point *queryPoint) {

    // distance
    double sqrDist = 0;

    flann::Matrix<float> q = cfln->convertPoint(queryPoint);

    flann::Matrix<int> indices(new int[q.rows*2], q.rows, 2);
    flann::Matrix<float> dists(new float[q.rows*2], q.rows, 2);


    if(kdtree->size() > 0){


        //do a knn search, using 128 checks
        kdtree->knnSearch(q, indices, dists, 2, flann::SearchParams(4));
        sqrDist = *(dists[1]);
    }
    else{

        cerr << "ERROR: The kdtree is empty!" << endl;
        exit(EXIT_FAILURE);
    }

    returnData rd;
    rd.sqrDist = sqrDist;
    rd.index = *(indices[1]);


    return rd;
}

vector<returnData> myFlann::calcNneigh(Point *queryPoint, int nNeigh) {
    return std::vector<returnData>();
}

void myFlann::printStats() {

}

