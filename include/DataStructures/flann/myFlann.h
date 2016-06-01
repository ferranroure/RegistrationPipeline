//
// Created by ferran on 1/06/16.
//

#ifndef PIPELINE_MYFLANN_H
#define PIPELINE_MYFLANN_H


#include "../IDataStructure.h"
#include <flann/flann.hpp>

class myFlann : IDataStructure {


public:


// Elements ---------------------------------------
    flann::Matrix<float> dataPts;
    flann::Index<flann::L2<int> > *kdtree;

    converterFlann *cfln;


    myFlann();
    myFlann(vector<Point*> *P);
    ~myFlann();

private:
    virtual returnData calcOneNN(Point *queryPoint, float errEps);

    virtual returnData calcOwnNN(Point *queryPoint);

    virtual vector<returnData> calcNneigh(Point *queryPoint, int nNeigh);

    virtual void printStats();



};


#endif //PIPELINE_MYFLANN_H
