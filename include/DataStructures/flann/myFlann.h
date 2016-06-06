//
// Created by ferran on 1/06/16.
//

#ifndef PIPELINE_MYFLANN_H
#define PIPELINE_MYFLANN_H


#include "../IDataStructure.h"
#include "../../Converters/converterFlann.h"
#include <flann/flann.hpp>


class myFlann : public IDataStructure {


public:


// Elements ---------------------------------------
    flann::Matrix<float> *dataPts;
    flann::Index<flann::L2<float> > *kdtree;

    converterFlann *cfln;


    myFlann();
    myFlann(vector<Point*> *P);
    ~myFlann();

    returnData calcOneNN(Point *queryPoint, float errEps);

    returnData calcOwnNN(Point *queryPoint);

    vector<returnData> calcNneigh(Point *queryPoint, int nNeigh);

    void printStats();



};


#endif //PIPELINE_MYFLANN_H
