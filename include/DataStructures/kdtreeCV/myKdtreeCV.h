//
// Created by ferran on 9/06/16.
//

#ifndef PIPELINE_MYKDTREECV_H
#define PIPELINE_MYKDTREECV_H


#include "../IDataStructure.h"
#include "../../Converters/converterKdtreeCV.h"

using namespace std;

class myKdtreeCV : public IDataStructure {


public:


// Elements ---------------------------------------
    cv::Mat * dataPts;
    cv::flann::Index * kdtree;

    converterKdtreeCV *ckdcv;


    myKdtreeCV();
    myKdtreeCV(vector<Point*> *P);
    ~myKdtreeCV();

    returnData calcOneNN(Point *queryPoint, float errEps);

    returnData calcOwnNN(Point *queryPoint);

    vector<returnData> calcNneigh(Point *queryPoint, int nNeigh);

    void printStats();




};


#endif //PIPELINE_MYKDTREECV_H
