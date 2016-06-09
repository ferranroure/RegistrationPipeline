//
// Created by ferran on 9/06/16.
//

#ifndef PIPELINE_CONVERTERKDTREECV_H
#define PIPELINE_CONVERTERKDTREECV_H


#include <cxcore.h>
#include "../point.h"
#include <vector>

class converterKdtreeCV {


public:
    converterKdtreeCV();
    ~converterKdtreeCV();

    cv::Mat * convertArray(vector<Point *> *P);
    vector<float> convertPoint(Point *p);

};


#endif //PIPELINE_CONVERTERKDTREECV_H
