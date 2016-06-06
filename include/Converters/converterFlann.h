//
// Created by ferran on 1/06/16.
//

#ifndef PIPELINE_CONVERTERFLANN_H
#define PIPELINE_CONVERTERFLANN_H

#define DIMENSIONS 3       // Dimensions


#include "../point.h"
#include <flann/flann.hpp>
#include <vector>


class converterFlann {

public:

    converterFlann();
    ~converterFlann();

    float* convertArray(vector<Point *> *P);     // Tranforms a vector<Point> array of floats (return a pointer to a first element)
    flann::Matrix<float> convertPoint(Point *p);                    // Tranforms a Point to an ANNpoint.


};


#endif //PIPELINE_CONVERTERFLANN_H
