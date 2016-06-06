//
// Created by ferran on 1/06/16.
//

#include "converterFlann.h"

converterFlann::converterFlann(){

}

converterFlann::~converterFlann(){

}

float* converterFlann::convertArray(vector<Point *> *P){

    float points[P->size()*DIMENSIONS];

    int i = 0;
    while (i < P->size()){

        points[i] = (float)P->at(i)->getX();
        points[i+1] = (float)P->at(i)->getY();
        points[i+2] = (float)P->at(i)->getZ();

        i = i+3;
    }

    return points;
}

flann::Matrix<float> converterFlann::convertPoint(Point *p){

    float p_arr[3];

    p_arr[0] = p->getX();
    p_arr[1] = p->getY();
    p_arr[2] = p->getZ();

    flann::Matrix<float> point(p_arr, 1, 2  );

    return point;

}