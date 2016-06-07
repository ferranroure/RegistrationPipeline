//
// Created by ferran on 1/06/16.
//

#include "converterFlann.h"

converterFlann::converterFlann(){

}

converterFlann::~converterFlann(){

}

flann::Matrix<float> * converterFlann::convertArray(vector<Point *> *P){

    float points[P->size()*DIMENSIONS];

    int j = 0;
    for (int i = 0; i<P->size(); i++){

        points[j] = (float)P->at(i)->getX(); j++;
        points[j] = (float)P->at(i)->getY(); j++;
        points[j] = (float)P->at(i)->getZ(); j++;

    }

    P->at(0)->print();
    cout << points[0] << " " << points[1] << " " << points[2] << endl;

    flann::Matrix<float> *nari = new flann::Matrix<float>(points, P->size(), DIMENSIONS);

    cout << *nari[0][0] << " " << *nari[0][1] << " " << *nari[0][2] << endl;

    exit(0);
    return new flann::Matrix<float>(points, P->size(), DIMENSIONS);
}

flann::Matrix<float> converterFlann::convertPoint(Point *p){

    float p_arr[3];

    p_arr[0] = p->getX();
    p_arr[1] = p->getY();
    p_arr[2] = p->getZ();

    flann::Matrix<float> point(p_arr, 1, 3);

    return point;

}