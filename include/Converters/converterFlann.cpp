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

//    int j = 0;
//    for (int i = 0; i<P->size(); i++){
//
//        points[j] = (float)P->at(i)->getX(); j++;
//        points[j] = (float)P->at(i)->getY(); j++;
//        points[j] = (float)P->at(i)->getZ(); j++;
//
//    }

    for(int i=0; i<P->size(); i++){
        points[(i * DIMENSIONS) + 0] = (float)P->at(i)->getX();
        points[(i * DIMENSIONS) + 1] = (float)P->at(i)->getY();
        points[(i * DIMENSIONS) + 2] = (float)P->at(i)->getZ();
    }
//cout <<" hola" << endl;
//    P->at(0)->print();
//    cout << points[0] << " " << points[1] << " " << points[2] << endl;
//
//    flann::Matrix<float> *nari = new flann::Matrix<float>(points, P->size(), DIMENSIONS);
//
//    cout << *(nari->ptr()) << " " << *(nari->ptr()+1) << " " << *(nari->ptr()+2) << endl;
//
//    exit(0);
    return new flann::Matrix<float>(points, P->size(), DIMENSIONS);
}

flann::Matrix<float> converterFlann::convertPoint(Point *p){

    float p_arr[3];

    p_arr[0] = p->getX();
    p_arr[1] = p->getY();
    p_arr[2] = p->getZ();


    flann::Matrix<float> point(p_arr, 1, 3);

//    p->print();
//    cout << p_arr[0] << " " << p_arr[1] << " " << p_arr[2] << endl;
//    cout << *point.ptr() << " " << *(point.ptr()+1) << " " << *(point.ptr()+2) <<  endl;
//
//
//    exit(0);
    return point;

}