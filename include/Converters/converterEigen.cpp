//
// Created by ferran on 27/05/16.
//

#include "converterEigen.h"

converterEigen::converterEigen() {

}

converterEigen::~converterEigen() {

}

MatrixXf converterEigen::convertArray(vector<Point *> *P){

    return MatrixXf::Random(3, 100);
}

VectorXf converterEigen::convertPoint(Point *p){

    return VectorXf::Random(3);
}