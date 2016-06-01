//
// Created by ferran on 27/05/16.
//

#include "myLibNabo.h"


myLibNabo::myLibNabo() {

    nns = NULL;
}

myLibNabo::~myLibNabo() {

    delete nns;
}

myLibNabo::myLibNabo(vector<Point *> *P) {

    ceig = new converterEigen();

    dataPts = ceig->convertArray(P);

    nns = NNSearchF::createKDTreeLinearHeap(dataPts);
}

returnData myLibNabo::calcOneNN(Point *queryPoint, float errEps) {

}

returnData myLibNabo::calcOwnNN(Point *queryPoint) {

}

vector<returnData> myLibNabo::calcNneigh(Point *queryPoint, int nNeigh) {

}

returnData myLibNabo::findPair(Point *queryPoint, float dist) {

}

void myLibNabo::printStats() {

}