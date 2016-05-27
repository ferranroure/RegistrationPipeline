//
// Created by ferran on 27/05/16.
//

#include "myLibNabo.h"


myLibNabo::myLibNabo() {

    nns = NULL;
}

myLibNabo::myLibNabo(vector<Point *> *P) {

    ceig = new converterEigen();

    dataPts = ceig->convertArray(P);

    nns = NNSearchF::createKDTreeLinearHeap(dataPts);
}