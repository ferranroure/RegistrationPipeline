//
// Created by ferran on 4/03/16.
//

#include "Cell.h"

Cell::Cell() {

    kdTree = NULL;

}

Cell::~Cell() {


    if(kdTree!=NULL){
        annDeallocPts(dataPts);
        delete kdTree;
    }


    for (int i = 0; i < points.size(); ++i) {

        delete points.at(i);
    }

    annClose();

}
//BUN
//0.0383251;1.02422 / 5;;;0.0722986 -> 0 kdtrees
//0.035234;1.02422 / 6.19456;;;0.0693265 -> 5
//0.0427568;1.02422 / 6;;;0.0745916 -> 10
//0.024281;1.02422 / 8;;;0.0880853 -> 50

//BUDDHA
//0.088366;1.05356 / 4;;;0.094919 -> 0
//0.0703161;1.05356 / 6.06987;;;0.0926789 -> 5
//0.068207;1.05356 / 6;;;0.098384 -> 10
//0.0503919;1.05356 / 8;;;0.10574 -> 50

//FAST_SPIRAL
//0.0639679;1.01566 / 13.9438;;;0.235115 -> 0
//0.0695798;1.01566 / 18.1022;;;0.274241 -> 5
//0.0647171;1.01566 / 18.1912;;;0.238611 -> 10
//0.081522;1.01566 / 18.7656;;;0.278072 -> 50


void Cell::kdtreezation(int thsPoints) {

    if(points.size() > thsPoints){
//    if(!points.empty()){
        dataPts = convertArray(points);         // Converting data from "points" to "dataPts"

        kdTree = new ANNkd_tree(dataPts, points.size(), DIMENSIONS);

    }
}


bool Cell::isKdtreezed() {

    return kdTree != NULL;
//    return false;
}

void Cell::addPoint(myPoint *p) {

    points.push_back(p);
}

int Cell::get_nPoints() {

    return points.size();
}


ANNkd_tree *Cell::getKdtree() {

    return kdTree;
}

myPoint *Cell::getPoint(int pos) {

    return points.at(pos);
}

ANNpointArray Cell::convertArray(vector<myPoint *> &P){

    ANNpointArray pa;
    pa = annAllocPts(P.size(), DIMENSIONS);
    for (int i = 0; i < P.size(); ++i) {

        pa[i][0] = P.at(i)->getX();
        pa[i][1] = P.at(i)->getY();
        pa[i][2] = P.at(i)->getZ();
    }

    return pa;
}
