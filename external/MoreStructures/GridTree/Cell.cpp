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

void Cell::kdtreezation(int thsPoints) {

//    if(points.size() > thsPoints){
    if(!points.empty()){
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
